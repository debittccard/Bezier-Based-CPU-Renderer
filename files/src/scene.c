#include "../include/scene.h"
#include <stdio.h>
#include <time.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include "../include/math.h"
#include "../include/geometry.h"
#include "../include/renderer.h"

int culled_count = 0;
static int rendered_count = 0;
static int shadowed_count = 0;
static double render_start_ms = 0.0;
static double shadow_start_ms = 0.0;

static double timespec_to_ms(const struct timespec *t){
    return (double)t->tv_sec*1000.0 + (double)t->tv_nsec/1e6;
}

static void *shadow_thread_func(void *arg){
    ShadowJob *job = (ShadowJob *)arg;

    int total = SHADOW_H * SHADOW_W;
    for(int i=0; i<total; i++) job->shadow_local[i] = FAR_DEPTH;

    for(int i = job->start; i < job->end; i++){
        TubeEntry *te = &job->tubes[i];
        if(!aabb_in_frustum_planes(te->bounds, job->planes)) continue;

        float angle = i * job->angle_step;
        Mat4 model = matMult(
            mat_translate(job->tx, job->ty, job->tz),
            matMult(
                matMult(mat_rotate_x(job->rx * angle),
                    matMult(mat_rotate_y(job->ry * angle),
                            mat_rotate_z(job->rz * angle))),
                mat_scale(job->scale, job->scale, job->scale)));

        Mat4 light_mvp = matMult(job->light_vp, model);
        render_tube_shadow_local(te->world_curve, light_mvp,
                                 te->tube_props.segments,
                                 te->tube_props.sides,
                                 te->tube_props.radius,
                                 job->shadow_local);

        int n = __sync_add_and_fetch(&shadowed_count, 1);
        int interval = job->tube_count / 20;
        if(interval>0 && n%interval==0){
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            double elapsed = timespec_to_ms(&now) - shadow_start_ms;
            double eta = (elapsed/n) * (job->tube_count - n);
            printf("shadowed %d / %d | ETA %.1fs\n", n, job->tube_count, eta/1000.0);
        }
    }
    return NULL;
}

static void *render_thread(void *arg){
    RenderJob *job = (RenderJob *)arg;

    for(int i = job->start; i < job->end; i++){
        TubeEntry *te = &job->tubes[i];
        TubeProperties props = te->tube_props;
        BezierCubic world_curve = te->world_curve;

        Vec3 mid = {
            (world_curve.p0.x+world_curve.p3.x)*0.5f,
            (world_curve.p0.y+world_curve.p3.y)*0.5f,
            (world_curve.p0.z+world_curve.p3.z)*0.5f
        };
        float dx=mid.x-job->eye.x, dy=mid.y-job->eye.y, dz=mid.z-job->eye.z;
        if(dx*dx+dy*dy+dz*dz > job->far_plane*job->far_plane) continue;

        if(!aabb_in_frustum_planes(te->bounds, job->planes)){
            __sync_fetch_and_add(&culled_count, 1);
            continue;
        }

        render_tube(world_curve, job->vp, props.segments, props.sides, props.radius,
                    te->color_r, te->color_g, te->color_b,
                    job->light, &job->tb);

        int n = __sync_add_and_fetch(&rendered_count, 1);
        int interval = job->tubes_total / 20;
        if(interval>0 && n%interval==0){
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            double elapsed = timespec_to_ms(&now) - render_start_ms;
            double avg = elapsed/n;
            double eta = avg*(job->tubes_total-n);
            printf("rendered %d / %d | avg %.2fms | ETA %.1fs\n",
                   n, job->tubes_total, avg, eta/1000.0);
        }
    }
    return NULL;
}

static void shadow_pass(SceneConfig *scene, Plane light_planes[6]){
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    shadow_start_ms = timespec_to_ms(&now);
    shadowed_count = 0;

    pthread_t threads[NUM_THREADS];
    ShadowJob jobs[NUM_THREADS];
    int per = scene->tube_count / NUM_THREADS;

    for(int t=0; t<NUM_THREADS; t++){
        jobs[t].tubes      = scene->tubes;
        jobs[t].start      = t*per;
        jobs[t].end        = (t==NUM_THREADS-1) ? scene->tube_count : (t+1)*per;
        jobs[t].light_vp   = scene->light_vp;
        jobs[t].tube_count = scene->tube_count;
        jobs[t].rx         = scene->transform.rotate_x;
        jobs[t].ry         = scene->transform.rotate_y;
        jobs[t].rz         = scene->transform.rotate_z;
        jobs[t].scale      = scene->transform.scale;
        jobs[t].tx         = scene->transform.translate_x;
        jobs[t].ty         = scene->transform.translate_y;
        jobs[t].tz         = scene->transform.translate_z;
        jobs[t].angle_step = scene->transform.angle_step;
        memcpy(jobs[t].planes, light_planes, sizeof(Plane)*6);

        jobs[t].shadow_local = malloc(SHADOW_H * SHADOW_W * sizeof(float));
        if(!jobs[t].shadow_local){
            fprintf(stderr, "shadow malloc failed thread %d\n", t);
            return;
        }
        int total = SHADOW_H*SHADOW_W;
        for(int i=0; i<total; i++) jobs[t].shadow_local[i] = FAR_DEPTH;

        pthread_create(&threads[t], NULL, shadow_thread_func, &jobs[t]);
    }

    for(int t=0; t<NUM_THREADS; t++)
        pthread_join(threads[t], NULL);

    // merge into global shadow_map
    for(int y=0; y<SHADOW_H; y++){
        for(int x=0; x<SHADOW_W; x++){
            float min = FAR_DEPTH;
            int idx = y*SHADOW_W+x;
            for(int t=0; t<NUM_THREADS; t++)
                if(jobs[t].shadow_local[idx] < min)
                    min = jobs[t].shadow_local[idx];
            shadow_map[y][x] = min;
        }
    }

    for(int t=0; t<NUM_THREADS; t++)
        free(jobs[t].shadow_local);

    clock_gettime(CLOCK_MONOTONIC, &now);
    printf("shadow pass: %.2f ms\n", timespec_to_ms(&now) - shadow_start_ms);
}

void render_scene(SceneConfig scene){
    struct timespec t_total0, t_total1, t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t_total0);

    Mat4 proj = mat_projection(scene.camera.fov, (float)WIDTH/HEIGHT,
                               scene.camera.near_plane, scene.camera.far_plane);
    Mat4 view = mat_look_at(scene.camera.eye, scene.camera.target, scene.camera.up);
    Mat4 vp   = matMult(proj, view);

    Plane planes[6], light_planes[6];
    extract_frustum_planes(planes, vp);
    extract_frustum_planes(light_planes, scene.light_vp);

    // world transform + AABB cache
    clock_gettime(CLOCK_MONOTONIC, &t0);
    for(int i=0; i<scene.tube_count; i++){
        TubeEntry *te = &scene.tubes[i];
        float angle = i * scene.transform.angle_step;

        Mat4 model = matMult(
            mat_translate(scene.transform.translate_x,
                          scene.transform.translate_y,
                          scene.transform.translate_z),
            matMult(
                matMult(mat_rotate_x(scene.transform.rotate_x * angle),
                    matMult(mat_rotate_y(scene.transform.rotate_y * angle),
                            mat_rotate_z(scene.transform.rotate_z * angle))),
                mat_scale(scene.transform.scale,
                          scene.transform.scale,
                          scene.transform.scale)));

        Vec4 p0=mat4_mul_vec4(model,(Vec4){te->curve.p0.x,te->curve.p0.y,te->curve.p0.z,1.0f});
        Vec4 p1=mat4_mul_vec4(model,(Vec4){te->curve.p1.x,te->curve.p1.y,te->curve.p1.z,1.0f});
        Vec4 p2=mat4_mul_vec4(model,(Vec4){te->curve.p2.x,te->curve.p2.y,te->curve.p2.z,1.0f});
        Vec4 p3=mat4_mul_vec4(model,(Vec4){te->curve.p3.x,te->curve.p3.y,te->curve.p3.z,1.0f});

        te->world_curve=(BezierCubic){
            {p0.x,p0.y,p0.z},{p1.x,p1.y,p1.z},
            {p2.x,p2.y,p2.z},{p3.x,p3.y,p3.z}
        };
        te->bounds=bezier_bounds(te->world_curve, te->tube_props.radius);
    }
    clock_gettime(CLOCK_MONOTONIC, &t1);
    printf("world transform: %.2f ms\n", timespec_to_ms(&t1)-timespec_to_ms(&t0));

    clock_gettime(CLOCK_MONOTONIC, &t0);
    shadow_pass(&scene, light_planes);
    clock_gettime(CLOCK_MONOTONIC, &t1);
    printf("shadow pass total: %.2f ms\n", timespec_to_ms(&t1)-timespec_to_ms(&t0));

    clock_gettime(CLOCK_MONOTONIC, &t0);
    RenderJob *jobs=(RenderJob *)malloc(NUM_THREADS*sizeof(RenderJob));
    pthread_t threads[NUM_THREADS];
    int per=scene.tube_count/NUM_THREADS;

    for(int t=0; t<NUM_THREADS; t++){
        jobs[t].tubes       = scene.tubes;
        jobs[t].start       = t*per;
        jobs[t].end         = (t==NUM_THREADS-1) ? scene.tube_count : (t+1)*per;
        jobs[t].vp          = vp;
        jobs[t].light       = scene.light;
        jobs[t].tubes_total = scene.tube_count;
        jobs[t].eye         = scene.camera.eye;
        jobs[t].far_plane   = scene.camera.far_plane;
        memcpy(jobs[t].planes, planes, sizeof(planes));
    }

    rendered_count=0; culled_count=0;
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    render_start_ms = timespec_to_ms(&now);

    for(int t=0; t<NUM_THREADS; t++)
        pthread_create(&threads[t], NULL, render_thread, &jobs[t]);
    for(int t=0; t<NUM_THREADS; t++)
        pthread_join(threads[t], NULL);

    free(jobs);

    clock_gettime(CLOCK_MONOTONIC, &t1);
    printf("main render: %.2f ms\n", timespec_to_ms(&t1)-timespec_to_ms(&t0));

    clock_gettime(CLOCK_MONOTONIC, &t_total1);
    printf("total: %.2f ms | culled: %d\n",
           timespec_to_ms(&t_total1)-timespec_to_ms(&t_total0), culled_count);
}