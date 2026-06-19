#include "../include/scene.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../include/threading.h"   

#include "../include/math_utils.h"
#include "../include/geometry.h"
#include "../include/renderer.h"
#include "../include/atomic_utils.h"
#include "../include/timer_utils.h"

int culled_count = 0;
static int rendered_count = 0;
static int shadowed_count = 0;
static double render_start_ms = 0.0;
static double shadow_start_ms = 0.0;

int num_threads = 8;

extern int render_width;
extern int render_height;

/* =========================
   SHADOW THREAD FUNCTION
   ========================= */

static void *shadow_thread_func(void *arg)
{
    ShadowJob *job = (ShadowJob *)arg;

    int total = shadow_h * shadow_w;
    for (int i = 0; i < total; i++)
        job->shadow_local[i] = FAR_DEPTH;

    for (int i = job->start; i < job->end; i++) {
        TubeEntry *te = &job->tubes[i];

        float angle = i * job->angle_step;
        float ti    = i * job->translate_step;

        Mat4 model = matMult(
            mat_translate(
                job->tx + job->mtx * ti,
                job->ty + job->mty * ti,
                job->tz + job->mtz * ti),
            matMult(
                matMult(
                    mat_rotate_x(job->rx * angle + job->rcx),
                    matMult(
                        mat_rotate_y(job->ry * angle + job->rcy),
                        mat_rotate_z(job->rz * angle + job->rcz))),
                mat_scale(job->scale, job->scale, job->scale)));

        Mat4 light_mvp = matMult(job->light_vp, model);

        if (!aabb_in_frustum(te->bounds, light_mvp))
            continue;

        render_tube_shadow_local(
            te->world_curve,
            light_mvp,
            te->tube_props.segments,
            te->tube_props.sides,
            te->tube_props.radius,
            job->shadow_local);

        int n = ATOMIC_ADD_AND_FETCH(&shadowed_count, 1);
        int interval = job->tube_count / 20;

        if (interval > 0 && n % interval == 0) {
            TimerStamp now = get_timestamp();
            double elapsed = timespec_to_ms(&now) - shadow_start_ms;
            double eta = (elapsed / n) * (job->tube_count - n);
            printf("shadowed %d / %d | ETA %.1fs\n",
                   n, job->tube_count, eta / 1000.0);
        }
    }

    return NULL;
}

/* =========================
   RENDER THREAD FUNCTION
   ========================= */

static void *render_thread(void *arg)
{
    RenderJob *job = (RenderJob *)arg;

    for (int i = job->start; i < job->end; i++) {
        TubeEntry *te = &job->tubes[i];
        TubeProperties props = te->tube_props;
        BezierCubic world_curve = te->world_curve;

        Vec3 mid = {
            (world_curve.p0.x + world_curve.p3.x) * 0.5f,
            (world_curve.p0.y + world_curve.p3.y) * 0.5f,
            (world_curve.p0.z + world_curve.p3.z) * 0.5f
        };

        float dx = mid.x - job->eye.x;
        float dy = mid.y - job->eye.y;
        float dz = mid.z - job->eye.z;

        if (dx*dx + dy*dy + dz*dz > job->far_plane * job->far_plane)
            continue;

        if (!aabb_in_frustum(te->bounds, job->vp)) {
            ATOMIC_FETCH_AND_ADD(&culled_count, 1);
            continue;
        }

        render_tube(
            world_curve,
            job->vp,
            props.segments,
            props.sides,
            props.radius,
            te->color_r,
            te->color_g,
            te->color_b,
            job->light,
            &job->tb);

        int n = ATOMIC_ADD_AND_FETCH(&rendered_count, 1);
        int interval = job->tubes_total / 20;

        if (interval > 0 && n % interval == 0) {
            TimerStamp now = get_timestamp();
            double elapsed = timespec_to_ms(&now) - render_start_ms;
            double avg = elapsed / n;
            double eta = avg * (job->tubes_total - n);

            printf("rendered %d / %d | avg %.2fms | ETA %.1fs\n",
                   n, job->tubes_total, avg, eta / 1000.0);
        }
    }

    return NULL;
}

/* =========================
   SHADOW PASS
   ========================= */

static void shadow_pass(SceneConfig *scene, Plane light_planes[6])
{
    TimerStamp now = get_timestamp();
    shadow_start_ms = timespec_to_ms(&now);
    shadowed_count = 0;

    thread_t *threads = malloc(num_threads * sizeof(thread_t));
    ShadowJob *jobs   = malloc(num_threads * sizeof(ShadowJob));

    if (!threads || !jobs) {
        fprintf(stderr, "shadow_pass: malloc failed\n");
        free(threads);
        free(jobs);
        return;
    }

    int per = scene->tube_count / num_threads;

    for (int t = 0; t < num_threads; t++) {
        jobs[t].tubes      = scene->tubes;
        jobs[t].start      = t * per;
        jobs[t].end        = (t == num_threads - 1)
                                ? scene->tube_count
                                : (t + 1) * per;

        jobs[t].light_vp   = scene->light_vp;
        jobs[t].tube_count = scene->tube_count;

        jobs[t].rx = scene->transform.rotate_x;
        jobs[t].ry = scene->transform.rotate_y;
        jobs[t].rz = scene->transform.rotate_z;
        jobs[t].scale = scene->transform.scale;

        jobs[t].tx = scene->transform.translate_x;
        jobs[t].ty = scene->transform.translate_y;
        jobs[t].tz = scene->transform.translate_z;

        jobs[t].angle_step     = scene->transform.angle_step;
        jobs[t].mtx            = scene->transform.multTranslatex;
        jobs[t].mty            = scene->transform.multTranslatey;
        jobs[t].mtz            = scene->transform.multTranslatez;
        jobs[t].translate_step = scene->transform.translate_step;

        memcpy(jobs[t].planes, light_planes, sizeof(Plane) * 6);

        jobs[t].shadow_local =
            malloc((size_t)shadow_h * shadow_w * sizeof(float));

        if (!jobs[t].shadow_local) {
            fprintf(stderr, "shadow malloc failed thread %d\n", t);
            for (int k = 0; k < t; k++)
                free(jobs[k].shadow_local);

            free(threads);
            free(jobs);
            return;
        }

        int total = shadow_h * shadow_w;
        for (int i = 0; i < total; i++)
            jobs[t].shadow_local[i] = FAR_DEPTH;

        threads[t] = thread_create(shadow_thread_func, &jobs[t]);
    }

    for (int t = 0; t < num_threads; t++)
        thread_join(threads[t]);

    /* merge shadow maps */
    for (int y = 0; y < shadow_h; y++) {
        for (int x = 0; x < shadow_w; x++) {
            float min = FAR_DEPTH;
            int idx = y * shadow_w + x;

            for (int t = 0; t < num_threads; t++) {
                if (jobs[t].shadow_local[idx] < min)
                    min = jobs[t].shadow_local[idx];
            }

            shadow_map[y * shadow_w + x] = min;
        }
    }

    for (int t = 0; t < num_threads; t++)
        free(jobs[t].shadow_local);

    free(jobs);
    free(threads);

    now = get_timestamp();
    printf("shadow pass: %.2f ms\n",
           timespec_to_ms(&now) - shadow_start_ms);
}

/* =========================
   MAIN RENDER FUNCTION
   ========================= */

void render_scene(SceneConfig scene)
{
    TimerStamp t_total0 = get_timestamp();

    Mat4 proj = mat_projection(
        scene.camera.fov,
        (float)render_width / render_height,
        scene.camera.near_plane,
        scene.camera.far_plane);

    Mat4 view = mat_look_at(
        scene.camera.eye,
        scene.camera.target,
        scene.camera.up);

    Mat4 vp = matMult(proj, view);

    Plane planes[6], light_planes[6];
    extract_frustum_planes(planes, vp, scene.camera.eye);
    extract_frustum_planes(light_planes, scene.light_vp, scene.light);

    /* world transform */
    TimerStamp t0 = get_timestamp();

    for (int i = 0; i < scene.tube_count; i++) {
        TubeEntry *te = &scene.tubes[i];

        float angle = i * scene.transform.angle_step;
        float ti    = i * scene.transform.translate_step;

        Mat4 model = matMult(
            mat_translate(
                scene.transform.translate_x +
                    scene.transform.multTranslatex * ti,
                scene.transform.translate_y +
                    scene.transform.multTranslatey * ti,
                scene.transform.translate_z +
                    scene.transform.multTranslatez * ti),
            matMult(
                matMult(
                    mat_rotate_x(scene.transform.rotate_x * angle +
                                 scene.transform.rconst_x),
                    matMult(
                        mat_rotate_y(scene.transform.rotate_y * angle +
                                     scene.transform.rconst_y),
                        mat_rotate_z(scene.transform.rotate_z * angle +
                                     scene.transform.rconst_z))),
                mat_scale(scene.transform.scale,
                          scene.transform.scale,
                          scene.transform.scale)));

        Vec4 p0 = mat4_mul_vec4(model,
                    (Vec4){te->curve.p0.x, te->curve.p0.y, te->curve.p0.z, 1.0f});
        Vec4 p1 = mat4_mul_vec4(model,
                    (Vec4){te->curve.p1.x, te->curve.p1.y, te->curve.p1.z, 1.0f});
        Vec4 p2 = mat4_mul_vec4(model,
                    (Vec4){te->curve.p2.x, te->curve.p2.y, te->curve.p2.z, 1.0f});
        Vec4 p3 = mat4_mul_vec4(model,
                    (Vec4){te->curve.p3.x, te->curve.p3.y, te->curve.p3.z, 1.0f});

        te->world_curve = (BezierCubic){
            {p0.x, p0.y, p0.z},
            {p1.x, p1.y, p1.z},
            {p2.x, p2.y, p2.z},
            {p3.x, p3.y, p3.z}
        };

        te->bounds = bezier_bounds_adaptive(
            te->world_curve,
            te->tube_props.radius,
            te->segment_bounds,
            &te->segment_count);
    }

    TimerStamp t1 = get_timestamp();
    printf("world transform: %.2f ms\n",
           timespec_to_ms(&t1) - timespec_to_ms(&t0));

    shadow_pass(&scene, light_planes);

    RenderJob *jobs   = malloc(num_threads * sizeof(RenderJob));
    thread_t   *threads = malloc(num_threads * sizeof(thread_t));

    if (!jobs || !threads) {
        fprintf(stderr, "render_scene: malloc failed\n");
        free(jobs);
        free(threads);
        return;
    }

    int per = scene.tube_count / num_threads;

    for (int t = 0; t < num_threads; t++) {
        jobs[t].tubes       = scene.tubes;
        jobs[t].start       = t * per;
        jobs[t].end         = (t == num_threads - 1)
                                ? scene.tube_count
                                : (t + 1) * per;

        jobs[t].vp          = vp;
        jobs[t].light       = scene.light;
        jobs[t].tubes_total = scene.tube_count;
        jobs[t].eye        = scene.camera.eye;
        jobs[t].far_plane  = scene.camera.far_plane;

        memcpy(jobs[t].planes, planes, sizeof(planes));
    }

    rendered_count = 0;
    culled_count   = 0;

    TimerStamp now = get_timestamp();
    render_start_ms = timespec_to_ms(&now);

    for (int t = 0; t < num_threads; t++)
        threads[t] = thread_create(render_thread, &jobs[t]);

    for (int t = 0; t < num_threads; t++)
        thread_join(threads[t]);

    free(jobs);
    free(threads);

    t1 = get_timestamp();
    printf("main render: %.2f ms\n",
           timespec_to_ms(&t1) - timespec_to_ms(&t0));

    TimerStamp t_total1 = get_timestamp();
    printf("total: %.2f ms | culled: %d\n",
           timespec_to_ms(&t_total1) - timespec_to_ms(&t_total0),
           culled_count);
}