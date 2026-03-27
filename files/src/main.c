#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "types.h"
#include "math.h"
#include "geometry.h"
#include "renderer.h"
#include "scene.h"

static Vec3 parse_vec3(const char *str){
    Vec3 v = {0};
    sscanf(str, "%f,%f,%f", &v.x, &v.y, &v.z);
    return v;
}

static void print_usage(const char *prog){
    printf("Usage: %s [options]\n", prog);
    printf("  -t    <int>    number of tubes (default 100000)\n");
    printf("  -seg  <int>    segments per tube (default 20)\n");
    printf("  -sid  <int>    sides per tube (default 12)\n");
    printf("  -r    <float>  tube radius (default 0.05)\n");
    printf("  -scale <float> scene scale (default 1.5)\n");
    printf("  -p0   <int>    point on curve {x,y,z} (default -1.5, 15.0, 23.0)\n");
    printf("  -p1   <int>    point on curve {x,y,z} (default -5.4,  2.5,  8.0)\n");
    printf("  -p2   <int>    point on curve {x,y,z} (default -60.5, -1.5, 16.0)\n");
    printf("  -p3   <int>    point on curve {x,y,z} (default 23.5, -24.0, 24.0\n");
    printf("  -rx   <float>  rotate x multiplier (default 0.0)\n");
    printf("  -ry   <float>  rotate y multiplier (default 1.0)\n");
    printf("  -rz   <float>  rotate z multiplier (default 1.0)\n");
    printf("  -rcx  <float>  rotate x constant (default 0.0)\n");
    printf("  -rcy  <float>  rotate y constant (default 1.0)\n");
    printf("  -rcz  <float>  rotate z constant (default 1.0)\n");
    printf("  -as   <float>  angle step(for continued rotation) (default 0.001)\n");
    printf("  -tx   <float>  translate x (default 0.0)\n");
    printf("  -ty   <float>  translate y (default 0.0)\n");
    printf("  -tz   <float>  translate z (default 0.0)\n");
    printf("  -cx   <float>  camera x (default 0.0)\n");
    printf("  -cy   <float>  camera y (default 0.0)\n");
    printf("  -cz   <float>  camera z (default -200.0)\n");
    printf("  -lx   <float>  light x (default 0.0)\n");
    printf("  -ly   <float>  light y (default 5.0)\n");
    printf("  -lz   <float>  light z (default -40.0)\n");
    printf("  -fov  <float>  field of view degrees (default 90)\n");
    printf("  -ow   <int>    pixel width of output (default 1920)\n");
    printf("  -oh   <int>    pixel height of output (default 1080)\n");
    printf("  -rgb  <int>    enable rainbow coloring\n");
    printf("  -color r,g,b   set solid color (0-255)\n");
    printf("  -o    <string> output file (default output.ppm)\n");
}

int main(int argc, char **argv){
    // defaults
    int   num_tubes    = 100000;
    int   segments     = 20;
    int   sides        = 12;
    float radius       = 0.05f;
    float scale        = 1.5f;
    float rx           = 0.0f;
    float ry           = 1.0f;
    float rz           = 1.0f;
    float rcx          = 0.0f;
    float rcy          = 0.0f;
    float rcz          = 0.0f;
    float angle_step   = 0.001f;
    float tx           = 0.0f;
    float ty           = 0.0f;
    float tz           = 0.0f;
    float cx           = 0.0f;
    float cy           = 0.0f;
    float cz           = -200.0f;
    float lx           = 0.0f;
    float ly           = 5.0f;
    float lz           = -40.0f;
    float fov_deg      = 90.0f;
    char *p0_str = NULL;
    char *p1_str = NULL;
    char *p2_str = NULL;
    char *p3_str = NULL;
    char  output[256]  = "output.ppm";
    int   out_w = OUT_WIDTH;
    int   out_h = OUT_HEIGHT;
    int use_rgb = 0;
    int use_custom_color = 0;
    int custom_r = 255, custom_g = 255, custom_b = 255;
    int custom_focus = 0; 
    float focusx = 0.0f, focusy = 0.0f, focusz = 0.0f;

    // parse args
    for(int i = 1; i < argc; i++){
        if(!strcmp(argv[i], "-h") || !strcmp(argv[i], "--help")){
            print_usage(argv[0]); return 0;
        }
        else if(!strcmp(argv[i], "-t")     && i+1<argc) num_tubes  = atoi(argv[++i]);
        else if(!strcmp(argv[i], "-seg")   && i+1<argc) segments   = atoi(argv[++i]);
        else if(!strcmp(argv[i], "-sid")   && i+1<argc) sides      = atoi(argv[++i]);
        else if(!strcmp(argv[i], "-r")     && i+1<argc) radius     = atof(argv[++i]);
        else if(!strcmp(argv[i], "-scale") && i+1<argc) scale      = atof(argv[++i]);
        else if(!strcmp(argv[i], "-p0") && i+1<argc) p0_str        = argv[++i];
        else if(!strcmp(argv[i], "-p1") && i+1<argc) p1_str        = argv[++i];
        else if(!strcmp(argv[i], "-p2") && i+1<argc) p2_str        = argv[++i];
        else if(!strcmp(argv[i], "-p3") && i+1<argc) p3_str        = argv[++i];
        else if(!strcmp(argv[i], "-rx")    && i+1<argc) rx         = atof(argv[++i]);
        else if(!strcmp(argv[i], "-ry")    && i+1<argc) ry         = atof(argv[++i]);
        else if(!strcmp(argv[i], "-rz")    && i+1<argc) rz         = atof(argv[++i]);
        else if(!strcmp(argv[i], "-rcx")    && i+1<argc) rcx       = atof(argv[++i]);
        else if(!strcmp(argv[i], "-rcy")    && i+1<argc) rcy       = atof(argv[++i]);
        else if(!strcmp(argv[i], "-rcz")    && i+1<argc) rcz       = atof(argv[++i]);
        else if(!strcmp(argv[i], "-as")    && i+1<argc) angle_step = atof(argv[++i]);
        else if(!strcmp(argv[i], "-tx")    && i+1<argc) tx         = atof(argv[++i]);
        else if(!strcmp(argv[i], "-ty")    && i+1<argc) ty         = atof(argv[++i]);
        else if(!strcmp(argv[i], "-tz")    && i+1<argc) tz         = atof(argv[++i]);
        else if(!strcmp(argv[i], "-cx")    && i+1<argc) cx         = atof(argv[++i]);
        else if(!strcmp(argv[i], "-cy")    && i+1<argc) cy         = atof(argv[++i]);
        else if(!strcmp(argv[i], "-cz")    && i+1<argc) cz         = atof(argv[++i]);
        else if(!strcmp(argv[i], "-lx")    && i+1<argc) lx         = atof(argv[++i]);
        else if(!strcmp(argv[i], "-ly")    && i+1<argc) ly         = atof(argv[++i]);
        else if(!strcmp(argv[i], "-lz")    && i+1<argc) lz         = atof(argv[++i]);
        else if(!strcmp(argv[i], "-fov")   && i+1<argc) fov_deg    = atof(argv[++i]);
        else if(!strcmp(argv[i], "-focus") && i+1<argc){sscanf(argv[++i], "%f,%f,%f", &focusx, &focusy, &focusz);custom_focus = 1;}
        else if(!strcmp(argv[i], "-o")     && i+1<argc) strncpy(output, argv[++i], 255);
        else if(!strcmp(argv[i], "-ow") && i+1<argc) out_w         = atoi(argv[++i]);
        else if(!strcmp(argv[i], "-oh") && i+1<argc) out_h         = atoi(argv[++i]);
        else if(!strcmp(argv[i], "-rgb")){use_rgb                  = 1;}
        else if(!strcmp(argv[i], "-color") && i+1<argc){sscanf(argv[++i], "%d,%d,%d", &custom_r, &custom_g, &custom_b);use_custom_color = 1;}
        else { printf("unknown arg: %s\n", argv[i]); print_usage(argv[0]); return 1; }
    }
    // clamp to max limits
    if(out_w <= 0) out_w = OUT_WIDTH;
    if(out_h <= 0) out_h = OUT_HEIGHT;
    if(custom_r < 0) custom_r = 0; if(custom_r > 255) custom_r = 255;
    if(custom_g < 0) custom_g = 0; if(custom_g > 255) custom_g = 255;
    if(custom_b < 0) custom_b = 0; if(custom_b > 255) custom_b = 255;
    if(segments > MAX_TUBE_SEGMENTS) segments = MAX_TUBE_SEGMENTS;
    if(sides    > MAX_TUBE_SIDES)    sides    = MAX_TUBE_SIDES;

    printf("tubes=%d seg=%d sid=%d scale=%.2f ry=%.2f rz=%.2f step=%.4f\n",
           num_tubes, segments, sides, scale, ry, rz, angle_step);

    renderer_init();

    SceneConfig scene = {0};

    scene.camera.eye        = (Vec3){cx, cy, cz};
    if(custom_focus == 1){scene.camera.target = (Vec3){focusx, focusy, focusz};}
    else{scene.camera.target = (Vec3){0.0f, 0.0f, 0.0f};}
    scene.camera.up         = (Vec3){0.0f, 1.0f, 0.0f};
    scene.camera.fov        = fov_deg * 3.14159f / 180.0f;
    scene.camera.near_plane = 0.2f;
    scene.camera.far_plane  = 500.0f;

    scene.light = (Vec3){lx, ly, lz};

    scene.transform.rotate_x   = rx;
    scene.transform.rotate_y   = ry;
    scene.transform.rotate_z   = rz;
    scene.transform.scale      = scale;
    scene.transform.translate_x = tx;
    scene.transform.translate_y = ty;
    scene.transform.translate_z = tz;
    scene.transform.angle_step = angle_step;
    scene.transform.rconst_x = rcx;
    scene.transform.rconst_y = rcy;
    scene.transform.rconst_z = rcz;



    TubeEntry *tubes = (TubeEntry *)malloc(num_tubes * sizeof(TubeEntry));

    for(int i = 0; i < num_tubes; i++){
        BezierCubic curve;

        // default
        curve = (BezierCubic){
            {-1.5f, 15.0f, 23.0f},
            {-5.4f,  2.5f,  8.0f},
            {-60.5f, -1.5f, 16.0f},
            {23.5f, -24.0f, 24.0f}
        };

        // override if all provided
        if((p0_str || p1_str || p2_str || p3_str) &&
        !(p0_str && p1_str && p2_str && p3_str)){
            printf("error: must provide all of -p0 -p1 -p2 -p3\n");
            return 1;
        }
        if(p0_str && p1_str && p2_str && p3_str){
            curve.p0 = parse_vec3(p0_str);
            curve.p1 = parse_vec3(p1_str);
            curve.p2 = parse_vec3(p2_str);
            curve.p3 = parse_vec3(p3_str);
        }

        tubes[i].curve = curve;
        tubes[i].tube_props.segments = segments;
        tubes[i].tube_props.sides    = sides;
        tubes[i].tube_props.radius   = radius;
        //color
        if(use_rgb){
            float phase = (float)i / (float)num_tubes * 2.0f * 3.14159f;
            tubes[i].color_r = (int)(127.5f * (1.0f + sinf(phase)));
            tubes[i].color_g = (int)(127.5f * (1.0f + sinf(phase + 2.094f)));
            tubes[i].color_b = (int)(127.5f * (1.0f + sinf(phase + 4.188f)));
        }
        else if(use_custom_color){
            tubes[i].color_r = custom_r;
            tubes[i].color_g = custom_g;
            tubes[i].color_b = custom_b;
        }
        else{
            tubes[i].color_r = 255;
            tubes[i].color_g = 255;
            tubes[i].color_b = 255;
        }
    }

    scene.tubes      = tubes;
    scene.tube_count = num_tubes;

    Mat4 light_proj  = mat_projection(3.14159f / 2.0f, (float)WIDTH/HEIGHT, 0.1f, 500.0f);
    Mat4 light_view  = mat_look_at(scene.light, (Vec3){0,0,0}, (Vec3){0,1,0});
    scene.light_vp   = matMult(light_proj, light_view);

    extern Mat4 light_vp_global;
    light_vp_global  = scene.light_vp;

    shadow_map_init();
    render_scene(scene);

    // output
    Pixel (*fb)[WIDTH] = get_framebuffer();
    FILE *f = fopen(output, "wb");
    if(!f){ fprintf(stderr, "failed to open %s\n", output); return 1; }
    fprintf(f, "P6\n%d %d\n255\n", out_w, out_h);

    for(int y = 0; y < out_h; y++){
        for(int x = 0; x < out_w; x++){
            float u  = (x + 0.5f) / out_w;
            float v  = (y + 0.5f) / out_h;
            float du = 0.5f / out_w;
            float dv = 0.5f / out_h;

            int sx0 = (int)((u-du)*WIDTH);  if(sx0 < 0) sx0 = 0;
            int sx1 = (int)((u+du)*WIDTH);  if(sx1 >= WIDTH)  sx1 = WIDTH-1;
            int sy0 = (int)((v-dv)*HEIGHT); if(sy0 < 0) sy0 = 0;
            int sy1 = (int)((v+dv)*HEIGHT); if(sy1 >= HEIGHT) sy1 = HEIGHT-1;
            if(sx1 <= sx0) sx1 = sx0+1;
            if(sy1 <= sy0) sy1 = sy0+1;

            int r=0, g=0, b=0, count=0;
            for(int sy=sy0; sy<=sy1; sy++){
                for(int sx=sx0; sx<=sx1; sx++){
                    r += fb[sy][sx].r;
                    g += fb[sy][sx].g;
                    b += fb[sy][sx].b;
                    count++;
                }
            }
            unsigned char px[3] = {r/count, g/count, b/count};
            fwrite(px, 1, 3, f);
        }
    }
    fclose(f);
    free(tubes);
    return 0;
}