#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#ifndef _WIN32
#include <unistd.h> 
#endif
#include "../include/types.h"
#include "../include/math_utils.h"
#include "../include/geometry.h"
#include "../include/renderer.h"
#include "../include/scene.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../include/stb_image_write.h"
int shadow_w_arg = 1024;
int shadow_h_arg = 1024;


#include <stdio.h>
#ifdef _WIN32
#include <windows.h>
int get_cpu_count(void) {
    SYSTEM_INFO si; GetSystemInfo(&si); return (int)si.dwNumberOfProcessors;
}
#else
#include <unistd.h>
int get_cpu_count(void) {
    long n = sysconf(_SC_NPROCESSORS_ONLN);
    if (n < 1) n = sysconf(_SC_NPROCESSORS_CONF);
    return (n < 1) ? 1 : (int)n;
}
#endif


extern float *zbuffer;   // so we can read depth for fog

// Background
static int   use_bg = 0;
static int   bg_r = 0, bg_g = 0, bg_b = 0;

static int   use_dof = 0;
static float focal_depth = 0.5f; 
static float aperture = 8.0f;         
static float max_blur_radius = 6.0f;  

//bloom
static int   use_bloom = 0;
static float bloom_threshold = 0.7f;   // brightness level to start glowing (0‑1)
static float bloom_intensity = 0.4f;   // how strong the glow is

// Fog
static int   use_fog = 0;
static float fog_density = 0.15f;        
static int   fog_r = 180, fog_g = 200, fog_b = 255;   // light atmospheric blue

// Vignette
static int   use_vignette = 0;
static float vignette_strength = 0.4f;
static Vec3 parse_vec3(const char *str){
    Vec3 v = {0};
    sscanf(str, "%f,%f,%f", &v.x, &v.y, &v.z);
    return v;
}

static Pixel sample_bilinear(const Pixel *buf, int bw, int bh, float u, float v) {
    float fx = u * bw;
    float fy = v * bh;
    int ix = (int)fx, iy = (int)fy;
    float sx = fx - ix, sy = fy - iy;
    int x0 = ix, x1 = ix + 1; if (x1 >= bw) x1 = bw - 1;
    int y0 = iy, y1 = iy + 1; if (y1 >= bh) y1 = bh - 1;
    Pixel p00 = buf[y0 * bw + x0];
    Pixel p10 = buf[y0 * bw + x1];
    Pixel p01 = buf[y1 * bw + x0];
    Pixel p11 = buf[y1 * bw + x1];
    Pixel res;
    res.r = (unsigned char)((1-sx)*(1-sy)*p00.r + sx*(1-sy)*p10.r + (1-sx)*sy*p01.r + sx*sy*p11.r + 0.5f);
    res.g = (unsigned char)((1-sx)*(1-sy)*p00.g + sx*(1-sy)*p10.g + (1-sx)*sy*p01.g + sx*sy*p11.g + 0.5f);
    res.b = (unsigned char)((1-sx)*(1-sy)*p00.b + sx*(1-sy)*p10.b + (1-sx)*sy*p01.b + sx*sy*p11.b + 0.5f);
    return res;
}




static void gauss_blur(Pixel *dst, const Pixel *src, int w, int h, int radius, float sigma) {
    // 1D kernel
    int ksize = 2 * radius + 1;
    float *kernel = malloc(ksize * sizeof(float));
    float sum = 0;
    for (int i = -radius; i <= radius; i++) {
        float val = expf(-(i * i) / (2.0f * sigma * sigma));
        kernel[i + radius] = val;
        sum += val;
    }
    for (int i = 0; i < ksize; i++) kernel[i] /= sum;

    // Temporary buffer for horizontal pass
    Pixel *temp = malloc((size_t)w * h * sizeof(Pixel));

    // Horizontal pass
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            float r = 0, g = 0, b = 0;
            for (int k = -radius; k <= radius; k++) {
                int sx = x + k;
                if (sx < 0) sx = 0; else if (sx >= w) sx = w - 1;
                Pixel sp = src[y * w + sx];
                float weight = kernel[k + radius];
                r += sp.r * weight; g += sp.g * weight; b += sp.b * weight;
            }
            temp[y * w + x].r = (unsigned char)(r + 0.5f);
            temp[y * w + x].g = (unsigned char)(g + 0.5f);
            temp[y * w + x].b = (unsigned char)(b + 0.5f);
        }
    }

    // Vertical pass
    for (int y = 0; y < h; y++) {
        for (int x = 0; x < w; x++) {
            float r = 0, g = 0, b = 0;
            for (int k = -radius; k <= radius; k++) {
                int sy = y + k;
                if (sy < 0) sy = 0; else if (sy >= h) sy = h - 1;
                Pixel sp = temp[sy * w + x];
                float weight = kernel[k + radius];
                r += sp.r * weight; g += sp.g * weight; b += sp.b * weight;
            }
            dst[y * w + x].r = (unsigned char)(r + 0.5f);
            dst[y * w + x].g = (unsigned char)(g + 0.5f);
            dst[y * w + x].b = (unsigned char)(b + 0.5f);
        }
    }

    free(kernel);
    free(temp);
}

static void print_usage(const char *prog){
    printf("Usage: %s [options]\n", prog);
    printf("  -t    <int>    number of tubes (default 100000)\n");
    printf("  -seg  <int>    segments per tube (default 20)\n");
    printf("  -sid  <int>    sides per tube (default 12)\n");
    printf("  -r    <float>  tube radius (default 0.05)\n");
    printf("  -scale <float> scene scale (default 1.5)\n");
    printf("  -p0   <str>    point on curve {x,y,z} (default -1.5,15.0,23.0)\n");
    printf("  -p1   <str>    point on curve {x,y,z} (default -5.4,2.5,8.0)\n");
    printf("  -p2   <str>    point on curve {x,y,z} (default -60.5,-1.5,16.0)\n");
    printf("  -p3   <str>    point on curve {x,y,z} (default 23.5,-24.0,24.0)\n");
    printf("  -rx   <float>  rotate x multiplier (default 0.0)\n");
    printf("  -ry   <float>  rotate y multiplier (default 1.0)\n");
    printf("  -rz   <float>  rotate z multiplier (default 1.0)\n");
    printf("  -rcx  <float>  rotate x constant (default 0.0)\n");
    printf("  -rcy  <float>  rotate y constant (default 1.0)\n");
    printf("  -rcz  <float>  rotate z constant (default 1.0)\n");
    printf("  -as   <float>  angle step (for continued rotation) (default 0.001)\n");
    printf("  -tx   <float>  translate x (default 0.0)\n");
    printf("  -ty   <float>  translate y (default 0.0)\n");
    printf("  -tz   <float>  translate z (default 0.0)\n");
    printf("  -mtx  <float>  translate x multiplier per tube (default 0.0)\n");
    printf("  -mty  <float>  translate y multiplier per tube (default 0.0)\n");
    printf("  -mtz  <float>  translate z multiplier per tube (default 0.0)\n");
    printf("  -ts   <float>  translate step per tube index (default 0.0)\n");
    printf("  -cx   <float>  camera x (default 0.0)\n");
    printf("  -cy   <float>  camera y (default 0.0)\n");
    printf("  -cz   <float>  camera z (default -200.0)\n");
    printf("  -lx   <float>  light x (default 0.0)\n");
    printf("  -ly   <float>  light y (default 5.0)\n");
    printf("  -lz   <float>  light z (default -40.0)\n");
    printf("  -fov  <float>  field of view degrees (default 90)\n");
    printf("  -focus <x,y,z> camera look-at target (default 0,0,0)\n");
    printf("  -ow   <int>    pixel width of output (default 1920)\n");
    printf("  -oh   <int>    pixel height of output (default 1080)\n");
    printf("  -iw   <int>    internal render width (default 3840)\n");
    printf("  -ih   <int>    internal render height (default 2160)\n");
    printf("  -sw   <int>    shadow map width (default 1024)\n");
    printf("  -sh   <int>    shadow map height (default 1024)\n");
    printf("  -threads <int>   number of rendering threads (auto)\n");
    printf("  -rgb  <flag>   enable rainbow coloring\n");
    printf("  -cycles <float> rainbow cycles across all tubes (use with -rgb)(default 1.0)\n");
    printf("  -color r,g,b   set solid color (0-255)\n");
    printf("  -png  <flag>   output as PNG instead of PPM\n");
    printf("  -aces <flag>   apply ACES filmic tone mapping\n");
    printf("  -bg    <r,g,b> background color for untouched pixels\n");
    printf("  -fog           enable exponential depth fog\n");
    printf("  -fogdensity <float> fog density (default 0.15, range 0.01–1.0)\n");
    printf("  -fogcolor <r,g,b>   fog colour (default 180,200,255)\n");
    printf("  -vignette [str]  enable vignette with optional strength (default 0.4)\n");
    printf("  -dof           enable depth of field\n");
    printf("  -focal <float> focal depth in NDC (default 0.5)\n");
    printf("  -aperture <float> blur amount (default 8.0)\n");
    printf("  -bloom          enable bloom (glow)\n");
    printf("  -bloomthreshold <float> brightness threshold (default 0.7)\n");
    printf("  -bloomintensity <float> glow strength (default 0.4)\n");
    printf("  -o    <string> output file (default output.ppm)\n");

}

int main(int argc, char **argv){
    /* defaults */
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
    float mtx = 0.0f, mty = 0.0f, mtz = 0.0f;
    float translate_step = 0.0f;
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
    int   internal_w = 3840;
    int   internal_h = 2160;
    int   use_rgb = 0;
    int   use_custom_color = 0;
    int   custom_r = 255, custom_g = 255, custom_b = 255;
    int   custom_focus = 0;
    float focusx = 0.0f, focusy = 0.0f, focusz = 0.0f;
    int   use_png = 0;
    int use_aces = 0;
    float rainbow_cycles = 1.0f;
    int num_threads_arg = 0; //auto

    /* parse args */
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
        else if(!strcmp(argv[i], "-rcx")   && i+1<argc) rcx        = atof(argv[++i]);
        else if(!strcmp(argv[i], "-rcy")   && i+1<argc) rcy        = atof(argv[++i]);
        else if(!strcmp(argv[i], "-rcz")   && i+1<argc) rcz        = atof(argv[++i]);
        else if(!strcmp(argv[i], "-as")    && i+1<argc) angle_step = atof(argv[++i]);
        else if(!strcmp(argv[i], "-tx")    && i+1<argc) tx         = atof(argv[++i]);
        else if(!strcmp(argv[i], "-ty")    && i+1<argc) ty         = atof(argv[++i]);
        else if(!strcmp(argv[i], "-tz")    && i+1<argc) tz         = atof(argv[++i]);
        else if(!strcmp(argv[i], "-mtx") && i+1<argc) mtx          = atof(argv[++i]);
        else if(!strcmp(argv[i], "-mty") && i+1<argc) mty          = atof(argv[++i]);
        else if(!strcmp(argv[i], "-mtz") && i+1<argc) mtz          = atof(argv[++i]);
        else if(!strcmp(argv[i], "-ts")  && i+1<argc) translate_step = atof(argv[++i]);
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
        else if(!strcmp(argv[i], "-iw") && i+1<argc) internal_w    = atoi(argv[++i]);
        else if(!strcmp(argv[i], "-ih") && i+1<argc) internal_h    = atoi(argv[++i]);
        else if(!strcmp(argv[i], "-sw") && i+1<argc) shadow_w_arg = atoi(argv[++i]);
        else if(!strcmp(argv[i], "-sh") && i+1<argc) shadow_h_arg = atoi(argv[++i]);
        else if(!strcmp(argv[i], "-rgb")){use_rgb                  = 1;}
        else if(!strcmp(argv[i], "-color") && i+1<argc){sscanf(argv[++i], "%d,%d,%d", &custom_r, &custom_g, &custom_b);use_custom_color = 1;}
        else if(!strcmp(argv[i], "-png")){use_png                  = 1;}
        else if(!strcmp(argv[i], "-aces")) { use_aces = 1; }
        else if(!strcmp(argv[i], "-cycles") && i+1<argc) rainbow_cycles = atof(argv[++i]);
        else if(!strcmp(argv[i], "-bg") && i+1<argc){ sscanf(argv[++i], "%d,%d,%d", &bg_r, &bg_g, &bg_b); use_bg = 1;}
        else if(!strcmp(argv[i], "-fog")) { use_fog = 1; }
        else if(!strcmp(argv[i], "-fogdensity") && i+1<argc) fog_density = atof(argv[++i]);
        else if(!strcmp(argv[i], "-fogcolor") && i+1<argc) {sscanf(argv[++i], "%d,%d,%d", &fog_r, &fog_g, &fog_b);use_fog = 1;}
        else if(!strcmp(argv[i], "-vignette")){ use_vignette = 1;if(i+1<argc && argv[i+1][0]!='-') vignette_strength = atof(argv[++i]);}
        else if(!strcmp(argv[i], "-dof")) { use_dof = 1; }
        else if(!strcmp(argv[i], "-focal") && i+1<argc) focal_depth = atof(argv[++i]);
        else if(!strcmp(argv[i], "-aperture") && i+1<argc) aperture = atof(argv[++i]);
        else if(!strcmp(argv[i], "-bloom")) { use_bloom = 1; }
        else if(!strcmp(argv[i], "-bloomthreshold") && i+1<argc) bloom_threshold = atof(argv[++i]);
        else if(!strcmp(argv[i], "-bloomintensity") && i+1<argc) bloom_intensity = atof(argv[++i]);
        else if(!strcmp(argv[i], "-threads") && i+1<argc) num_threads_arg = atoi(argv[++i]);
        else { printf("unknown arg: %s\n", argv[i]); print_usage(argv[0]); return 1; }
    }

    /* clamp to sane limits */
    if(out_w <= 0) out_w = OUT_WIDTH;
    if(out_h <= 0) out_h = OUT_HEIGHT;
    if(internal_w < 64) internal_w = 64;
    if(internal_h < 64) internal_h = 64;
    if(custom_r < 0) custom_r = 0; if(custom_r > 255) custom_r = 255;
    if(custom_g < 0) custom_g = 0; if(custom_g > 255) custom_g = 255;
    if(custom_b < 0) custom_b = 0; if(custom_b > 255) custom_b = 255;
    if(segments > MAX_TUBE_SEGMENTS) segments = MAX_TUBE_SEGMENTS;
    if(sides    > MAX_TUBE_SIDES)    sides    = MAX_TUBE_SIDES;

    printf("tubes=%d seg=%d sid=%d scale=%.2f ry=%.2f rz=%.2f step=%.4f\n",
           num_tubes, segments, sides, scale, ry, rz, angle_step);

    if (internal_w * out_h != internal_h * out_w) {
        printf("Warning: mismatched aspect ratios\n");
        return 1;
    }


    if(use_png) {
        // If user hasn't overridden the default name, use .png
        if(strcmp(output, "output.ppm") == 0) {
            strcpy(output, "output.png");
        } else {
            // If user gave a name but it has no extension, append .png
            size_t len = strlen(output);
            if(len > 4 && strcmp(output + len - 4, ".ppm") != 0 &&
                        strcmp(output + len - 4, ".png") != 0) {
                strcat(output, ".png");
            }
        }
    }

    /* initialise renderer with user-chosen internal resolution */
    renderer_init(internal_w, internal_h);

    SceneConfig scene = {0};

    scene.camera.eye        = (Vec3){cx, cy, cz};
    if(custom_focus == 1){scene.camera.target = (Vec3){focusx, focusy, focusz};}
    else{scene.camera.target = (Vec3){0.0f, 0.0f, 0.0f};}
    scene.camera.up         = (Vec3){0.0f, 1.0f, 0.0f};
    scene.camera.fov        = fov_deg * 3.14159f / 180.0f;
    scene.camera.near_plane = 0.2f;
    scene.camera.far_plane  = 2000.0f;

    scene.light = (Vec3){lx, ly, lz};

    scene.transform.rotate_x   = rx;
    scene.transform.rotate_y   = ry;
    scene.transform.rotate_z   = rz;
    scene.transform.rconst_x   = rcx;
    scene.transform.rconst_y   = rcy;
    scene.transform.rconst_z   = rcz;
    scene.transform.scale      = scale;
    scene.transform.translate_x = tx;
    scene.transform.translate_y = ty;
    scene.transform.translate_z = tz;
    scene.transform.angle_step = angle_step;
    scene.transform.multTranslatex = mtx;
    scene.transform.multTranslatey = mty;
    scene.transform.multTranslatez = mtz;
    scene.transform.translate_step = translate_step;

    TubeEntry *tubes = (TubeEntry *)malloc((size_t)num_tubes * sizeof(TubeEntry));

    for(int i = 0; i < num_tubes; i++){
        BezierCubic curve;

        curve = (BezierCubic){
            {-1.5f, 15.0f, 23.0f},
            {-5.4f,  2.5f,  8.0f},
            {-60.5f, -1.5f, 16.0f},
            {23.5f, -24.0f, 24.0f}
        };

        if((p0_str || p1_str || p2_str || p3_str) &&
           !(p0_str && p1_str && p2_str && p3_str)){
            printf("error: must provide all of -p0 -p1 -p2 -p3\n");
            free(tubes);
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

        if(use_rgb) {
            float hue = fmodf((float)i / (float)num_tubes * rainbow_cycles * 360.0f, 360.0f);
            float s = 1.0f, v = 1.0f;      // full saturation & brightness
            float c = v * s;
            float x = c * (1.0f - fabsf(fmodf(hue / 60.0f, 2.0f) - 1.0f));
            float m = v - c;
            float rp, gp, bp;
            if(hue < 60) { rp = c; gp = x; bp = 0; }
            else if(hue < 120) { rp = x; gp = c; bp = 0; }
            else if(hue < 180) { rp = 0; gp = c; bp = x; }
            else if(hue < 240) { rp = 0; gp = x; bp = c; }
            else if(hue < 300) { rp = x; gp = 0; bp = c; }
            else { rp = c; gp = 0; bp = x; }
            tubes[i].color_r = (int)((rp + m) * 255.0f);
            tubes[i].color_g = (int)((gp + m) * 255.0f);
            tubes[i].color_b = (int)((bp + m) * 255.0f);
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

    /* Light view-projection matrix */
    Mat4 light_proj  = mat_projection(3.14159f / 2.0f, (float)internal_w / internal_h, 0.1f, 500.0f);
    Mat4 light_view  = mat_look_at(scene.light, (Vec3){0,0,0}, (Vec3){0,1,0});
    scene.light_vp   = matMult(light_proj, light_view);

    extern Mat4 light_vp_global;
    light_vp_global  = scene.light_vp;

    shadow_init(shadow_w_arg, shadow_h_arg);
    extern int num_threads;

    /* auto detect thread count if not provided */
    if (num_threads_arg <= 0) {
        int cores = get_cpu_count();
        num_threads_arg = (cores > 1) ? cores - 1 : 1;
        if (num_threads_arg > 64) num_threads_arg = 64;
    }
    num_threads = num_threads_arg;

    render_scene(scene);

    /* output downsampling using get_framebuffer() */
    Pixel *fb = get_framebuffer();
    if(use_bg) {
        for(int y = 0; y < render_height; y++) {
            Pixel *row = fb + y * render_width;
            for(int x = 0; x < render_width; x++) {
                if(zbuffer[y * render_width + x] == FAR_DEPTH) {   // untouched pixel
                    row[x].r = bg_r;
                    row[x].g = bg_g;
                    row[x].b = bg_b;
                }
            }
        }
    }
    //fog
    if(use_fog) {
        float density = fog_density;
        for(int y = 0; y < render_height; y++) {
            Pixel *row = fb + y * render_width;
            float *zrow = zbuffer + y * render_width;
            for(int x = 0; x < render_width; x++) {
                float depth = zrow[x];
                if(depth >= FAR_DEPTH) continue;   // sky pixels – fog doesn't affect them

                // Exponential fog factor: 0 = clear, 1 = fully fogged
                float fog_factor = 1.0f - expf(-depth * density);
                // Optional: if you want a minimum visibility
                // fog_factor = fminf(fog_factor, 0.9f);

                row[x].r = (unsigned char)(row[x].r * (1.0f - fog_factor) + fog_r * fog_factor);
                row[x].g = (unsigned char)(row[x].g * (1.0f - fog_factor) + fog_g * fog_factor);
                row[x].b = (unsigned char)(row[x].b * (1.0f - fog_factor) + fog_b * fog_factor);
            }
        }
    }

    // ACES tone mapping
    if(use_aces) {
        for(int y = 0; y < render_height; y++) {
            Pixel *row = fb + y * render_width;
            for(int x = 0; x < render_width; x++) {
                float r = row[x].r / 255.0f;
                float g = row[x].g / 255.0f;
                float b = row[x].b / 255.0f;

                // ACES approximation (Narkowicz 2015)
                r = (r * (2.51f * r + 0.03f)) / (r * (2.43f * r + 0.59f) + 0.14f);
                g = (g * (2.51f * g + 0.03f)) / (g * (2.43f * g + 0.59f) + 0.14f);
                b = (b * (2.51f * b + 0.03f)) / (b * (2.43f * b + 0.59f) + 0.14f);

                // Clamp to [0,1]
                if(r > 1.0f) r = 1.0f; if(r < 0.0f) r = 0.0f;
                if(g > 1.0f) g = 1.0f; if(g < 0.0f) g = 0.0f;
                if(b > 1.0f) b = 1.0f; if(b < 0.0f) b = 0.0f;

                row[x].r = (unsigned char)(r * 255.0f);
                row[x].g = (unsigned char)(g * 255.0f);
                row[x].b = (unsigned char)(b * 255.0f);
            }
        }
    }


    //vingette
    if(use_vignette) {
        float cx = render_width * 0.5f;
        float cy = render_height * 0.5f;
        for(int y = 0; y < render_height; y++) {
            Pixel *row = fb + y * render_width;
            float dy = (y - cy) / cy;   // -1 … 1
            for(int x = 0; x < render_width; x++) {
                float dx = (x - cx) / cx;
                float d2 = dx*dx + dy*dy;
                float factor = 1.0f - vignette_strength * d2;
                if(factor < 0.0f) factor = 0.0f;
                row[x].r = (unsigned char)(row[x].r * factor);
                row[x].g = (unsigned char)(row[x].g * factor);
                row[x].b = (unsigned char)(row[x].b * factor);
            }
        }
    }
    if(use_dof) {
        int w = render_width, h = render_height;
        int hw = w / 2, hh = h / 2;
        int qw = w / 4, qh = h / 4;

        // Allocate temporary buffers for pyramid levels
        Pixel *level1 = malloc((size_t)hw * hh * sizeof(Pixel));
        Pixel *level2 = malloc((size_t)qw * qh * sizeof(Pixel));
        if (!level1 || !level2) {
            fprintf(stderr, "DOF pyramid alloc failed\n");
            free(level1); free(level2);
            goto skip_dof;
        }

        // Level 1
        // 2x2 box downsample from original framebuffer
        for (int y = 0; y < hh; y++) {
            for (int x = 0; x < hw; x++) {
                int sx0 = x * 2, sx1 = x * 2 + 1;
                int sy0 = y * 2, sy1 = y * 2 + 1;
                if (sx1 >= w) sx1 = w - 1;
                if (sy1 >= h) sy1 = h - 1;

                int r = 0, g = 0, b = 0, cnt = 0;
                for (int sy = sy0; sy <= sy1; sy++) {
                    Pixel *src = fb + sy * w;
                    for (int sx = sx0; sx <= sx1; sx++) {
                        r += src[sx].r; g += src[sx].g; b += src[sx].b;
                        cnt++;
                    }
                }
                level1[y * hw + x].r = r / cnt;
                level1[y * hw + x].g = g / cnt;
                level1[y * hw + x].b = b / cnt;
            }
        }
        // Gentle Gaussian blur on half‑res
        gauss_blur(level1, level1, hw, hh, 1, 0.8f);

        // Level 2 
        for (int y = 0; y < qh; y++) {
            for (int x = 0; x < qw; x++) {
                int sx0 = x * 2, sx1 = x * 2 + 1;
                int sy0 = y * 2, sy1 = y * 2 + 1;
                if (sx1 >= hw) sx1 = hw - 1;
                if (sy1 >= hh) sy1 = hh - 1;

                int r = 0, g = 0, b = 0, cnt = 0;
                for (int sy = sy0; sy <= sy1; sy++) {
                    Pixel *src = level1 + sy * hw;
                    for (int sx = sx0; sx <= sx1; sx++) {
                        r += src[sx].r; g += src[sx].g; b += src[sx].b;
                        cnt++;
                    }
                }
                level2[y * qw + x].r = r / cnt;
                level2[y * qw + x].g = g / cnt;
                level2[y * qw + x].b = b / cnt;
            }
        }
        // Gaussian blur on quarter‑res
        gauss_blur(level2, level2, qw, qh, 1, 0.8f);

        // DOF composite
        for (int y = 0; y < h; y++) {
            Pixel *out_row = fb + y * w;
            for (int x = 0; x < w; x++) {
                float z = zbuffer[y * w + x];
                if (z >= FAR_DEPTH) continue;   // sky stays sharp (original colour untouched)

                float coc = fabsf(z - focal_depth) * aperture;
                // Map CoC to a level index (0..2)
                float level = (coc / max_blur_radius);
                if (level > 1.0f) level = 1.0f;  // clamp
                level *= 2.0f;                   // map to 0..2
                int level_low = (int)level;
                int level_high = level_low + 1;
                if (level_high > 2) level_high = 2;
                float t = level - level_low;

                float u = (x + 0.5f) / w;
                float v = (y + 0.5f) / h;

                // Sample from the two appropriate pyramid levels
                Pixel p_low, p_high;
                if (level_low == 0)      p_low  = fb[y * w + x];   // original sharp
                else if (level_low == 1) p_low  = sample_bilinear(level1, hw, hh, u, v);
                else                     p_low  = sample_bilinear(level2, qw, qh, u, v);

                if (level_high == 0)      p_high = fb[y * w + x];
                else if (level_high == 1) p_high = sample_bilinear(level1, hw, hh, u, v);
                else                      p_high = sample_bilinear(level2, qw, qh, u, v);

                // Linear blend
                out_row[x].r = (unsigned char)(p_low.r * (1.0f - t) + p_high.r * t + 0.5f);
                out_row[x].g = (unsigned char)(p_low.g * (1.0f - t) + p_high.g * t + 0.5f);
                out_row[x].b = (unsigned char)(p_low.b * (1.0f - t) + p_high.b * t + 0.5f);
            }
        }

        free(level1);
        free(level2);
        skip_dof: ;
    }


    if(use_bloom) {
        int w = render_width, h = render_height;
        int hw = w / 2, hh = h / 2;

        // Allocate half‑res buffer for bright areas
        Pixel *bloom_low = malloc((size_t)hw * hh * sizeof(Pixel));
        if(!bloom_low) {
            fprintf(stderr, "Bloom alloc failed\n");
            goto skip_bloom;
        }

        // downsample
        for(int y = 0; y < hh; y++) {
            Pixel *dst = bloom_low + y * hw;
            for(int x = 0; x < hw; x++) {
                int sx0 = x*2, sx1 = x*2+1;
                int sy0 = y*2, sy1 = y*2+1;
                if(sx1 >= w) sx1 = w-1;
                if(sy1 >= h) sy1 = h-1;

                int r=0, g=0, b=0, cnt=0;
                for(int sy=sy0; sy<=sy1; sy++) {
                    Pixel *src = fb + sy * w;
                    for(int sx=sx0; sx<=sx1; sx++) {
                        r += src[sx].r; g += src[sx].g; b += src[sx].b;
                        cnt++;
                    }
                }
                unsigned char avg_r = r/cnt, avg_g = g/cnt, avg_b = b/cnt;
                float brightness = (avg_r + avg_g + avg_b) / (255.0f * 3.0f);
                if(brightness > bloom_threshold) {
                    float scale = (brightness - bloom_threshold) / (1.0f - bloom_threshold);
                    dst[x].r = (unsigned char)(avg_r * scale);
                    dst[x].g = (unsigned char)(avg_g * scale);
                    dst[x].b = (unsigned char)(avg_b * scale);
                } else {
                    dst[x].r = dst[x].g = dst[x].b = 0;
                }
            }
        }

        // Gaussian blur
        gauss_blur(bloom_low, bloom_low, hw, hh, 2, 1.2f);

        // Upsample and add to original framebuffer
        for(int y = 0; y < h; y++) {
            Pixel *row = fb + y * w;
            for(int x = 0; x < w; x++) {
                int lx = x/2, ly = y/2;
                if(lx >= hw) lx = hw-1;
                if(ly >= hh) ly = hh-1;
                Pixel *b = &bloom_low[ly * hw + lx];
                int r = row[x].r + (int)(b->r * bloom_intensity);
                int g = row[x].g + (int)(b->g * bloom_intensity);
                int bv = row[x].b + (int)(b->b * bloom_intensity);
                row[x].r = r > 255 ? 255 : (unsigned char)r;
                row[x].g = g > 255 ? 255 : (unsigned char)g;
                row[x].b = bv > 255 ? 255 : (unsigned char)bv;
            }
        }

        free(bloom_low);
        skip_bloom: ;
    }

    if(use_png) {
        /* Allocate flat array for stb_image_write */
        unsigned char *png_data = (unsigned char *)malloc((size_t)out_w * out_h * 3);
        if(!png_data){ fprintf(stderr, "png malloc failed\n"); free(tubes); return 1; }

        for(int y = 0; y < out_h; y++){
            for(int x = 0; x < out_w; x++){
                float u  = (x + 0.5f) / out_w;
                float v  = (y + 0.5f) / out_h;
                float du = 0.5f / out_w;
                float dv = 0.5f / out_h;

                int sx0 = (int)((u-du)*render_width);
                int sx1 = (int)((u+du)*render_width);
                int sy0 = (int)((v-dv)*render_height);
                int sy1 = (int)((v+dv)*render_height);
                if(sx0 < 0) sx0 = 0;
                if(sx1 > render_width-1) sx1 = render_width-1;
                if(sy0 < 0) sy0 = 0;
                if(sy1 > render_height-1) sy1 = render_height-1;
                if(sx0 > sx1) sx0 = sx1;
                if(sy0 > sy1) sy0 = sy1;

                int r=0, g=0, b=0, count=0;
                for(int sy=sy0; sy<=sy1; sy++){
                    long long row_offset = (long long)sy * render_width;
                    for(int sx=sx0; sx<=sx1; sx++){
                        Pixel *p = &fb[row_offset + sx];
                        r += p->r;
                        g += p->g;
                        b += p->b;
                        count++;
                    }
                }
                int idx = (y * out_w + x) * 3;
                png_data[idx+0] = count ? r/count : 0;
                png_data[idx+1] = count ? g/count : 0;
                png_data[idx+2] = count ? b/count : 0;
            }
        }
        if(!stbi_write_png(output, out_w, out_h, 3, png_data, out_w * 3)){
            fprintf(stderr, "failed to write %s\n", output);
        }
        free(png_data);
    } else {
        /* Original PPM output */
        FILE *f = fopen(output, "wb");
        if(!f){ fprintf(stderr, "failed to open %s\n", output); free(tubes); return 1; }
        fprintf(f, "P6\n%d %d\n255\n", out_w, out_h);

        for(int y = 0; y < out_h; y++){
            for(int x = 0; x < out_w; x++){
                float u  = (x + 0.5f) / out_w;
                float v  = (y + 0.5f) / out_h;
                float du = 0.5f / out_w;
                float dv = 0.5f / out_h;

                int sx0 = (int)((u-du)*render_width);
                int sx1 = (int)((u+du)*render_width);
                int sy0 = (int)((v-dv)*render_height);
                int sy1 = (int)((v+dv)*render_height);
                if(sx0 < 0) sx0 = 0;
                if(sx1 > render_width-1) sx1 = render_width-1;
                if(sy0 < 0) sy0 = 0;
                if(sy1 > render_height-1) sy1 = render_height-1;
                if(sx0 > sx1) sx0 = sx1;
                if(sy0 > sy1) sy0 = sy1;

                int r=0, g=0, b=0, count=0;
                for(int sy=sy0; sy<=sy1; sy++){
                    long long row_offset = (long long)sy * render_width;
                    for(int sx=sx0; sx<=sx1; sx++){
                        Pixel *p = &fb[row_offset + sx];
                        r += p->r;
                        g += p->g;
                        b += p->b;
                        count++;
                    }
                }
                unsigned char px[3] = {
                    count ? r/count : 0,
                    count ? g/count : 0,
                    count ? b/count : 0
                };
                fwrite(px, 1, 3, f);
            }
        }
        fclose(f);
    }

    free(tubes);
    renderer_cleanup();    /* free framebuffer and zbuffer */

    return 0;
}
