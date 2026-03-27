#ifndef RENDERER_H
#define RENDERER_H
#include "types.h"

void renderer_init();
void draw_pixel(int x, int y, float depth, unsigned char r, unsigned char g, unsigned char b);
void draw_line(int x0, int y0, int x1, int y1);
void drawTriangleFilled(int x0, int y0, int x1, int y1, int x2, int y2, float depth, int r, int g, int b);
void render_bezier(BezierCubic b, Mat4 mvp, int segments);
Pixel (*get_framebuffer())[WIDTH];
void render_tube(BezierCubic b, Mat4 mvp, int segments, int sides,
                 float radius, int cr, int cg, int cb, Vec3 light,
                 ThreadBuffers *tb);
void drawTriangleGouraud(
    int x0, int y0, float i0,
    int x1, int y1, float i1,
    int x2, int y2, float i2,
    float depth, int cr, int cg, int cb);
void extract_frustum_planes(Plane planes[6], Mat4 m);
void shadow_map_init();
void shadow_map_write(int x, int y, float depth);
void render_tube_shadow(BezierCubic b, Mat4 light_mvp, int segments, int sides, float radius);
void drawTriangleGouraudShadow(
    int x0, int y0, float z0, float i0, LightCoord l0,
    int x1, int y1, float z1, float i1, LightCoord l1,
    int x2, int y2, float z2, float i2, LightCoord l2,
    int cr, int cg, int cb
    );
void render_tube_shadow_local(
    BezierCubic b,
    Mat4 light_mvp,
    int segments,
    int sides,
    float radius,
    float *shadow_local);
static inline void shadow_map_write_local(
    float map[SHADOW_H][SHADOW_W],
    int x, int y, float depth);
#endif