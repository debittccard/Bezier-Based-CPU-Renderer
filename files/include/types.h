#ifndef TYPES_H
#define TYPES_H

#define NUM_TUBES 100000
#define TUBE_SEGMENTS 10
#define TUBE_SIDES 20
#define MAX_TUBE_SEGMENTS 250
#define MAX_TUBE_SIDES 250
#define MAX_TUBE_VERTS ((MAX_TUBE_SEGMENTS + 1) * MAX_TUBE_SIDES)
#define MAX_SEGMENTS 8

extern int render_width;
extern int render_height;
extern int shadow_w;
extern int shadow_h;

#define OUT_WIDTH 1920
#define OUT_HEIGHT 1080
#define FAR_DEPTH 1e30f

extern int num_threads;

typedef struct{
    float x, y, z;
} Vec3;

typedef struct{
    float x, y, z, w;
} Vec4;

typedef struct{
    float m[4][4];
} Mat4;

typedef struct{
    unsigned char r, g, b;
} Pixel;

typedef struct{
    Vec3 v0, v1, v2;
} Triangle;

typedef struct{
    Triangle* tris;
    int count;
} Mesh;

typedef struct {
    Vec3 p0, p1, p2, p3;
} BezierCubic;

typedef struct {
    Vec3 origin;
    Vec3 dir;
} Ray;

typedef struct {
    Vec3 a;
    Vec3 b;
    float radius;
} Segment;

typedef struct{
    Vec3 min;
    Vec3 max;
}AABB;

typedef struct {
    int segments;
    int sides;
    float radius;
} TubeProperties;

typedef struct {
    BezierCubic world_curve;
    AABB bounds;
    BezierCubic curve;
    TubeProperties tube_props;
    int color_r, color_g, color_b;
    AABB segment_bounds[MAX_TUBE_SEGMENTS];
    int segment_count;
} TubeEntry;

typedef struct {
    Vec3 eye;
    Vec3 target;
    Vec3 up;
    float fov;
    float near_plane;
    float far_plane;
} CameraConfig;

typedef struct {
    float rotate_x;
    float rotate_y;
    float rotate_z;
    float rconst_x;
    float rconst_y;
    float rconst_z;
    float scale;
    float translate_x;
    float translate_y;
    float translate_z;
    float multTranslatex;
    float multTranslatey;
    float multTranslatez;
    float angle_step;
    float translate_step; 
} TransformConfig;

typedef struct {
    CameraConfig camera;
    TransformConfig transform;
    Vec3 light;
    TubeEntry *tubes;
    int tube_count;
    Mat4 light_vp;
} SceneConfig;

typedef struct {
    float a, b, c, d;
} Plane;

// shadow_map is still static, but if you want it dynamic later, same pattern
extern float *shadow_map;

typedef struct {
    float x, y, z; // already normalized to [0,1] for x,y
} LightCoord;

// ... (ThreadBuffers, RenderJob, ShadowJob unchanged)

typedef struct {
    int   ring_x[MAX_TUBE_VERTS];
    int   ring_y[MAX_TUBE_VERTS];
    float ring_z[MAX_TUBE_VERTS];
    Vec3  ring_n[MAX_TUBE_VERTS];
    LightCoord ring_light[MAX_TUBE_VERTS];
    int   ring_valid[MAX_TUBE_VERTS];
    int   ring_ring_valid[MAX_TUBE_SEGMENTS + 1];
} ThreadBuffers;

typedef struct {
    TubeEntry *tubes;
    Mat4 view;
    int start;
    int end;
    int tubes_total;
    float fov;
    Mat4 vp;
    Plane planes[6];
    Vec3 light;
    Vec3 eye;
    float far_plane;
    ThreadBuffers tb;
} RenderJob;

typedef struct {
    TubeEntry *tubes;
    int start, end;
    Mat4 light_vp;
    Plane planes[6];
    float *shadow_local;
    int tube_count;
    int thread_id;
    float rx, ry, rz, scale, tx, ty, tz, angle_step;
    float rcx, rcy, rcz;
    float mtx, mty, mtz, translate_step;
} ShadowJob;

#endif