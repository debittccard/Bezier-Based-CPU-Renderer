#include <math.h>
#include "../include/types.h"

/* =========================
   VECTOR OPS
   ========================= */

float dot(Vec3 a, Vec3 b){
    return a.x*b.x + a.y*b.y + a.z*b.z;
}

Vec3 cross(Vec3 a, Vec3 b){
    Vec3 r;
    r.x = a.y*b.z - a.z*b.y;
    r.y = a.z*b.x - a.x*b.z;
    r.z = a.x*b.y - a.y*b.x;
    return r;
}

Vec3 normalize(Vec3 v){
    float len = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
    if(len == 0.0f) return v;

    Vec3 r;
    r.x = v.x / len;
    r.y = v.y / len;
    r.z = v.z / len;
    return r;
}

/* =========================
   MATRIX OPS
   ========================= */

Vec4 mat4_mul_vec4(Mat4 mat, Vec4 v){
    Vec4 r;

    r.x = v.x*mat.m[0][0] + v.y*mat.m[0][1] + v.z*mat.m[0][2] + v.w*mat.m[0][3];
    r.y = v.x*mat.m[1][0] + v.y*mat.m[1][1] + v.z*mat.m[1][2] + v.w*mat.m[1][3];
    r.z = v.x*mat.m[2][0] + v.y*mat.m[2][1] + v.z*mat.m[2][2] + v.w*mat.m[2][3];
    r.w = v.x*mat.m[3][0] + v.y*mat.m[3][1] + v.z*mat.m[3][2] + v.w*mat.m[3][3];

    return r;
}

Mat4 matMult(Mat4 a, Mat4 b){
    Mat4 result = {0};

    for(int i = 0; i < 4; i++){
        for(int j = 0; j < 4; j++){
            for(int k = 0; k < 4; k++){
                result.m[i][j] += a.m[i][k] * b.m[k][j];
            }
        }
    }

    return result;
}

Mat4 mat_identity(){
    Mat4 m = {0};

    m.m[0][0] = 1.0f;
    m.m[1][1] = 1.0f;
    m.m[2][2] = 1.0f;
    m.m[3][3] = 1.0f;

    return m;
}

/* =========================
   TRANSFORMS
   ========================= */

Mat4 mat_translate(float tx, float ty, float tz){
    Mat4 m = mat_identity();

    m.m[0][3] = tx;
    m.m[1][3] = ty;
    m.m[2][3] = tz;

    return m;
}

Mat4 mat_scale(float sx, float sy, float sz){
    Mat4 m = mat_identity();

    m.m[0][0] = sx;
    m.m[1][1] = sy;
    m.m[2][2] = sz;

    return m;
}

Mat4 mat_rotate_x(float angle){
    Mat4 m = mat_identity();

    m.m[1][1] = cosf(angle);
    m.m[1][2] = -sinf(angle);
    m.m[2][1] = sinf(angle);
    m.m[2][2] = cosf(angle);

    return m;
}

Mat4 mat_rotate_y(float angle){
    Mat4 m = mat_identity();

    m.m[0][0] = cosf(angle);
    m.m[0][2] = sinf(angle);
    m.m[2][0] = -sinf(angle);
    m.m[2][2] = cosf(angle);

    return m;
}

Mat4 mat_rotate_z(float angle){
    Mat4 m = mat_identity();

    m.m[0][0] = cosf(angle);
    m.m[0][1] = -sinf(angle);
    m.m[1][0] = sinf(angle);
    m.m[1][1] = cosf(angle);

    return m;
}



Mat4 mat_projection(float fov, float aspect, float near, float far){
    Mat4 m = {0};

    float f = 1.0f / tanf(fov * 0.5f);

    m.m[0][0] = f / aspect;
    m.m[1][1] = f;
    m.m[2][2] = far / (far - near);
    m.m[2][3] = (-far * near) / (far - near);
    m.m[3][2] = 1.0f;

    return m;
}



Mat4 mat_look_at(Vec3 eye, Vec3 target, Vec3 up){
    Vec3 f = normalize((Vec3){
        target.x - eye.x,
        target.y - eye.y,
        target.z - eye.z
    });

    Vec3 r = normalize(cross(f, up));
    Vec3 u = cross(r, f);

    Mat4 m = mat_identity();

    m.m[0][0] = r.x;
    m.m[0][1] = r.y;
    m.m[0][2] = r.z;

    m.m[1][0] = u.x;
    m.m[1][1] = u.y;
    m.m[1][2] = u.z;

    m.m[2][0] = -f.x;
    m.m[2][1] = -f.y;
    m.m[2][2] = -f.z;

    m.m[0][3] = -dot(r, eye);
    m.m[1][3] = -dot(u, eye);
    m.m[2][3] =  dot(f, eye);

    return m;
}

/* =========================
   FRUSTUM CULLING (MVP)
   ========================= */

int aabb_in_frustum(AABB box, Mat4 mvp){
    Vec3 corners[8] = {
        {box.min.x, box.min.y, box.min.z},
        {box.max.x, box.min.y, box.min.z},
        {box.min.x, box.max.y, box.min.z},
        {box.max.x, box.max.y, box.min.z},
        {box.min.x, box.min.y, box.max.z},
        {box.max.x, box.min.y, box.max.z},
        {box.min.x, box.max.y, box.max.z},
        {box.max.x, box.max.y, box.max.z}
    };

    for(int i = 0; i < 8; i++){
        Vec4 v = {corners[i].x, corners[i].y, corners[i].z, 1.0f};
        Vec4 c = mat4_mul_vec4(mvp, v);

        if(c.w > 0){
            float nx = c.x / c.w;
            float ny = c.y / c.w;
            float nz = c.z / c.w;

            if(nx >= -1 && nx <= 1 &&
               ny >= -1 && ny <= 1 &&
               nz >= 0 && nz <= 1.1f)
                return 1;
        }
        else if(c.w < 0){
            float nx = c.x / c.w;
            float ny = c.y / c.w;

            if(nx >= -1 && nx <= 1 &&
               ny >= -1 && ny <= 1)
                return 1;
        }
    }

    return 0;
}

/* =========================
   FRUSTUM (PLANE VERSION)
   ========================= */

int aabb_in_frustum_planes(AABB box, Plane planes[6]){
    for(int i = 0; i < 6; i++){
        Plane p = planes[i];

        Vec3 v = {
            (p.a > 0) ? box.max.x : box.min.x,
            (p.b > 0) ? box.max.y : box.min.y,
            (p.c > 0) ? box.max.z : box.min.z
        };

        if(p.a*v.x + p.b*v.y + p.c*v.z + p.d < 0){
            return 0;
        }
    }

    return 1;
}