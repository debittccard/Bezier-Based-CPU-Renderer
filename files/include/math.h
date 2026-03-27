#ifndef MATH_H
#define MATH_H
#include "types.h"
#include <math.h>

float dot(Vec3 a, Vec3 b);
Vec3 cross(Vec3 a, Vec3 b);
Vec4 mat4_mul_vec4(Mat4 mat, Vec4 v);
Mat4 matMult(Mat4 a, Mat4 b);
Mat4 mat_identity();
Mat4 mat_projection(float fov, float aspect, float near, float far);
Mat4 mat_translate(float tx, float ty, float tz);
Vec3 normalize(Vec3 v);
Mat4 mat_rotate_x(float angle);
Mat4 mat_rotate_y(float angle);
Mat4 mat_rotate_z(float angle);
Mat4 mat_look_at(Vec3 eye, Vec3 target, Vec3 up);
int aabb_in_frustum(AABB box, Mat4 mvp);
Mat4 mat_scale(float sx, float sy, float sz);
int aabb_in_frustum_planes(AABB box, Plane planes[6]);
#endif