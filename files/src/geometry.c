#include "../include/types.h"
#include <math.h>

Vec3 bezier_eval(BezierCubic b, float t) {
    float u = 1.0f - t;
    //De Casteljau algorithm
    Vec3 a = {
        u * b.p0.x + t * b.p1.x,
        u * b.p0.y + t * b.p1.y,
        u * b.p0.z + t * b.p1.z
    };
    Vec3 c = {
        u * b.p1.x + t * b.p2.x,
        u * b.p1.y + t * b.p2.y,
        u * b.p1.z + t * b.p2.z
    };
    Vec3 d = {
        u * b.p2.x + t * b.p3.x,
        u * b.p2.y + t * b.p3.y,
        u * b.p2.z + t * b.p3.z
    };

    Vec3 e = {
        u * a.x + t * c.x,
        u * a.y + t * c.y,
        u * a.z + t * c.z
    };
    Vec3 f = {
        u * c.x + t * d.x,
        u * c.y + t * d.y,
        u * c.z + t * d.z
    };

    return (Vec3){
        u * e.x + t * f.x,
        u * e.y + t * f.y,
        u * e.z + t * f.z
    };
}

Vec3 bezier_tangent(BezierCubic b, float t) {
    float u = 1.0f - t;

    Vec3 d0 = { b.p1.x - b.p0.x, b.p1.y - b.p0.y, b.p1.z - b.p0.z };
    Vec3 d1 = { b.p2.x - b.p1.x, b.p2.y - b.p1.y, b.p2.z - b.p1.z };
    Vec3 d2 = { b.p3.x - b.p2.x, b.p3.y - b.p2.y, b.p3.z - b.p2.z };

    float w0 = 3.0f * u * u;
    float w1 = 6.0f * u * t;
    float w2 = 3.0f * t * t;

    return (Vec3){
        w0 * d0.x + w1 * d1.x + w2 * d2.x,
        w0 * d0.y + w1 * d1.y + w2 * d2.y,
        w0 * d0.z + w1 * d1.z + w2 * d2.z
    };
}

/* solve A*t^2 + B*t + C = 0 in [0,1] */
static int solve_quadratic_01(float A, float B, float C, float roots[2]) {
    if (fabsf(A) < 1e-7f) {
        if (fabsf(B) < 1e-7f) return 0;
        float t = -C / B;
        if (t >= 0.0f && t <= 1.0f) {
            roots[0] = t;
            return 1;
        }
        return 0;
    }

    float disc = B * B - 4.0f * A * C;
    if (disc < 0.0f) {
        if (disc > -1e-7f) disc = 0.0f;
        else return 0;
    }

    if (disc == 0.0f) {
        float t = -B / (2.0f * A);
        if (t >= 0.0f && t <= 1.0f) {
            roots[0] = t;
            return 1;
        }
        return 0;
    }

    float sqrt_disc = sqrtf(disc);
    float q = -0.5f * (B + copysignf(sqrt_disc, B));
    float t1 = q / A;
    float t2 = C / q;

    int count = 0;
    if (t1 >= 0.0f && t1 <= 1.0f) roots[count++] = t1;
    if (t2 >= 0.0f && t2 <= 1.0f) roots[count++] = t2;
    return count;
}

/* near-linear check */
static int is_curve_flat(BezierCubic b) {
    float dx = b.p3.x - b.p0.x;
    float dy = b.p3.y - b.p0.y;
    float dz = b.p3.z - b.p0.z;
    float len2 = dx*dx + dy*dy + dz*dz;
    if (len2 < 1e-10f) return 1;

    float t1 = ((b.p1.x - b.p0.x)*dx + (b.p1.y - b.p0.y)*dy + (b.p1.z - b.p0.z)*dz) / len2;
    float t2 = ((b.p2.x - b.p0.x)*dx + (b.p2.y - b.p0.y)*dy + (b.p2.z - b.p0.z)*dz) / len2;

    Vec3 proj1 = { b.p0.x + t1*dx, b.p0.y + t1*dy, b.p0.z + t1*dz };
    Vec3 proj2 = { b.p0.x + t2*dx, b.p0.y + t2*dy, b.p0.z + t2*dz };

    float dist1 = (b.p1.x - proj1.x)*(b.p1.x - proj1.x) +
                  (b.p1.y - proj1.y)*(b.p1.y - proj1.y) +
                  (b.p1.z - proj1.z)*(b.p1.z - proj1.z);

    float dist2 = (b.p2.x - proj2.x)*(b.p2.x - proj2.x) +
                  (b.p2.y - proj2.y)*(b.p2.y - proj2.y) +
                  (b.p2.z - proj2.z)*(b.p2.z - proj2.z);

    float threshold = 1e-6f * len2;
    return (dist1 < threshold && dist2 < threshold);
}

AABB bezier_bounds(BezierCubic b, float radius) {
    AABB box;

    box.min.x = fminf(fminf(b.p0.x, b.p1.x), fminf(b.p2.x, b.p3.x));
    box.min.y = fminf(fminf(b.p0.y, b.p1.y), fminf(b.p2.y, b.p3.y));
    box.min.z = fminf(fminf(b.p0.z, b.p1.z), fminf(b.p2.z, b.p3.z));

    box.max.x = fmaxf(fmaxf(b.p0.x, b.p1.x), fmaxf(b.p2.x, b.p3.x));
    box.max.y = fmaxf(fmaxf(b.p0.y, b.p1.y), fmaxf(b.p2.y, b.p3.y));
    box.max.z = fmaxf(fmaxf(b.p0.z, b.p1.z), fmaxf(b.p2.z, b.p3.z));

    if (!is_curve_flat(b)) {
        float d0x = b.p1.x - b.p0.x;
        float d1x = b.p2.x - b.p1.x;
        float d2x = b.p3.x - b.p2.x;
        float Ax = d2x - 2.0f*d1x + d0x;
        float Bx = 2.0f * (d1x - d0x);
        float Cx = d0x;

        float d0y = b.p1.y - b.p0.y;
        float d1y = b.p2.y - b.p1.y;
        float d2y = b.p3.y - b.p2.y;
        float Ay = d2y - 2.0f*d1y + d0y;
        float By = 2.0f * (d1y - d0y);
        float Cy = d0y;

        float d0z = b.p1.z - b.p0.z;
        float d1z = b.p2.z - b.p1.z;
        float d2z = b.p3.z - b.p2.z;
        float Az = d2z - 2.0f*d1z + d0z;
        float Bz = 2.0f * (d1z - d0z);
        float Cz = d0z;

        float t_candidates[8];
        int tc = 0;
        t_candidates[tc++] = 0.0f;
        t_candidates[tc++] = 1.0f;

        float roots[2];
        int nr;

        nr = solve_quadratic_01(Ax, Bx, Cx, roots);
        for (int i = 0; i < nr; i++) t_candidates[tc++] = roots[i];

        nr = solve_quadratic_01(Ay, By, Cy, roots);
        for (int i = 0; i < nr; i++) t_candidates[tc++] = roots[i];

        nr = solve_quadratic_01(Az, Bz, Cz, roots);
        for (int i = 0; i < nr; i++) t_candidates[tc++] = roots[i];

        int unique_tc = 0;
        for (int i = 0; i < tc; i++) {
            int dup = 0;
            for (int j = 0; j < unique_tc; j++) {
                if (fabsf(t_candidates[i] - t_candidates[j]) < 1e-5f) {
                    dup = 1;
                    break;
                }
            }
            if (!dup) t_candidates[unique_tc++] = t_candidates[i];
        }
        tc = unique_tc;

        for (int i = 0; i < tc; i++) {
            float t = t_candidates[i];
            if (t < 0.0f) t = 0.0f;
            if (t > 1.0f) t = 1.0f;

            Vec3 p = bezier_eval(b, t);
            if (p.x < box.min.x) box.min.x = p.x;
            if (p.x > box.max.x) box.max.x = p.x;
            if (p.y < box.min.y) box.min.y = p.y;
            if (p.y > box.max.y) box.max.y = p.y;
            if (p.z < box.min.z) box.min.z = p.z;
            if (p.z > box.max.z) box.max.z = p.z;
        }
    }

    box.min.x -= radius;
    box.min.y -= radius;
    box.min.z -= radius;
    box.max.x += radius;
    box.max.y += radius;
    box.max.z += radius;

    return box;
}