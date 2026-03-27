#include "types.h"
#include <math.h>
Vec3 bezier_eval(BezierCubic b, float t){

    float u = 1.0f - t;

    Vec3 a = {
        u*b.p0.x + t*b.p1.x,
        u*b.p0.y + t*b.p1.y,
        u*b.p0.z + t*b.p1.z
    };

    Vec3 c = {
        u*b.p1.x + t*b.p2.x,
        u*b.p1.y + t*b.p2.y,
        u*b.p1.z + t*b.p2.z
    };

    Vec3 d = {
        u*b.p2.x + t*b.p3.x,
        u*b.p2.y + t*b.p3.y,
        u*b.p2.z + t*b.p3.z
    };

    Vec3 e = {
        u*a.x + t*c.x,
        u*a.y + t*c.y,
        u*a.z + t*c.z
    };

    Vec3 f = {
        u*c.x + t*d.x,
        u*c.y + t*d.y,
        u*c.z + t*d.z
    };

    Vec3 result = {
        u*e.x + t*f.x,
        u*e.y + t*f.y,
        u*e.z + t*f.z
    };

    return result;
}

Vec3 bezier_tangent(BezierCubic b, float t){
    float u = 1.0f - t;
    Vec3 d0 = {b.p1.x-b.p0.x, b.p1.y-b.p0.y, b.p1.z-b.p0.z};
    Vec3 d1 = {b.p2.x-b.p1.x, b.p2.y-b.p1.y, b.p2.z-b.p1.z};
    Vec3 d2 = {b.p3.x-b.p2.x, b.p3.y-b.p2.y, b.p3.z-b.p2.z};

    float w0 = 3*u*u;
    float w1 = 6*u*t;
    float w2 = 3*t*t;

    Vec3 result = {
        w0*d0.x + w1*d1.x + w2*d2.x,
        w0*d0.y + w1*d1.y + w2*d2.y,
        w0*d0.z + w1*d1.z + w2*d2.z
    };
    return result;
}


AABB bezier_bounds(BezierCubic b, float radius){
    AABB box;

    Vec3 p = bezier_eval(b, 0.0f);
    box.min = box.max = p;

    // more samples (was 10 → now 20)
    for(int i = 1; i <= 20; i++){
        float t = i / 20.0f;
        Vec3 p = bezier_eval(b, t);

        if(p.x < box.min.x) box.min.x = p.x;
        if(p.y < box.min.y) box.min.y = p.y;
        if(p.z < box.min.z) box.min.z = p.z;

        if(p.x > box.max.x) box.max.x = p.x;
        if(p.y > box.max.y) box.max.y = p.y;
        if(p.z > box.max.z) box.max.z = p.z;
    }

    // padding
    float padding = radius * 3.0f;

    box.min.x -= padding;
    box.min.y -= padding;
    box.min.z -= padding;

    box.max.x += padding;
    box.max.y += padding;
    box.max.z += padding;

    return box;
}