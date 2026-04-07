#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "../include/types.h"
#include "../include/geometry.h"
#include "../include/math.h"

Pixel framebuffer[HEIGHT][WIDTH];
float zbuffer[HEIGHT][WIDTH];
float shadow_map[SHADOW_H][SHADOW_W];
Mat4 light_vp_global;



void renderer_init(){
    for(int j = 0; j < MAX_TUBE_SIDES; j++){
        float a = (float)j / MAX_TUBE_SIDES * 2.0f * 3.14159f;
        global_cos_table[j] = cosf(a);
        global_sin_table[j] = sinf(a);
    }
    for(int y = 0; y < HEIGHT; y++){
        for(int x = 0; x < WIDTH; x++){
            framebuffer[y][x].r = 0;
            framebuffer[y][x].g = 0;
            framebuffer[y][x].b = 0;
            zbuffer[y][x] = FAR_DEPTH;
        }
    }
    for(int y = 0; y < SHADOW_H; y++){
        for(int x = 0; x < SHADOW_W; x++){
            shadow_map[y][x] = FAR_DEPTH;
        }
    }
}

void shadow_map_init(){
    for(int y = 0; y < SHADOW_H; y++){
        for(int x = 0; x < SHADOW_W; x++){
            shadow_map[y][x] = FAR_DEPTH;
        }
    }
}

void shadow_map_write(int x, int y, float depth){
    if(x >= 0 && x < SHADOW_W && y >= 0 && y < SHADOW_H){
        if(depth < shadow_map[y][x])
            shadow_map[y][x] = depth;
    }
}

void draw_pixel(int x, int y, float depth, unsigned char r, unsigned char g, unsigned char b){
    if(x >= 0 && x < WIDTH && y >= 0 && y < HEIGHT){
        if(depth < zbuffer[y][x]){
            zbuffer[y][x] = depth;
            framebuffer[y][x].r = r;
            framebuffer[y][x].g = g;
            framebuffer[y][x].b = b;
        }
    }
}


void render_tube_shadow_local(
    BezierCubic curve,
    Mat4 light_mvp,
    int segments,
    int sides,
    float radius,
    float *shadow_local)
{
    segments = segments/2; if(segments<2) segments=2;
    sides    = sides/2;    if(sides<3)    sides=3;

    for(int s=0; s<=segments; s++){
        float t = (float)s/segments;
        Vec3 center  = bezier_eval(curve, t);
        Vec3 tangent = normalize(bezier_tangent(curve, t));
        Vec3 up = {0,1,0};
        if(fabsf(dot(tangent,up))>0.99f) up=(Vec3){0,0,1};
        Vec3 right = normalize(cross(tangent,up));
        Vec3 up2   = normalize(cross(right,tangent));

        for(int j=0; j<sides; j++){
            float a  = (float)j/sides*2.0f*3.14159f;
            float ca = cosf(a), sa = sinf(a);
            Vec3 p = {
                center.x+radius*(ca*right.x+sa*up2.x),
                center.y+radius*(ca*right.y+sa*up2.y),
                center.z+radius*(ca*right.z+sa*up2.z)
            };
            Vec4 clip = mat4_mul_vec4(light_mvp,(Vec4){p.x,p.y,p.z,1.0f});
            if(fabsf(clip.w)<1e-6f) continue;
            float iw=1.0f/clip.w;
            float nx=clip.x*iw, ny=clip.y*iw, nz=clip.z*iw;
            if(nx<-1||nx>1||ny<-1||ny>1) continue;
            int sx=(int)((nx+1.0f)*0.5f*SHADOW_W);
            int sy=(int)((1.0f-ny)*0.5f*SHADOW_H);
            if(sx<0||sx>=SHADOW_W||sy<0||sy>=SHADOW_H) continue;
            int idx = sy*SHADOW_W+sx;
            if(nz < shadow_local[idx])
                shadow_local[idx] = nz;
        }
    }
}
static inline int is_backface(int x0, int y0, int x1, int y1, int x2, int y2){
    return ((x1-x0)*(y2-y0) - (y1-y0)*(x2-x0)) <= 0;
}

Pixel (*get_framebuffer())[WIDTH]{
    return framebuffer;
}

void drawTriangleGouraudShadow(
    int x0, int y0, float z0, float i0, LightCoord l0,
    int x1, int y1, float z1, float i1, LightCoord l1,
    int x2, int y2, float z2, float i2, LightCoord l2,
    int cr, int cg, int cb)
{
    // reject invalid
    if(!isfinite(z0)||!isfinite(z1)||!isfinite(z2)) return;

    // AREA CULL
    int area = (x1-x0)*(y2-y0) - (y1-y0)*(x2-x0);
    if(area <= 0) return;          // backface
    if(area < 4 && area > -4) return;          // tiny triangle kill

    // sort by y
    if(y0>y1){ int tx=x0,ty=y0; float ti=i0,tz=z0; LightCoord tl=l0;
        x0=x1;y0=y1;i0=i1;z0=z1;l0=l1;x1=tx;y1=ty;i1=ti;z1=tz;l1=tl; }
    if(y0>y2){ int tx=x0,ty=y0; float ti=i0,tz=z0; LightCoord tl=l0;
        x0=x2;y0=y2;i0=i2;z0=z2;l0=l2;x2=tx;y2=ty;i2=ti;z2=tz;l2=tl; }
    if(y1>y2){ int tx=x1,ty=y1; float ti=i1,tz=z1; LightCoord tl=l1;
        x1=x2;y1=y2;i1=i2;z1=z2;l1=l2;x2=tx;y2=ty;i2=ti;z2=tz;l2=tl; }

    int dy02 = y2 - y0;
    if(dy02 == 0) return;

    float inv_y02 = 1.0f / dy02;

    for(int y = y0; y <= y2; y++){
        if(y < 0 || y >= HEIGHT) continue;

        float t_long = (y - y0) * inv_y02;

        int xa = x0 + (int)(t_long * (x2 - x0));
        float ia = i0 + t_long * (i2 - i0);
        float za = z0 + t_long * (z2 - z0);

        LightCoord la = {
            l0.x + t_long*(l2.x - l0.x),
            l0.y + t_long*(l2.y - l0.y),
            l0.z + t_long*(l2.z - l0.z)
        };

        int xb;
        float ib, zb;
        LightCoord lb;

        if(y < y1){
            int dy01 = y1 - y0;
            if(dy01 == 0) continue;

            float t = (y - y0) / (float)dy01;
            xb = x0 + (int)(t * (x1 - x0));
            ib = i0 + t*(i1 - i0);
            zb = z0 + t*(z1 - z0);

            lb = (LightCoord){
                l0.x + t*(l1.x - l0.x),
                l0.y + t*(l1.y - l0.y),
                l0.z + t*(l1.z - l0.z)
            };
        } else {
            int dy12 = y2 - y1;
            if(dy12 == 0) continue;

            float t = (y - y1) / (float)dy12;
            xb = x1 + (int)(t * (x2 - x1));
            ib = i1 + t*(i2 - i1);
            zb = z1 + t*(z2 - z1);

            lb = (LightCoord){
                l1.x + t*(l2.x - l1.x),
                l1.y + t*(l2.y - l1.y),
                l1.z + t*(l2.z - l1.z)
            };
        }

        if(xa > xb){
            int tx = xa; xa = xb; xb = tx;
            float ti = ia; ia = ib; ib = ti;
            float tz = za; za = zb; zb = tz;
            LightCoord tl = la; la = lb; lb = tl;
        }

        if(xb < 0 || xa >= WIDTH) continue;
        if(xa < 0) xa = 0;
        if(xb >= WIDTH) xb = WIDTH - 1;

        int dx = xb - xa;
        float inv_x = (dx != 0) ? 1.0f/dx : 0.0f;

        for(int x = xa; x <= xb; x++){
            float t = (dx != 0) ? (x - xa)*inv_x : 0.0f;

            float depth = za + t*(zb - za);
            if(depth >= zbuffer[y][x]) continue;

            float intensity = ia + t*(ib - ia);
            if(intensity <= 0.05f) continue;

            // clamp once
            if(intensity > 1.0f) intensity = 1.0f;

            // shadow only if worth it
            if(intensity > 0.35f){
                LightCoord lc = {
                    la.x + t*(lb.x - la.x),
                    la.y + t*(lb.y - la.y),
                    la.z + t*(lb.z - la.z)
                };

                if(lc.x >= 0.0f && lc.x <= 1.0f &&
                   lc.y >= 0.0f && lc.y <= 1.0f){

                    int lxi = (int)(lc.x * (SHADOW_W - 1));
                    int lyi = (int)(lc.y * (SHADOW_H - 1));

                    if(lc.z > shadow_map[lyi][lxi] + 0.001f)
                        intensity *= 0.25f;
                }
            }

            zbuffer[y][x] = depth;

            framebuffer[y][x].r = (unsigned char)(intensity * cr);
            framebuffer[y][x].g = (unsigned char)(intensity * cg);
            framebuffer[y][x].b = (unsigned char)(intensity * cb);
        }
    }
}

void render_tube_shadow(BezierCubic b, Mat4 light_mvp, int segments, int sides, float radius){
    segments=segments/2; sides=sides/2;
    if(segments<2) segments=2;
    if(sides<3) sides=3;
    if(segments>MAX_TUBE_SEGMENTS||sides>MAX_TUBE_SIDES) return;

    for(int i=0;i<=segments;i++){
        float t=(float)i/segments;
        Vec3 center=bezier_eval(b,t);
        Vec3 tangent=normalize(bezier_tangent(b,t));
        Vec3 up={0,1,0};
        if(fabsf(dot(tangent,up))>0.99f) up=(Vec3){0,0,1};
        Vec3 right=normalize(cross(tangent,up));
        Vec3 up2=normalize(cross(right,tangent));

        for(int j=0;j<sides;j++){
            float ca=global_cos_table[j*(MAX_TUBE_SIDES/sides)];
            float sa=global_sin_table[j*(MAX_TUBE_SIDES/sides)];
            Vec3 p={
                center.x+radius*(ca*right.x+sa*up2.x),
                center.y+radius*(ca*right.y+sa*up2.y),
                center.z+radius*(ca*right.z+sa*up2.z)
            };
            Vec4 clip=mat4_mul_vec4(light_mvp,(Vec4){p.x,p.y,p.z,1.0f});
            if(fabsf(clip.w)<1e-6f) continue;
            float iw=1.0f/clip.w;
            float ndcx=clip.x*iw, ndcy=clip.y*iw, ndcz=clip.z*iw;
            if(ndcx<-1||ndcx>1||ndcy<-1||ndcy>1) continue;
            int sx=(int)((ndcx+1.0f)*0.5f*SHADOW_W);
            int sy=(int)((1.0f-ndcy)*0.5f*SHADOW_H);
            shadow_map_write(sx,sy,ndcz);
        }
    }
}

void render_tube(BezierCubic b, Mat4 mvp, int segments, int sides,
                 float radius, int cr, int cg, int cb, Vec3 light,
                 ThreadBuffers *tb)
{
    
    // LOD 
    
    Vec3 mid = bezier_eval(b, 0.5f);
    float dist2 = mid.x*mid.x + mid.y*mid.y + (mid.z+100.0f)*(mid.z+100.0f);
    if(dist2 > 200.0f*200.0f){
        return; // already culled anyway
    }
    else if(dist2 > 120.0f*120.0f){
        segments = 3;
        sides = 3;
    }
    else if(dist2 > 60.0f*60.0f){
        segments = 4;
        sides = 4;
    }

    if(segments < 2) segments = 2;
    if(sides < 3) sides = 3;
    if(segments > MAX_TUBE_SEGMENTS) segments = MAX_TUBE_SEGMENTS;
    if(sides > MAX_TUBE_SIDES) sides = MAX_TUBE_SIDES;

   
    int step = MAX_TUBE_SIDES / sides;

 
    for(int i = 0; i <= segments; i++){
        tb->ring_ring_valid[i] = 0;
        int base = i * MAX_TUBE_SIDES;
        for(int j = 0; j < sides; j++){
            tb->ring_valid[base + j] = 0;
        }
    }

    light = normalize(light);
    //biuld rings
    for(int i = 0; i <= segments; i++){
        float t = (float)i / segments;

        Vec3 center = bezier_eval(b, t);
        Vec3 tangent = normalize(bezier_tangent(b, t));

        // stable frame
        Vec3 up = {0,1,0};
        if(fabsf(dot(tangent, up)) > 0.99f)
            up = (Vec3){0,0,1};

        Vec3 right = normalize(cross(tangent, up));
        Vec3 up2   = cross(right, tangent); // already orthogonal

        int base = i * MAX_TUBE_SIDES;
        int valid_count = 0;

        for(int j = 0; j < sides; j++){
            int idx = base + j;

            // lookup tabble kinda buggy rn
            float ca = global_cos_table[j * step];
            float sa = global_sin_table[j * step];

            // position
            Vec3 p = {
                center.x + radius*(ca*right.x + sa*up2.x),
                center.y + radius*(ca*right.y + sa*up2.y),
                center.z + radius*(ca*right.z + sa*up2.z)
            };

            // clip transform
            Vec4 clip = mat4_mul_vec4(mvp, (Vec4){p.x,p.y,p.z,1.0f});
            if(fabsf(clip.w) < 1e-6f) continue;

            float iw = 1.0f / clip.w;

            tb->ring_x[idx] = (int)((clip.x*iw + 1.0f) * 0.5f * WIDTH);
            tb->ring_y[idx] = (int)((1.0f - clip.y*iw) * 0.5f * HEIGHT);
            tb->ring_z[idx] = clip.z * iw;

            // normals
            tb->ring_n[idx] = (Vec3){
                ca*right.x + sa*up2.x,
                ca*right.y + sa*up2.y,
                ca*right.z + sa*up2.z
            };

            // shadow coord
            Vec4 lc = mat4_mul_vec4(light_vp_global, (Vec4){p.x,p.y,p.z,1.0f});
            if(fabsf(lc.w) > 1e-6f){
                float ilw = 1.0f / lc.w;
                tb->ring_light[idx] = (LightCoord){
                    lc.x * ilw,
                    lc.y * ilw,
                    lc.z * ilw
                };
            }

            tb->ring_valid[idx] = 1;
            valid_count++;
        }

        if(valid_count == sides)
            tb->ring_ring_valid[i] = 1;
    }

    // triangle pass
    for(int i = 0; i < segments; i++){
        if(!tb->ring_ring_valid[i] || !tb->ring_ring_valid[i+1]) continue;

        int b0 = i * MAX_TUBE_SIDES;
        int b1 = (i+1) * MAX_TUBE_SIDES;

        for(int j = 0; j < sides; j++){
            int next = (j+1) % sides;

            int a = b0 + j;
            int b = b0 + next;
            int c = b1 + j;
            int d = b1 + next;

            float amb = 0.2f;

            float ia = amb + (1.0f-amb)*fmaxf(0, dot(tb->ring_n[a], light));
            float ib = amb + (1.0f-amb)*fmaxf(0, dot(tb->ring_n[b], light));
            float ic = amb + (1.0f-amb)*fmaxf(0, dot(tb->ring_n[c], light));
            float id = amb + (1.0f-amb)*fmaxf(0, dot(tb->ring_n[d], light));

            if(!is_backface(tb->ring_x[a], tb->ring_y[a],
                            tb->ring_x[b], tb->ring_y[b],
                            tb->ring_x[c], tb->ring_y[c])){
                drawTriangleGouraudShadow(
                    tb->ring_x[a], tb->ring_y[a], tb->ring_z[a], ia, tb->ring_light[a],
                    tb->ring_x[b], tb->ring_y[b], tb->ring_z[b], ib, tb->ring_light[b],
                    tb->ring_x[c], tb->ring_y[c], tb->ring_z[c], ic, tb->ring_light[c],
                    cr, cg, cb);
            }

            if(!is_backface(tb->ring_x[b], tb->ring_y[b],
                            tb->ring_x[d], tb->ring_y[d],
                            tb->ring_x[c], tb->ring_y[c])){
                drawTriangleGouraudShadow(
                    tb->ring_x[b], tb->ring_y[b], tb->ring_z[b], ib, tb->ring_light[b],
                    tb->ring_x[d], tb->ring_y[d], tb->ring_z[d], id, tb->ring_light[d],
                    tb->ring_x[c], tb->ring_y[c], tb->ring_z[c], ic, tb->ring_light[c],
                    cr, cg, cb);
            }
        }
    }
}

void extract_frustum_planes(Plane planes[6], Mat4 m){
    planes[0]=(Plane){m.m[0][3]+m.m[0][0],m.m[1][3]+m.m[1][0],m.m[2][3]+m.m[2][0],m.m[3][3]+m.m[3][0]};
    planes[1]=(Plane){m.m[0][3]-m.m[0][0],m.m[1][3]-m.m[1][0],m.m[2][3]-m.m[2][0],m.m[3][3]-m.m[3][0]};
    planes[2]=(Plane){m.m[0][3]+m.m[0][1],m.m[1][3]+m.m[1][1],m.m[2][3]+m.m[2][1],m.m[3][3]+m.m[3][1]};
    planes[3]=(Plane){m.m[0][3]-m.m[0][1],m.m[1][3]-m.m[1][1],m.m[2][3]-m.m[2][1],m.m[3][3]-m.m[3][1]};
    planes[4]=(Plane){m.m[0][3]+m.m[0][2],m.m[1][3]+m.m[1][2],m.m[2][3]+m.m[2][2],m.m[3][3]+m.m[3][2]};
    planes[5]=(Plane){m.m[0][3]-m.m[0][2],m.m[1][3]-m.m[1][2],m.m[2][3]-m.m[2][2],m.m[3][3]-m.m[3][2]};

    for(int i=0;i<6;i++){
        float len=sqrtf(planes[i].a*planes[i].a+planes[i].b*planes[i].b+planes[i].c*planes[i].c);
        planes[i].a/=len; planes[i].b/=len; planes[i].c/=len; planes[i].d/=len;
    }
}