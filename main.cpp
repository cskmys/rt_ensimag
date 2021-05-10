#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

using namespace std;
using namespace glm;

vec3 center = vec3(0.0f, 0.0f, -1.0f);// vec3(0, 1, 93/255, 1);
float radius = 0.5f;// 1.0
vec3 gl_FragCoord;

////////////////////////////////////////////////////
// debug
bool pE = false;

template<typename T>
void pVal(T val, const string& s = ""){
    if(pE){
        string str = s;
        if(!str.empty()){
            str.append(" ");
        }
        cerr << str << to_string(val) << endl;
    }
}

float max(float a, float b){
    return glm::max(a, b);
}

float min(float a, float b){
    return glm::min(a, b);
}
////////////////////////////////////////////////////
// vector operations
bool refract(vec3 v, vec3 n, float ni_over_nt, vec3 &refracted){
    vec3 uv = normalize(v);
    float dt = dot(uv, n);
    float discriminant = 1.0f - (ni_over_nt * ni_over_nt * (1 - (dt * dt)));
    if(discriminant > 0){
        refracted = ni_over_nt * (uv - (n * dt)) - (n * sqrt(discriminant));
        return true;
    } else {
        return false;
    }
}

////////////////////////////////////////////////////
// Ray class
struct Ray{
    vec3 origin;
    vec3 direction;
    Ray():origin(vec3(0.0)), direction(vec3(0.0)){}
    Ray(vec3 a, vec3 b): origin(a), direction(b){}
};

vec3 point_at_parameter(Ray r, float t){
    vec3 pt = r.origin + (t * r.direction);
    return pt;
}

////////////////////////////////////////////////////
// Material
#define MATTE 0
#define METAL 1
#define DIELEC 2

struct material{
    int type;
    vec4 albedo;
    vec2 ref_idx;
    material():type(0), albedo(vec4(0.0)), ref_idx(vec2(0.0)){}
    material(int type, vec4 albedo):type(type), albedo(albedo), ref_idx(vec2(0.0)){}
    material(int type, vec2 ref_idx):type(type), albedo(vec4(0.0)), ref_idx(ref_idx){}
};

////////////////////////////////////////////////////
// Hittable
struct aabb{
    vec3 vmin;
    vec3 vmax;
    aabb():vmin(vec3(0.0f)), vmax(vec3(0.0f)){}
    aabb(vec3 vmin, vec3 vmax):vmin(vmin), vmax(vmax){}
};

bool hit_aabb(aabb box, Ray r, float t_min, float t_max){
    for (int i = 0; i < 3; ++i) {
        float invD = 1.0f / r.direction[i];
        float t0 = (box.vmin[i] - r.origin[i]) * invD;
        float t1 = (box.vmax[i] - r.origin[i]) * invD;
        if (invD < 0.0f){
            float temp = t0;
            t0 = t1;
            t1 = temp;
        }
        t_min = t0 > t_min ? t0 : t_min;
        t_max = t1 < t_max ? t1 : t_max;
        if(t_max <= t_min){
            return false;
        }
    }
    return true;
}

struct hit_record{
    float t; // parameter
    vec3 p; // hit point
    vec3 normal; // surface normal at hit point
    material m; // material of sphere
    hit_record():t(0.0), p(vec3(0.0)), normal(vec3(0.0)), m(){}
};

struct sphere{
    vec3 center;
    float radius;
    material m;
    aabb box;
    sphere():center(vec3(0.0)), radius(0.0), m(), box(){}
    sphere(vec3 center, float radius, material m):center(center), radius(radius), m(m), box() {}
};

aabb get_sphere_aabb(sphere &s){
    aabb box = aabb(s.center - vec3(s.radius), center + vec3(s.radius));
    return box;
}

bool check_sphere_hit_rec(sphere s, Ray r, float temp, float t_min, float t_max, hit_record &rec){
    if(temp < t_max && temp > t_min){
        rec.t = temp;
        rec.p = point_at_parameter(r, rec.t);
        rec.normal = (rec.p - s.center) / s.radius;
        rec.m = s.m;
        return true;
    }
    return false;
}

bool hit_sphere(sphere s, Ray r, float t_min, float t_max, hit_record &rec){
    if(!hit_aabb(s.box, r, t_min, t_max)){
        return false;
    }
    vec3 oc = r.origin - s.center;
    float a = dot(r.direction, r.direction);
    float b = dot(r.direction, oc);
    float c = dot(oc, oc) - (s.radius * s.radius);
    float discriminant = (b * b) - (a * c); // 4 in 4ac after sqrt becomes 2 and gets divided by 2a
    if(discriminant > 0){
        float temp = (-b - sqrt(discriminant)) / a;
        hit_record hr;
        if(check_sphere_hit_rec(s, r, temp, t_min, t_max, hr)){
            rec = hr;
            return true;
        }
        temp = (-b + sqrt(discriminant)) / a;
        if(check_sphere_hit_rec(s, r, temp, t_min, t_max, hr)){
            rec = hr;
            return true;
        }
    }
    return false;
}

////////////////////////////////////////////////////
// Hittable List
#define NB_SPHERE_MAX 5
struct sphere_list{
    sphere s[NB_SPHERE_MAX];
    int list_size;
    sphere_list():list_size(0){}
};

bool hit_sphere_list(sphere_list s_list, Ray r, float t_min, float t_max, hit_record &rec){
    hit_record temp_rec;
    bool hit_anything = false;
    float closest_so_far = t_max;
    for (int i = 0; i < s_list.list_size; ++i) {
        if(hit_sphere(s_list.s[i], r, t_min, closest_so_far, temp_rec)){
            hit_anything = true;
            closest_so_far = temp_rec.t;
            rec = temp_rec;
        }
    }
    return hit_anything;
}

sphere_list setup_aabb_sphere_list(sphere_list s_list){
    sphere_list aabb_sphere_list = s_list;
    for (int i = 0; i < s_list.list_size; ++i) {
        aabb_sphere_list.s[i].box = get_sphere_aabb(s_list.s[i]);
    }
    return aabb_sphere_list;
}

////////////////////////////////////////////////////
// stack
struct stackFrame{
    Ray r;
    vec4 I;
    int depth;
    stackFrame(): r(Ray()), I(vec4(0.0)), depth(0){}
    stackFrame(Ray r, vec4 I, int depth): r(r), I(I), depth(depth){}
};

#define MAX_BOUNCE 4
#define STACK_SIZ (MAX_BOUNCE * 2)
int sp = 0;
stackFrame Stack[STACK_SIZ];

void push(stackFrame dat){
    if(dat.depth < MAX_BOUNCE){
        if(sp < STACK_SIZ){
            Stack[sp] = dat;
            ++sp;
        }
    }
}

stackFrame pop(){
    stackFrame dat;
    if(sp > 0){
        --sp;
        dat = Stack[sp];
    }
    return dat;
}

bool isStackEmpty(){
    bool res = false;
    if(sp <= 0){
        res = true;
    }
    return res;
}

void flushStack(){
    sp = 0;
}

///////////////////////////////////////////////////////////////////
// scatter
vec3 random_in_unit_sphere() {
    vec3 p = normalize(vec3(gl_FragCoord.x, gl_FragCoord.y, gl_FragCoord.z));
    return p;
}

bool scatter_matte(Ray r_in, hit_record rec, vec4 &attenuation, Ray &scattered){
    vec3 target = rec.p + rec.normal + random_in_unit_sphere(); // p + N is the new point(center of imaginary sphere) in direction of random point in unit sphere
    scattered = Ray(rec.p, target - rec.p);
    attenuation = rec.m.albedo;
    return true;
}

bool scatter_metal(Ray r_in, hit_record rec, vec4 &attenuation, Ray &scattered){
    vec3 norm_rin_dir = normalize(r_in.direction);
    vec3 reflected = reflect(norm_rin_dir, rec.normal);
    scattered = Ray(rec.p, reflected);
    attenuation = rec.m.albedo;
    return (dot(scattered.direction, rec.normal) > 0);
}

float fresnelDielec(vec3 incidence, vec3 outward_normal, vec3 transmitted, float ni, float nt){
    float cos_i = dot(incidence, outward_normal)/(length(incidence) * length(outward_normal));
    float cos_t = dot(transmitted, outward_normal)/(length(transmitted) * length(outward_normal));
    float nt_cos_i = nt * cos_i;
    float ni_cos_t = ni * cos_t;
    float r_par = (nt_cos_i - ni_cos_t) / (nt_cos_i + ni_cos_t);
    float ni_cos_i = ni * cos_i;
    float nt_cos_t = nt * cos_t;
    float r_per = (ni_cos_i - nt_cos_t) / (ni_cos_i + nt_cos_t);
    float f = (float)(pow(r_par, 2) + pow(r_per, 2)) / 2.0f;
    return f;
}

bool scatter_dielec(Ray r_in, hit_record rec, vec4 &fresnel, Ray &r_ref, Ray &r_trans){
    vec3 outward_normal;
    vec3 reflected = reflect(r_in.direction, rec.normal);
    vec3 refracted;
    float ni;
    float nt;
    if(dot(r_in.direction, rec.normal) > 0){
        outward_normal = -rec.normal;
        ni = rec.m.ref_idx.x;
        nt = 1.0f;
    } else {
        outward_normal = rec.normal;
        ni = 1.0f;
        nt = rec.m.ref_idx.x;
    }
    if(refract(r_in.direction, outward_normal, ni/nt, refracted)){
        float f = fresnelDielec(r_in.direction, outward_normal, refracted, ni, nt);
        fresnel = vec4(vec3(f), 1.0);
        r_ref = Ray(rec.p, reflected);
        r_trans = Ray(rec.p, refracted);
    } else {
        fresnel = vec4(1.0);
        r_ref = Ray(rec.p, reflected);
        r_trans = Ray(vec3(0.0), vec3(0.0));
    }
    return true;
}

///////////////////////////////////////////////////////////////////
// color
vec4 getColorFromEnvironment(vec3 direction){
//    vec3 pos = direction;
//    vec2 uv = vec2(0.0);
//    uv.x = atan(pos.z, pos.x) * 0.5;
//    uv.y = asin(pos.y);
//    uv = uv / M_PI + 0.5;
//    vec4 envCol = texture(envMap, uv);
//    return envCol;
    float t = 0.5f * (direction.y + 1.0f);
    return ( (1.0f - t) * vec4(1.0f) ) + (t * vec4(0.5f, 0.7f, 1.0f, 1.0f));
}

#define MAX_FLOAT	999999999
vec4 color(Ray r, sphere_list s, int depth){
    hit_record rec;
    vec4 col = vec4(vec3(0.0), 1.0);

    if (hit_sphere_list(s, r, 0.001, float(MAX_FLOAT), rec)){
        ++depth;
        if(depth < MAX_BOUNCE){
            Ray scattered;
            Ray transmitted;
            vec4 attenuation;
            bool res = false;
            bool trans = false;
            switch (rec.m.type) {
                case MATTE:
                    res = scatter_matte(r, rec, attenuation, scattered);
                    break;
                case METAL:
                    res = scatter_metal(r, rec, attenuation, scattered);
                    break;
                case DIELEC:
                    res = scatter_dielec(r, rec, attenuation, scattered, transmitted);
                    if(transmitted.origin != vec3(0.0)){
                        trans = true;
                    }
                    break;
            }
            if(res){
                if(trans){
                    col += vec4(vec3(1.0f) - vec3(attenuation), 1.0f) * color(transmitted, s, depth);
                }
                col += attenuation * color(scattered, s, depth);
            }
        }
    } else {
        vec3 unit_direction = normalize(r.direction);
        col += getColorFromEnvironment(unit_direction);
    }
    col = vec4(vec3(col), 1.0);
    return col;
}

vec4 color_nonrecursive(Ray r, sphere_list s){
    vec4 col = vec4(vec3(0.0), 1.0);
    stackFrame d = stackFrame(r, vec4(1.0f), -1);
    push(d);
    while(!isStackEmpty()){
        d = pop();
        vec4 I = d.I;

        hit_record rec;
        if (hit_sphere_list(s, d.r, 0.001, float(MAX_FLOAT), rec)){

            d.depth++;

            if(d.depth < MAX_BOUNCE){
                Ray scattered;
                Ray transmitted;
                vec4 attenuation;
                bool res = false;
                bool trans = false;
                switch (rec.m.type) {
                    case MATTE:
                        res = scatter_matte(d.r, rec, attenuation, scattered);
                        break;
                    case METAL:
                        res = scatter_metal(d.r, rec, attenuation, scattered);
                        break;
                    case DIELEC:
                        res = scatter_dielec(d.r, rec, attenuation, scattered, transmitted);
                        if(transmitted.origin != vec3(0.0)) {
                            trans = true;
                        }
                        break;
                }
                if(res){
                    d.I = I * attenuation;
                    d.r = scattered;
                    push(d);
                    if(trans){
                        d.I = I * vec4(vec3(1.0f) - vec3(attenuation), 1.0f);
                        d.r = transmitted;
                        push(d);
                    }
                }
            }
        } else {
            vec3 unit_direction = normalize(d.r.direction);
            col += I * getColorFromEnvironment(unit_direction);
        }
    }
    col = vec4(vec3(col), 1.0);
    return col;
}

////////////////////////////////////////////////////
// Camera
class camera{
public:
    camera():origin(vec3(0.0)), lower_left_corner(vec3(-2.0,-1.0,-1.0)), horizontal(vec3(4.0f, 0.0f, 0.0f)), vertical(vec3(0.0f, 2.0f, 0.0f)){

    }
    Ray get_ray(float u, float v) const{
        return {origin, lower_left_corner + (u * horizontal) + (v * vertical)};
    }
    vec3 origin;
    vec3 lower_left_corner;
    vec3 horizontal;
    vec3 vertical;
};
////////////////////////////////////////////////////

int main() {
    int nx = 200;
    int ny = 100;
    int ns = 100;
    cout << "P3\n" << nx << " " << ny << "\n255\n";

    sphere_list s_list;
    s_list.list_size = 4;
    s_list.s[0] = sphere(center, radius, material(MATTE, vec4(0.1f, 0.2f, 0.5f, 1.0f)));
    s_list.s[1] = sphere(vec3(0.0f, -100.5f, -1.0f), 100.0f, material(MATTE, vec4(0.8f, 0.8f, 0.0f, 1.0f)));
    s_list.s[2] = sphere(vec3(1.0f, 0.0f, -1.0f), 0.5f, material(METAL, vec4(0.8f, 0.6f, 0.2f, 1.0f)));
    s_list.s[3] = sphere(vec3(-1.0f, 0.0f, -1.0f), 0.5f, material(DIELEC, vec2(1.5f, 0.0f))); // METAL, vec4(0.8f, 0.8f, 0.8f, 1.0f));

    s_list = setup_aabb_sphere_list(s_list);

    camera cam;
    for(int j = ny - 1; j >= 0; j--){
        for(int i = 0; i < nx; i++){
            vec4 col(0.0);
            for(int s = 0; s < ns; s++){
                gl_FragCoord = vec3(i, j, s);
                float u = (float)(i + drand48()) / float(nx);
                float v = (float)(j + drand48()) / float(ny);
                Ray r = cam.get_ray(u, v);
                col += color(r, s_list, -1);
//                col += color_nonrecursive(r, s_list);
                flushStack();
            }
            col /= float(ns);
            col = sqrt(col);
            int ir = int(255.99f * col[0]);
            int ig = int(255.99f * col[1]);
            int ib = int(255.99f * col[2]);
            cout << ir << " " << ig << " " << ib << "\n";
        }
    }
    return 0;
}

/*
 Steel:n=2.485,k=3.433
 Silver:n=0.177,k=3.638
 Gold:n=0.37,k=2.82
 Copper: n=0.617,k=2.63
*/