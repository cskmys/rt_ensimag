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
// Hittable
struct hit_record{
    float t; // parameter
    vec3 p; // hit point
    vec3 normal; // surface normal at hit point
    int material;
    vec4 albedo;
    float ref_idx;
    hit_record():t(0.0), p(vec3(0.0)), normal(vec3(0.0)), material(0), albedo(vec4(0.0)), ref_idx(0.0f){}
};

struct sphere{
    vec3 center;
    float radius;
    int material;
    vec4 albedo;
    float ref_idx;
    sphere():center(vec3(0.0)), radius(0.0), material(0), albedo(vec4(0.0)), ref_idx(0.0f){}
    sphere(vec3 center, float radius, int material, vec4 albedo):center(center), radius(radius), material(material), albedo(albedo), ref_idx(0.0){}
    sphere(vec3 center, float radius, int material, float ref_idx):center(center), radius(radius), material(material), albedo(vec4(0.0)), ref_idx(ref_idx){}
};

bool check_hit_rec(sphere s, Ray r, float temp, float t_min, float t_max, hit_record &rec){
    if(temp < t_max && temp > t_min){
        rec.t = temp;
        rec.p = point_at_parameter(r, rec.t);
        rec.normal = (rec.p - s.center) / s.radius;
        rec.material = s.material;
        rec.albedo = s.albedo;
        rec.ref_idx = s.ref_idx;
        return true;
    }
    return false;
}

bool hit_sphere(sphere s, Ray r, float t_min, float t_max, hit_record &rec){
    vec3 oc = r.origin - s.center;
    float a = dot(r.direction, r.direction);
    float b = dot(r.direction, oc);
    float c = dot(oc, oc) - (s.radius * s.radius);
    float discriminant = (b * b) - (a * c); // 4 in 4ac after sqrt becomes 2 and gets divided by 2a
    if(discriminant > 0){
        float temp = (-b - sqrt(discriminant)) / a;
        hit_record hr;
        if(check_hit_rec(s, r, temp, t_min, t_max, hr)){
            rec = hr;
            return true;
        }
        temp = (-b + sqrt(discriminant)) / a;
        if(check_hit_rec(s, r, temp, t_min, t_max, hr)){
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
// material
#define MATTE 0
#define METAL 1
#define DIELEC 2
vec3 random_in_unit_sphere() {
    vec3 p = normalize(vec3(gl_FragCoord.x, gl_FragCoord.y, gl_FragCoord.z));
    return p;
}

bool scatter_matte(Ray r_in, hit_record rec, vec4 &attenuation, Ray &scattered){
    vec3 target = rec.p + rec.normal + random_in_unit_sphere(); // p + N is the new point(center of imaginary sphere) in direction of random point in unit sphere
    scattered = Ray(rec.p, target - rec.p);
    attenuation = rec.albedo;
    return true;
}

bool scatter_metal(Ray r_in, hit_record rec, vec4 &attenuation, Ray &scattered){
    vec3 norm_rin_dir = normalize(r_in.direction);
    vec3 reflected = reflect(norm_rin_dir, rec.normal);
    scattered = Ray(rec.p, reflected);
    attenuation = rec.albedo;
    return (dot(scattered.direction, rec.normal) > 0);
}

bool scatter_dielec(Ray r_in, hit_record rec, vec4 &attenuation, Ray &scattered){
    vec3 outward_normal;
    vec3 reflected = reflect(r_in.direction, rec.normal);
    float ni_over_nt;
    attenuation = vec4(1.0, 1.0, 1.0, 1.0);
    vec3 refracted;
    if(dot(r_in.direction, rec.normal) > 0){
        outward_normal = -rec.normal;
        ni_over_nt = rec.ref_idx;
    } else {
        outward_normal = rec.normal;
        ni_over_nt = 1 / rec.ref_idx;
    }
    if(refract(r_in.direction, outward_normal, ni_over_nt, refracted)){
        scattered = Ray(rec.p, refracted);
    } else {
        scattered = Ray(rec.p, reflected);
    }
    return true;
}

///////////////////////////////////////////////////////////////////
// color
vec4 getColorFromEnvironment(vec3 direction){
    float t = 0.5f * (direction.y + 1.0f);
    return ( (1.0f - t) * vec4(1.0f) ) + (t * vec4(0.5f, 0.7f, 1.0f, 1.0f));
}

#define MAX_FLOAT	999999999
vec4 color(Ray r, sphere_list s, int depth){
    hit_record rec;
    if (hit_sphere_list(s, r, 0.001, MAX_FLOAT, rec)){

        ++depth;
        if(depth < MAX_BOUNCE){
            Ray scattered;
            vec4 attenuation;
            bool res = false;
            switch (rec.material) {
                case MATTE:
                    res = scatter_matte(r, rec, attenuation, scattered);
                    break;
                case METAL:
                    res = scatter_metal(r, rec, attenuation, scattered);
                    break;
                case DIELEC:
                    res = scatter_dielec(r, rec, attenuation, scattered);
                    break;
            }
            if(res){
                return attenuation * color(scattered, s, depth);
            } else {
                return vec4(0.0, 0.0, 0.0, 1.0);
            }
        } else {
            return vec4(0.0, 0.0, 0.0, 1.0);
        }
    } else {
        vec3 unit_direction = normalize(r.direction);
        vec4 col = getColorFromEnvironment(unit_direction);
        return col;
    }
}

vec4 color_nonrecursive(Ray r, sphere_list s){
    stackFrame d = stackFrame(r, vec4(1.0f), -1);
    push(d);
    while(!isStackEmpty()){
        d = pop();

        hit_record rec;
        if (hit_sphere_list(s, d.r, 0.001, MAX_FLOAT, rec)){

            d.depth++;
            if(d.depth < MAX_BOUNCE){
                Ray scattered;
                vec4 attenuation;
                bool res = false;
                switch (rec.material) {
                    case MATTE:
                        res = scatter_matte(d.r, rec, attenuation, scattered);
                        break;
                    case METAL:
                        res = scatter_metal(d.r, rec, attenuation, scattered);
                        break;
                    case DIELEC:
                        res = scatter_dielec(d.r, rec, attenuation, scattered);
                        break;
                }
                if(res){
                    d.I *= attenuation;
                    d.r = scattered;
                    push(d);
                } else {
                    return vec4(vec3(0.0), 1.0);
                }
            } else {
                return vec4(vec3(0.0), 1.0);
            }
        } else {
            vec3 unit_direction = normalize(d.r.direction);
            vec4 col = getColorFromEnvironment(unit_direction);
            return d.I * col;
        }
    }
    return vec4(1.0, 0.0, 0.0f, 1.0f);
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
    s_list.s[0] = sphere(center, radius, MATTE, vec4(0.1f, 0.2f, 0.5f, 1.0f));
    s_list.s[1] = sphere(vec3(0.0f, -100.5f, -1.0f), 100.0f, MATTE, vec4(0.8f, 0.8f, 0.0f, 1.0f));
    s_list.s[2] = sphere(vec3(1.0f, 0.0f, -1.0f), 0.5f, METAL, vec4(0.8f, 0.6f, 0.2f, 1.0f));
    s_list.s[3] = sphere(vec3(-1.0f, 0.0f, -1.0f), 0.5f, DIELEC, 1.5); // METAL, vec4(0.8f, 0.8f, 0.8f, 1.0f));

    camera cam;
    for(int j = ny - 1; j >= 0; j--){
        for(int i = 0; i < nx; i++){
            vec4 col(0.0);
            for(int s = 0; s < ns; s++){
                gl_FragCoord = vec3(i, j, s);
                float u = (float)(i + drand48()) / float(nx);
                float v = (float)(j + drand48()) / float(ny);
                Ray r = cam.get_ray(u, v);
//                col += color(r, s_list, -1);
                col += color_nonrecursive(r, s_list);
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
