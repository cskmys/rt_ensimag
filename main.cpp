#include <iostream>
#include <glm/glm.hpp>
#include <glm/gtx/string_cast.hpp>

using namespace std;
using namespace glm;

vec3 center = vec3(0.0f, 0.0f, -1.0f);// vec3(0, 1, 93/255, 1);
float radius = 0.5f;// 1.0
vec3 gl_FragCoord;
vec3 lightPosition = vec3(2.0);
float eta = 1.5;
float shininess = 10.0f;
bool blinnPhong = true;
float lightIntensity = 1.0f;
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
// Ray
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
    material(int type, vec4 albedo, vec2 ref_idx):type(type), albedo(albedo), ref_idx(ref_idx){}
};

////////////////////////////////////////////////////
// Hittable
struct aabb{
    vec3 vmin;
    vec3 vmax;
    aabb():vmin(vec3(0.0f)), vmax(vec3(0.0f)){}
    aabb(vec3 vmin, vec3 vmax):vmin(vmin), vmax(vmax){}
};

#define MAX_FLOAT	999999999
bool hit_aabb(aabb box, Ray r, float t_min, float t_max){
    vec3 vmino = (box.vmin - r.origin);
    vec3 vmaxo = (box.vmax - r.origin);
    vec3 vminod = vmino/r.direction;
    vec3 vmaxod = vmaxo/r.direction;

    for (int i = 0; i < 3; ++i) {
        if(r.direction[i] == 0.0f){
            vminod[i] = float(vmino[i] > 0 ? 1 : -1) * float(MAX_FLOAT);
            vmaxod[i] = float(vmaxo[i] > 0 ? 1 : -1) * float(MAX_FLOAT);
        }
    }
    vec3 t0 = min(vminod, vmaxod);
    vec3 t1 = max(vminod, vmaxod);

    for(int i = 0; i < 3; ++i){
        t_min = max(t0[i], t_min);
        t_max = min(t1[i], t_max);
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
    aabb box = aabb(s.center - vec3(s.radius), s.center + vec3(s.radius));
//    aabb box = aabb(center + vec3(s.radius), s.center - vec3(s.radius));
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

bool hit_sphere_with_aabb(sphere s, Ray r, float t_min, float t_max, hit_record &rec){
    if(!hit_aabb(s.box, r, t_min, t_max)){
        return false;
    }
    return hit_sphere(s, r, t_min, t_max, rec);
}

struct plane{
    vec3 center;
    vec3 normal;
    material m;
    plane():center(vec3(0.0)), normal(vec3(0.0)){}
    plane(vec3 center, vec3 normal, material m):center(center), normal(normal), m(m){}
};
bool check_plane_hit_rec(plane p, Ray r, float temp, float t_min, float t_max, hit_record &rec){
    if(temp < t_max && temp > t_min){
        rec.t = temp;
        rec.p = point_at_parameter(r, rec.t);
        rec.normal = p.normal;
        rec.m = p.m;
        return true;
    }
    return false;
}
#define EPSILON 0.0001f
bool hit_plane(plane p, Ray r, float t_min, float t_max, hit_record &rec){
    float denom = dot(p.normal, r.direction);
    if (abs(denom) > EPSILON){
        float temp = dot(p.center - r.origin, p.normal) / denom;
        if (temp >= 0){
            hit_record hr;
            if(check_plane_hit_rec(p, r, temp, t_min, t_max, hr)){
                rec = hr;
                return true;
            }
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
        if(hit_sphere_with_aabb(s_list.s[i], r, t_min, closest_so_far, temp_rec)){
            hit_anything = true;
            closest_so_far = temp_rec.t;
            rec = temp_rec;
        }
    }
    return hit_anything;
}

bool hit_plane_sphere_list(plane p, sphere_list s_list, Ray r, float t_min, float t_max, hit_record &rec){
    hit_record temp_rec;
    bool hit_anything = false;
    float closest_so_far = t_max;
    if(hit_sphere_list(s_list, r, t_min, closest_so_far, temp_rec)){
        hit_anything = true;
        closest_so_far = temp_rec.t;
        rec = temp_rec;
    }
    if(hit_plane(p, r, t_min, closest_so_far, temp_rec)){
        hit_anything = true;
        rec = temp_rec;
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
    int rayType;
    hit_record rec;
    int depth;
    stackFrame(): r(Ray()), I(vec4(0.0)), rayType(-1), rec(), depth(0){}
    stackFrame(Ray r, vec4 I, int rayType, hit_record rec, int depth): r(r), I(I), rayType(rayType), rec(rec), depth(depth){}
    stackFrame(Ray r, vec4 I, int depth): r(r), I(I), rayType(-1), rec(), depth(depth){}
};

#define MAX_BOUNCE 4
#define STACK_SIZ (MAX_BOUNCE * 10)
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

bool scatter_matte(Ray r_in, hit_record rec, vec4 &col, Ray &scattered){
    vec3 target = rec.p + rec.normal + random_in_unit_sphere(); // p + N is the new point(center of imaginary sphere) in direction of random point in unit sphere
    scattered = Ray(rec.p, target - rec.p);
    col = rec.m.albedo;
    return true;
}

bool scatter_metal(Ray r_in, hit_record rec, vec4 &col, Ray &scattered){
    vec3 norm_rin_dir = normalize(r_in.direction);
    vec3 reflected = reflect(norm_rin_dir, rec.normal);
    scattered = Ray(rec.p, reflected);
    col = rec.m.albedo;
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

bool scatter_dielec(Ray r_in, hit_record rec, vec4 &col, Ray &r_ref, Ray &r_trans){
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
        col = rec.m.albedo * vec4(vec3(f), 1.0);
        r_ref = Ray(rec.p, reflected);
        r_trans = Ray(rec.p, refracted);
    } else {
        col = rec.m.albedo * vec4(1.0);
        r_ref = Ray(rec.p, reflected);
        r_trans = Ray(vec3(0.0), vec3(0.0));
    }
    return true;
}

////////////////////////////////////////////////////
// Direct Illumination
float gaussianDist(float theta, float alpha){
    float g_theta = 2.0f / float( 1 + sqrt(1 + (pow(alpha, 2) * pow(tan(theta), 2))));
    return g_theta;
}

float shadowMasking(float theta_i, float theta_o, float alpha){
    float g_i = gaussianDist(theta_i, alpha);
    float g_o = gaussianDist(theta_o, alpha);
    float g = g_i * g_o;
    return g;
}

float microFacetNormDist(float theta, float alpha){
    if(theta < 0){
        return 0;
    }
    if(theta > radians(90.0f)){
        return 0;
    }
    float alpha_sq = pow(alpha, 2.0f);
    float d_theta = alpha_sq / float(radians(90.0f) * pow(cos(theta), 4) * pow(alpha_sq + pow(tan(theta), 2.0f), 2.0f));
    return d_theta;
}

float fresnelDielecOpaque(float etaReal, float angle){
    float c_i = sqrt(pow(etaReal, 2.0f) - pow(sin(angle), 2.0f));

    float cos_angle = cos(angle);

    float f_s = pow(abs((cos_angle - c_i)/(cos_angle + c_i)), 2.0f);

    float temp = pow(etaReal, 2.0f) * cos_angle;
    float f_p = pow(abs((temp - c_i)/(temp + c_i)), 2.0f);

    float f = (f_s + f_p)/2;
    return f;
}

float fresnelDielecTransComp(vec3 incidence, vec3 outward_normal, vec3 transmitted, float ni, float nt){
    float cos_i = dot(incidence, outward_normal)/(length(incidence) * length(outward_normal));
    float cos_t = dot(transmitted, outward_normal)/(length(transmitted) * length(outward_normal));
    float nt_cos_i = nt * cos_i;
    float ni_cos_t = ni * cos_t;
    float r_par = (nt_cos_i - ni_cos_t) / (nt_cos_i + ni_cos_t);
    float ni_cos_i = ni * cos_i;
    float nt_cos_t = nt * cos_t;
    float r_per = (ni_cos_i - nt_cos_t) / (ni_cos_i + nt_cos_t);
    float f = float(pow(r_par, 2) + pow(r_per, 2)) / 2.0f;
    return f;
}

float fresnelDielecTrans(vec3 l, vec3 n, float etaReal){
    vec3 reflected = reflect(l, n);
    vec3 refracted;
    float ni;
    float nt;
    vec3 outward_normal;
    if(dot(l, n) > 0){
        outward_normal = -n;
        ni = etaReal;
        nt = 1.0f;
    } else {
        outward_normal = n;
        ni = 1.0f;
        nt = etaReal;
    }
    float f;
    if(refract(l, outward_normal, ni/nt, refracted)){
        f = fresnelDielecTransComp(l, outward_normal, refracted, ni, nt);
    } else {
        f = 1.0f;
    }
    return f;
}

float fresnelMetal(vec3 l, vec3 n, vec2 etaMat){
    float n_dot_l = max(dot(l, n), 0.0);
    float n_dot_l2 = pow(n_dot_l, 2.0f);
    float eta_nd = 2 * etaMat.x * n_dot_l;
    float eta_nk = dot(etaMat, etaMat);
    float denom_r_par_2 = (eta_nk * n_dot_l2) - eta_nd + 1;
    float denom_r_per_2 = eta_nk + n_dot_l2 + eta_nd;
    if(denom_r_par_2 == 0.0 || denom_r_per_2 == 0.0){
        return 1.0f;
    }
    float r_par_2 = ((eta_nk * n_dot_l2) - eta_nd + 1)/denom_r_par_2;
    float r_per_2 = (eta_nk + n_dot_l2 - eta_nd)/denom_r_per_2;
    float f = (r_par_2 + r_per_2)/2.0f;
    return f;
}

vec4 getSpecularColor(vec4 baseCol, float I, vec2 etaMat, float shineExpo, bool modelType, int material, vec3 vertNormal, vec3 eyeVector, vec3 lightDir){
    vec3 n = normalize(vertNormal);
    vec3 v = normalize(eyeVector);
    vec3 l = normalize(-lightDir);
    vec3 h = normalize(l + v);

    float theta_d = acos(dot(h, l));
    float f_theta_d = 0.0f;
    switch (material) {
        case DIELEC:
            f_theta_d = fresnelDielecOpaque(etaMat.x, theta_d);
            break;
        case MATTE:
            f_theta_d = fresnelDielecTrans(-l, n, etaMat.x);
            break;
        case METAL:
            f_theta_d = fresnelMetal(l, n, etaMat);
            break;
        default:
            break;
    }

    vec4 specCol = vec4(vec3(0.0), 1.0);
    if(dot(n, l) >= 0.0){
        float shine = 0.0;
        float model = 0.0;
        if(modelType){
            shine = max(shineExpo, 1.0f);
            model = max(dot(n, h), 0.0f);
        } else {
            shine = max(round(shineExpo / 10.0f), 1.0f);

            float theta_i = acos(dot(n, v));
            float theta_o = acos(dot(n, l));
            float theta_h = acos(dot(n, h));

            vec3 r = normalize(reflect(-l, n));
            float alpha = acos(dot(v, r));

            float g_io = shadowMasking(theta_i, theta_o, alpha);
            float d_theta_h = microFacetNormDist(theta_h, alpha);

            model = (d_theta_h * g_io) / (4 * cos(theta_i) * cos(theta_o));
        }
        specCol = f_theta_d * baseCol * pow(model, shine);
    }
    return specCol;
}

vec4 getDiffuseColor(vec4 col, float I, vec3 vertNormal, vec3 lightDir){
    vec3 n = normalize(vertNormal);
    vec3 l = normalize(-lightDir);
    vec4 diffCol = 0.6f * col * max(dot(n, l), 0.0f) * I;
    return diffCol;
}

vec4 getAmbientColor(vec4 col, float I){
    vec4 ambCol = 0.2f * col * I;
    return ambCol;
}

vec4 getDirectIllumination(vec4 baseCol, float I, vec2 etaMat, float shineExpo, bool modelType, int material, vec3 vertNormal, vec3 eyeVector, vec3 lightDir){
    vec4 ambCol = getAmbientColor(baseCol, I);
    vec4 diffCol = getDiffuseColor(baseCol, I, vertNormal, lightDir);
    vec4 specCol = getSpecularColor(baseCol, I, etaMat, shineExpo, modelType, material, vertNormal, eyeVector, lightDir);
    vec4 dICol = ambCol + diffCol + specCol;
    return dICol;
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

vec4 color_shadow_ray(Ray r, sphere_list s, hit_record rec){
    vec4 col = vec4(vec3(0.0), 1.0);
    hit_record shadow_rec;
    Ray shadow = Ray(rec.p, normalize(lightPosition - rec.p));
    if(!hit_sphere_list(s, shadow, 0.001f, float(MAX_FLOAT), shadow_rec)){
        col = getDirectIllumination(rec.m.albedo, lightIntensity, vec2(eta, 0.0f), shininess, blinnPhong, rec.m.type, rec.normal, vec3(0.0f) - rec.p, rec.p - lightPosition);
    } else {
        col = getAmbientColor(rec.m.albedo, lightIntensity);
    }
    return col;
}

vec4 color_global(Ray r, sphere_list s, int depth){
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
                    col += vec4(vec3(1.0f) - vec3(attenuation), 1.0f) * color_global(transmitted, s, depth);
                }
                col += attenuation * color_global(scattered, s, depth);
            }
        }

        col /= vec4(float(depth) + 1.0f);
        col += color_shadow_ray(r, s, rec);
        col = vec4(vec3(min(col, 1.0f)), 1.0f);
    } else {
        vec3 unit_direction = normalize(r.direction);
        col = getColorFromEnvironment(unit_direction);
    }
    return col;
}

bool scatter(Ray r, hit_record rec, vec4 &col, Ray &scattered, Ray &transmitted){
    bool res;
    switch (rec.m.type) {
        case MATTE:
            res = scatter_matte(r, rec, col, scattered);
            break;
        case METAL:
            res = scatter_metal(r, rec, col, scattered);
            break;
        case DIELEC:
            res = scatter_dielec(r, rec, col, scattered, transmitted);
            break;
    }
    return res;
}

#include<stack>
stack<stackFrame> fS;
#define RAY_HIT_REF 0
#define RAY_HIT_TRANS 1
#define RAY_MISS 2
vec4 color_nonrecursive_global(Ray r, plane p, sphere_list s){
    vec4 col = vec4(0.0, 0.0, 0.0, 1.0);

    stackFrame d = stackFrame(r, vec4(1.0f), -1);
    push(d);
    while (!isStackEmpty()){
        d = pop();
        bool hit = hit_plane_sphere_list(p, s, d.r, 0.001, float(MAX_FLOAT), d.rec);
        if (hit && d.depth < MAX_BOUNCE){
            d.depth++;
            Ray scattered;
            Ray transmitted;
            vec4 temp;
            bool res = scatter(r, d.rec, temp, scattered, transmitted);
            if(res){
                d.rayType = RAY_HIT_REF;
                d.r = scattered;
                push(d);
                fS.push(d);
                if(transmitted.origin != vec3(0.0)){
                    d.rayType = RAY_HIT_TRANS;
                    d.r = transmitted;
                    push(d);
                    fS.push(d);
                }
            }
        } else {
            if(!hit){
                if(d.depth < 0){
                    return getColorFromEnvironment(d.r.direction);
                }
                d.rayType = RAY_MISS;
                fS.push(d);
            }
        }
    }
    col = vec4(1.0);
    while(!fS.empty()){
        d = fS.top();
        fS.pop();
        vec4 dCol = color_shadow_ray(d.r, s, d.rec);
        vec4 iCol;
        Ray t1;
        Ray t2;
        switch(d.rayType){
            case RAY_MISS:
                iCol = getColorFromEnvironment(d.r.direction);
                continue;
                break;
            case RAY_HIT_REF:
                scatter(d.r, d.rec, iCol, t1, t2);
                break;
            case RAY_HIT_TRANS:
                scatter(d.r, d.rec, iCol, t1, t2);
                iCol = d.rec.m.albedo - iCol;
                break;
        }
        col = col * iCol + dCol;
        col /= float(d.depth+1);
    }
    col = min(col, vec4(1.0f));
    return col;
}


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
                        d.I = I * vec4(vec3(rec.m.albedo) - vec3(attenuation), 1.0f);
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
    s_list.list_size = 3;
    s_list.s[0] = sphere(center, radius, material(MATTE, vec4(0.1f, 0.2f, 0.5f, 1.0f)));
//    s_list.s[1] = sphere(vec3(0.0f, -100.5f, -1.0f), 100.0f, material(MATTE, vec4(0.8f, 0.8f, 0.0f, 1.0f)));
    s_list.s[1] = sphere(vec3(1.0f, 0.0f, -1.0f), 0.5f, material(METAL, vec4(0.8f, 0.6f, 0.2f, 1.0f)));
    s_list.s[2] = sphere(vec3(-1.0f, 0.0f, -1.0f), 0.5f, material(DIELEC, vec4(1.0f), vec2(1.5f, 0.0f))); // METAL, vec4(0.8f, 0.8f, 0.8f, 1.0f));

    s_list = setup_aabb_sphere_list(s_list);

    plane p = plane(vec3(0.0, -0.5, -1.0), vec3(0.0, 1.0, 0.0), material(MATTE, vec4(0.8f, 0.8f, 0.0f, 1.0f)));

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
//                col += color_nonrecursive(r, s_list);
//                col += color_global(r, s_list, -1);
                col += color_nonrecursive_global(r, p, s_list);
                flushStack();
                while(!fS.empty()){
                    fS.pop();
                }
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