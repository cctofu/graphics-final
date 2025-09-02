#ifndef __MATERIAL_H__
#define __MATERIAL_H__

#include "utils.hpp"
#include "camera.hpp"
#include "texture.hpp"

class Material;
struct Hit {
    Vec3 p;
    Vec3 norm;
    double t;
    Material *material;
    double u = 0.0, v = 0.0;
    bool front_face;

    inline void set_face_normal(const Ray& r, const Vec3& outward_normal) {
        front_face = r.direction().dot(outward_normal) < 0;
        norm = front_face ? outward_normal : Vec3(0,0,0) - outward_normal;
    }
};


class Material {
public:
    virtual bool scatter(const Ray &ray, const Hit &hit, Vec3 &attenuation, Ray &scattered) const = 0;
    virtual Vec3 illuminate(double u, double v, const Vec3& p) const {
        return Vec3(0,0,0);
    }
};

class Diffuse : public Material {
public:
    Diffuse(Texture* a) : albedo(a) {}
    virtual bool scatter(const Ray &ray, const Hit &hit, Vec3 &attenuation, Ray &scattered) const {
        Vec3 target = hit.p + hit.norm + random_in_unit_sphere();
        scattered = Ray(hit.p, (target - hit.p).normalized());
        attenuation = albedo->value(hit.u, hit.v, hit.p);
        return true;
    }

    Texture* albedo;
};

class Specular : public Material
{
    Texture* albedo;
    double fuzz;

public:
    Specular(Texture* a, double f) : albedo(a), fuzz(clamp(f)) {}
    virtual bool scatter(const Ray &ray, const Hit &hit, Vec3 &attenuation, Ray &scattered) const
    {
        Vec3 reflected = ray.direction().reflect(hit.norm);
        scattered = Ray(hit.p, reflected + random_in_unit_sphere() * fuzz);
        attenuation = albedo->value(hit.u, hit.v, hit.p);
        return (scattered.direction().dot(hit.norm) > 0);
    }
};

class Refract : public Material
{
    bool refract(const Vec3 &v, const Vec3 &n, double n_relative, Vec3 &refr) const
    {
        Vec3 vv = v.normalized();
        double cosv = v.dot(n);
        double det = 1 - n_relative * n_relative * (1 - cosv * cosv);
        if (det > 0)
        {
            refr = (v - n * cosv) * n_relative + n * sqrt(det);
            return true;
        }
        else
            return false;
    }
    double schlick (double cosine, double ref) const
    {
        double r0 = (1 - ref) / (1 + ref);
        r0 *= r0;
        return r0 + (1 - r0) * pow(1 - cosine, 5);
    }
    Vec3 color;

public:
    Refract(double ref, Vec3 c = Vec3(1,1,1)) : ri(ref), color(c) {}
    virtual bool scatter(const Ray &ray, const Hit &hit, Vec3 &attenuation, Ray &scattered) const {
        Vec3 out_norm;
        Vec3 reflected = ray.direction().reflect(hit.norm);
        double n_relative;
        attenuation = color;
        Vec3 refracted;
        double cosine;
        if (hit.norm.dot(ray.direction()) > 0)
        {
            out_norm = hit.norm;
            n_relative = ri;
            cosine = ri * ray.direction().dot(hit.norm);
        }
        else
        {
            out_norm = Vec3() - hit.norm;
            n_relative = 1 / ri;
            cosine = -ri * ray.direction().dot(hit.norm);
        }

        if (refract(ray.direction(), out_norm, n_relative, refracted))
        {
            double reflect_prob = schlick(cosine, ri);
            if (reflect_prob <= drand48())
                scattered = Ray(hit.p, refracted);
            else
                scattered = Ray(hit.p, reflected);
        }
        else
        {
            scattered = Ray(hit.p, reflected);
        }
        return true;
    }

    double ri;
};

Vec3 reflect(const Vec3& v, const Vec3& n) {
    return v - 2 * v.dot(n) * n;
}

bool refract(const Vec3& v, const Vec3& n, double ni_over_nt, Vec3& refracted) {
    Vec3 uv = v.normalized();
    double dt = uv.dot(n);
    double discriminant = 1.0 - ni_over_nt * ni_over_nt * (1 - dt * dt);
    if (discriminant > 0) {
        refracted = ni_over_nt * (uv - n * dt) - n * sqrt(discriminant);
        return true;
    }
    return false;
}

Vec3 random_unit_vector() {
    double a = random_double(0, 2 * PI);
    double z = random_double(-1, 1);  
    double r = sqrt(1 - z*z);        
    return Vec3(r*cos(a), r*sin(a), z);
}

class DiffuseLight: public Material{
    Texture* emit;
public:
    DiffuseLight(Texture *a): emit(a) {}
    virtual bool scatter(const Ray &ray, const Hit &hit, Vec3 &attenuation, Ray &scattered) const override {
        return false;
    }
    virtual Vec3 illuminate(double u, double v, const Vec3& p) const {
        return emit->value(u, v, p);
    }
};

class Metal : public Material {
public:
    Metal(const Vec3& a) : albedo(a) {}
    
    virtual bool scatter(const Ray& ray, const Hit& hit, Vec3& attenuation, Ray& scattered) const override {
        Vec3 reflected = reflect(ray.direction().normalized(), hit.norm);
        scattered = Ray(hit.p, reflected);
        attenuation = albedo;
        return (scattered.direction().dot(hit.norm) > 0);
    }

    Vec3 albedo;
};

class Lambertian : public Material {
public:
    Lambertian(const Vec3& a) : albedo(a) {}

    virtual bool scatter(const Ray& ray, const Hit& hit, Vec3& attenuation, Ray& scattered) const override {
        Vec3 scatter_direction = hit.norm + random_unit_vector();
        scattered = Ray(hit.p, scatter_direction);
        attenuation = albedo;
        return true;
    }

    Vec3 albedo;
};

class Dielectric : public Material {
public:
    Dielectric(double index_of_refraction) : ir(index_of_refraction) {}

    virtual bool scatter(const Ray& ray, const Hit& hit, Vec3& attenuation, Ray& scattered) const override {
        attenuation = Vec3(1.0, 1.0, 1.0);
        double refraction_ratio = hit.front_face ? (1.0 / ir) : ir;

        Vec3 unit_direction = ray.direction().normalized();
        double cos_theta = fmin(-unit_direction.dot(hit.norm), 1.0);
        double sin_theta = sqrt(1.0 - cos_theta * cos_theta);

        bool cannot_refract = refraction_ratio * sin_theta > 1.0;
        Vec3 direction;

        Vec3 refracted;
        if (refract(unit_direction, hit.norm, refraction_ratio, refracted)) {
            scattered = Ray(hit.p, refracted);
        }

        scattered = Ray(hit.p, direction);
        return true;
    }

private:
    static double reflectance(double cosine, double ref_idx) {
        auto r0 = (1 - ref_idx) / (1 + ref_idx);
        r0 = r0 * r0;
        return r0 + (1 - r0) * pow(1 - cosine, 5);
    }

    double ir; // Index of Refraction
};

#endif