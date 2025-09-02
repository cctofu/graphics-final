#ifndef __CONSTANT_MEDIUM_H__
#define __CONSTANT_MEDIUM_H__

#include "material.hpp"
#include "object.hpp"
#include "texture.hpp"
#include <limits>

const double infi = std::numeric_limits<double>::infinity();

class ConstantMedium : public Object  {
    public:
        ConstantMedium(Object* b, double d, Texture* a)
            : boundary(b),
              neg_inv_density(-1/d)
            {}

        ConstantMedium(Object* b, double d, Vec3 color)
            : boundary(b),
              neg_inv_density(-1/d)
            {}

        virtual bool intersect(const Ray& r, double t_min, double t_max, Hit& rec) const override{
            Hit rec1, rec2;

            if (!boundary->intersect(r, -infi, infi, rec1))
                return false;

            if (!boundary->intersect(r, rec1.t+0.0001, infi, rec2))
                return false;

            if (rec1.t < t_min) rec1.t = t_min;
            if (rec2.t > t_max) rec2.t = t_max;

            if (rec1.t >= rec2.t)
                return false;

            if (rec1.t < 0)
                rec1.t = 0;

            const auto ray_length = r.direction().len();
            const auto distance_inside_boundary = (rec2.t - rec1.t) * ray_length;
            const auto hit_distance = neg_inv_density * log(drand48());

            if (hit_distance > distance_inside_boundary)
                return false;

            rec.t = rec1.t + hit_distance / ray_length;
            rec.p = r.point(rec.t);

            rec.norm = Vec3(1,0,0);  // arbitrary
            rec.material = phaseFunc;

            return true;
        }

        virtual bool bounding_box(double time0, double time1, AABB& output_box) const override {
            return boundary->bounding_box(time0, time1, output_box);
        }

        virtual AABB get_aabb() const override {
            return boundary->get_aabb();
        }

        virtual int longest_axis() const override {
            return boundary->longest_axis();
        }

    public:
        Object* boundary;
        Material* phaseFunc;
        double neg_inv_density;
};


#endif