#ifndef __SHAPE_H__
#define __SHAPE_H__

#include "utils.hpp"
#include "camera.hpp"
#include "material.hpp"
#include "aabb.hpp"
#include <vector>

const double MAX_double = 10000000000.0;

double det(const Vec3& a, const Vec3& b, const Vec3& c) {
    double res = 0;
    res += a.x * b.y * c.z + a.y * b.z * c.x + a.z * b.x * c.y;
    res -= a.x * b.z * c.y + a.y * b.x * c.z + a.z * b.y * c.x;
    return res;
}

inline double degrees_to_radians(double x){
    return acos(-1) * x / 180.0f;
}

class Object {
public:
    Material *material;
    virtual bool intersect(const Ray &r, double t_min, double t_max, Hit &hit) const = 0;
    virtual bool bounding_box(double t0, double t1, AABB& box) const = 0;
    virtual AABB get_aabb() const = 0;
    virtual int longest_axis() const = 0;
};


class Group : public Object {
public:
    std::vector<Object *> list;
    Group() { list.clear(); }
    void add(Object *obj) {
        list.push_back(obj);
    }
    int size() {
        return list.size();
    }
    Object* & operator[] (int i) {return list[i];}
    std::vector<Object *> getList() {return list;}
    virtual bool intersect(const Ray &r, double t_min, double t_max, Hit &hit) const override {
        bool if_hit = false;
        Hit tmp_hit;
        double tmp_t = t_max;
        for (int i = 0; i < list.size(); ++i) {
            if (list[i]->intersect(r, t_min, tmp_t, tmp_hit)) {
                if_hit = true;
                tmp_t = tmp_hit.t;
                hit = tmp_hit;
            }
        }
        return if_hit;
    }
    virtual bool bounding_box(double t0, double t1, AABB& box) const override {
        if (list.empty()) 
            return false;
        AABB tmp_box;
        bool first_box = true;
        for (const auto& object : list) {
            if (!object->bounding_box(t0, t1, tmp_box)) return false;
            box = first_box ? tmp_box : combine_box(box, tmp_box);
            first_box = false;
        }
        return true;
    }
    
    virtual AABB get_aabb() const override {
        AABB aabb;
        bool first_box = true;
        for (const auto& object : list) {
            if(first_box){
                aabb = object->get_aabb();
                first_box = false;
            } else {
                aabb = combine_box(aabb, object->get_aabb());
            }
        }
        return aabb;
    }

    virtual int longest_axis() const override {
        AABB aabb = get_aabb();
        Vec3 diagonal = aabb.max() - aabb.min();
        if (diagonal.x > diagonal.y && diagonal.x > diagonal.z)
            return 0; // x-axis is longest
        else if (diagonal.y > diagonal.z)
            return 1; // y-axis is longest
        else
            return 2; // z-axis is longest
    }
};

class Sphere : public Object
{
    void get_UV(const Vec3& p, double& u, double& v) const{
        double phi = atan2(p.z, p.x);
        double theta = asin(p.y);
        u = 0.5 - phi / (2 * PI);
        v = 0.5 + theta / PI;
    }
public:
    double radius;
    Vec3 center;
    Sphere(double rad_, Vec3 cent, Material *mat) : radius(rad_), center(cent) {
        material = mat;
    }
    virtual bool intersect(const Ray &r, double t_min, double t_max, Hit &hit) const override {
        Vec3 op = center - r.o;
        double t, b = op.dot(r.direction()), det = b * b - op.len2() + radius * radius;
        if (det > 0) {
            double tmp = (b - sqrt(det));
            if (tmp > t_min && tmp < t_max) {
                hit.t = tmp;
                hit.p = r.point(tmp);
                hit.norm = (hit.p - center) / radius;
                hit.norm.normalize();
                hit.material = this->material;
                get_UV(hit.norm, hit.u, hit.v);
                return true;
            }
            tmp = (b + sqrt(det));
            if (tmp > t_min && tmp < t_max) {
                hit.t = tmp;
                hit.p = r.point(tmp);
                hit.norm = (hit.p - center) / radius;
                hit.norm.normalize();
                hit.material = this->material;
                get_UV(hit.norm, hit.u, hit.v);
                return true;
            }
        }
        return false;
    }

    virtual bool bounding_box(double t0, double t1, AABB& box) const override {
        box = AABB(center - Vec3(radius, radius, radius), 
                   center + Vec3(radius, radius, radius));
        return true;
    }
    
    virtual AABB get_aabb() const override {
        return AABB(center - Vec3(radius, radius, radius), center + Vec3(radius, radius, radius));
    }

    virtual int longest_axis() const override {
        return 0;
    }
};

class Triangle: public Object {

public:
	Triangle() = delete;
	Triangle( const Vec3& a, const Vec3& b, const Vec3& c, Material* m) {
        material = m;
		vertices[0] = a;
		vertices[1] = b;
		vertices[2] = c;
		this->norm = (b - a) % (c - a);
	    this->norm.normalize();
	}
	virtual bool intersect( const Ray& ray, double t_min, double t_max, Hit& hit) const override {
        double t = 0.0, b = 0.0, r = 0.0;
		Vec3 e1 = vertices[0] - vertices[1], e2 = vertices[0] - vertices[2], s = vertices[0] - ray.origin();
		
        double d0 = det(ray.direction(), e1, e2);
        double d1 = det(s, e1, e2);
        double d2 = det(ray.direction(), s, e2);
        double d3 = det(ray.direction(), e1, s);
		if (d0 == 0.0)
			return false;
		t = d1 / d0;
		b = d2 / d0;
		r = d3 / d0;
		if (t > t_min && t < t_max) {
			if (b >= 0 && b <= 1 && r >= 0 && r <= 1 && b + r <= 1) {
				hit.t = t;
                hit.norm = this->norm;
                hit.p = ray.point(t);
                hit.material = this->material;
                Vec3 calc = vertices[0] - hit.p;
				return true;
			}
		}
		return false;
	}
    Vec3 normal() { 
        return this->norm; 
    }
    virtual bool bounding_box(double t0, double t1, AABB& box) const override {
        Vec3 vert[3];
        for (int i = 0; i < 3; ++i)
            vert[i] = vertices[i];
        Vec3 minV = vert[0], maxV = vert[0];
        for (int i = 1; i < 3; ++i) {
            for (int r = 0; r < 3; r++) {
                minV[r] = fmin(minV[r], vert[i][r]) - 0.00001;
                maxV[r] = fmax(maxV[r], vert[i][r]) + 0.00001;
            }
        }
        box = AABB(minV, maxV);
        return true;
    }

    virtual AABB get_aabb() const override {
        Vec3 vert[3];
        for (int i = 0; i < 3; ++i)
            vert[i] = vertices[i];
        Vec3 minV = vert[0], maxV = vert[0];
        for (int i = 1; i < 3; ++i) {
            for (int r = 0; r < 3; r++) {
                minV[r] = fmin(minV[r], vert[i][r]) - 0.00001;
                maxV[r] = fmax(maxV[r], vert[i][r]) + 0.00001;
            }
        }
        return AABB(minV, maxV);
    }

    virtual int longest_axis() const override {
        AABB aabb = get_aabb();
        Vec3 diagonal = aabb.max() - aabb.min();
        if (diagonal.x > diagonal.y && diagonal.x > diagonal.z)
            return 0; // x-axis is longest
        else if (diagonal.y > diagonal.z)
            return 1; // y-axis is longest
        else
            return 2; // z-axis is longest
    }

private:
	Vec3 norm;
	Vec3 vertices[3];

};

class MovSphere: public Object{
    Vec3 center0, center1;
    double time0, time1;
    double radius;
    Vec3 get_center(double t) const {
        return center0 + (center1 - center0) * (t - time0) / (time1 - time0);
    }

public:
    MovSphere() = delete;
    MovSphere(Vec3 cen0, Vec3 cen1, double t0, double t1, double r, Material* m):
        center0(cen0), center1(cen1), time0(t0), time1(t1), radius(r) {
            material = m;
    }
    
    virtual bool intersect(const Ray &r, double t_min, double t_max, Hit &hit) const override {
        Vec3 center = get_center(r.time);
        Vec3 op = center - r.o;
        double t, b = op.dot(r.direction()), det = b * b - op.len2() + radius * radius;
        if (det > 0) {
            double tmp = (b - sqrt(det));
            if (tmp > t_min && tmp < t_max) {
                hit.t = tmp;
                hit.p = r.point(tmp);
                hit.norm = (hit.p - center) / radius;
                hit.norm.normalize();
                hit.material = this->material;
                return true;
            }
            tmp = (b + sqrt(det));
            if (tmp > t_min && tmp < t_max) {
                hit.t = tmp;
                hit.p = r.point(tmp);
                hit.norm = (hit.p - center) / radius;
                hit.norm.normalize();
                hit.material = this->material;
                return true;
            }
        }
        return false;
    }
    
    virtual bool bounding_box(double t0, double t1, AABB& box) const override {
        AABB box1(get_center(t0) - Vec3(radius, radius, radius), 
                   get_center(t0) + Vec3(radius, radius, radius));
        AABB box2(get_center(t1) - Vec3(radius, radius, radius), 
                   get_center(t1) + Vec3(radius, radius, radius));
        box = combine_box(box1, box2);
        return true;
    }

    virtual AABB get_aabb() const override {
        AABB box1(get_center(time0) - Vec3(radius, radius, radius), 
                   get_center(time0) + Vec3(radius, radius, radius));
        AABB box2(get_center(time1) - Vec3(radius, radius, radius), 
                   get_center(time1) + Vec3(radius, radius, radius));
        return combine_box(box1, box2);
    }

    virtual int longest_axis() const override {
        AABB aabb = get_aabb();
        Vec3 diagonal = aabb.max() - aabb.min();
        if (diagonal.x > diagonal.y && diagonal.x > diagonal.z)
            return 0; // x-axis is longest
        else if (diagonal.y > diagonal.z)
            return 1; // y-axis is longest
        else
            return 2; // z-axis is longest
    }
};

class XY_RECT : public Object {
public: 
    XY_RECT() {}
    XY_RECT(double _x0, double _x1, double _y0, double _y1, double _k, Material* mat)
        : x0(_x0), x1(_x1), y0(_y0), y1(_y1), k(_k) {
        material = mat;
    };

    virtual bool intersect(const Ray& r, double t_min, double t_max, Hit& rec) const override {
        auto t = (k-r.origin().z) / r.direction().z;
        if (t < t_min || t > t_max)
            return false;
        auto x = r.origin().x + t*r.direction().x;
        auto y = r.origin().y + t*r.direction().y;
        if (x < x0 || x > x1 || y < y0 || y > y1)
            return false;
        rec.u = (x-x0)/(x1-x0);
        rec.v = (y-y0)/(y1-y0);
        rec.t = t;
        auto outward_normal = Vec3(0, 0, 1);
        rec.set_face_normal(r, outward_normal);
        rec.material = material;
        rec.p = r.point(t);
        return true;
    }

    virtual bool bounding_box(double time0, double time1, AABB& output_box) const override {
        output_box = AABB(Vec3(x0,y0, k-0.0001), Vec3(x1, y1, k+0.0001));
        return true;
    }
    
    virtual AABB get_aabb() const override {
    return AABB(Vec3(x0,y0, k-0.0001), Vec3(x1, y1, k+0.0001));
    }

    virtual int longest_axis() const override {
        AABB aabb = get_aabb();
        Vec3 diagonal = aabb.max() - aabb.min();
        if (diagonal.x > diagonal.y && diagonal.x > diagonal.z)
            return 0; // x-axis is longest
        else if (diagonal.y > diagonal.z)
            return 1; // y-axis is longest
        else
            return 2; // z-axis is longest
    }

public:
    double x0, x1, y0, y1, k;
};

class YZ_RECT : public Object {
public: 
    YZ_RECT() {}
    YZ_RECT(double _y0, double _y1, double _z0, double _z1, double _k, Material* mat)
        : y0(_y0), y1(_y1), z0(_z0), z1(_z1), k(_k) {
        material = mat;
    };

    virtual bool intersect(const Ray& r, double t_min, double t_max, Hit& rec) const override {
        auto t = (k-r.origin().x) / r.direction().x;
        if (t < t_min || t > t_max)
            return false;
        auto y = r.origin().y + t*r.direction().y;
        auto z = r.origin().z + t*r.direction().z;
        if (y < y0 || y > y1 || z < z0 || z > z1)
            return false;
        rec.u = (y-y0)/(y1-y0);
        rec.v = (z-z0)/(z1-z0);
        rec.t = t;
        auto outward_normal = Vec3(1, 0, 0);
        rec.set_face_normal(r, outward_normal);
        rec.material = material;
        rec.p = r.point(t);
        return true;
    }

    virtual bool bounding_box(double time0, double time1, AABB& output_box) const override {
        output_box = AABB(Vec3(k-0.0001, y0, z0), Vec3(k+0.0001, y1, z1));
        return true;
    }

    virtual AABB get_aabb() const override {
        return AABB(Vec3(k-0.0001, y0, z0), Vec3(k+0.0001, y1, z1));
    }

    virtual int longest_axis() const override {
        AABB aabb = get_aabb();
        Vec3 diagonal = aabb.max() - aabb.min();
        if (diagonal.x > diagonal.y && diagonal.x > diagonal.z)
            return 0; // x-axis is longest
        else if (diagonal.y > diagonal.z)
            return 1; // y-axis is longest
        else
            return 2; // z-axis is longest
    }

public:
    double y0, y1, z0, z1, k;
};

class XZ_RECT : public Object {
public: 
    XZ_RECT() {}
    XZ_RECT(double _x0, double _x1, double _z0, double _z1, double _k, Material* mat)
        : x0(_x0), x1(_x1), z0(_z0), z1(_z1), k(_k) {
        material = mat;
    };

    virtual bool intersect(const Ray& r, double t_min, double t_max, Hit& rec) const override {
        auto t = (k-r.origin().y) / r.direction().y;
        if (t < t_min || t > t_max)
            return false;
        auto x = r.origin().x + t*r.direction().x;
        auto z = r.origin().z + t*r.direction().z;
        if (x < x0 || x > x1 || z < z0 || z > z1)
            return false;
        rec.u = (x-x0)/(x1-x0);
        rec.v = (z-z0)/(z1-z0);
        rec.t = t;
        auto outward_normal = Vec3(0, 1, 0);
        rec.set_face_normal(r, outward_normal);
        rec.material = material;
        rec.p = r.point(t);
        return true;
    }

    virtual bool bounding_box(double time0, double time1, AABB& output_box) const override {
        output_box = AABB(Vec3(x0,k-0.0001,z0), Vec3(x1, k+0.0001, z1));
        return true;
    }

    virtual AABB get_aabb() const override {
        return AABB(Vec3(x0,k-0.0001,z0), Vec3(x1, k+0.0001, z1));
    }

    virtual int longest_axis() const override {
        AABB aabb = get_aabb();
        Vec3 diagonal = aabb.max() - aabb.min();
        if (diagonal.x > diagonal.y && diagonal.x > diagonal.z)
            return 0; // x-axis is longest
        else if (diagonal.y > diagonal.z)
            return 1; // y-axis is longest
        else
            return 2; // z-axis is longest
    }

public:
    double x0, x1, z0, z1, k;
};

class Rectangle: public Object {

public:
	Rectangle() = delete;
	Rectangle( const Vec3& a, const Vec3& b, const Vec3& c, Material* m) {
        material = m;
		vertices[0] = a;
		vertices[1] = b;
        Vec3 e1 = (a - b).normalized(), e2 = (c - b).normalized();
        Vec3 cc = c - (c - b) * e1.dot(e2);
		vertices[2] = cc;

        vertices[3] = a + cc - b;
        tr1 = new Triangle(a, b, cc, m);
        tr2 = new Triangle(a, vertices[3], cc, m);
		this->norm = (b - a) % (cc - a);
	    this->norm.normalize();
	}
	virtual bool intersect( const Ray& ray, double t_min, double t_max, Hit& hit) const override {
        bool res1 = tr1->intersect(ray, t_min, t_max, hit);
        t_max = res1 ? hit.t : t_max;
        bool res2 = tr2->intersect(ray, t_min, t_max, hit);
        double t = 0.0, b = 0.0, r = 0.0;
		res1 =  res1 || res2;
        if (res1) {
            get_UV(hit.p, hit.v, hit.u);
            hit.norm = this->norm;
        }
        return res1;
	}
    Vec3 normal() { return this->norm; }

    virtual bool bounding_box(double t0, double t1, AABB& box) const override {
        AABB b1, b2;
        tr1->bounding_box(t0, t1, b1);
        tr2->bounding_box(t0, t1, b2);
        box = combine_box(b1, b2);
        return true;
    }

    AABB get_aabb() const override {
        return combine_box(tr1->get_aabb(), tr2->get_aabb());
    }
    int longest_axis() const override {
        return this->get_aabb().longest_axis();
    }

private:
	Vec3 norm;
	Vec3 vertices[4];
    Triangle* tr1, *tr2;
    void get_UV(const Vec3& p, double& u, double& v) const {
        Vec3 e1 = (vertices[0] - vertices[1]).normalized();
        Vec3 e2 = (vertices[2] - vertices[1]).normalized();
        Vec3 tmp = p - vertices[1];
        double x1 = (vertices[0] - vertices[1]).len(), x2 = (vertices[2] - vertices[1]).len();
        u = tmp.dot(e1) / x1;
        v = tmp.dot(e2) / x2;
    }

};

class Box : public Object  {
    public:
        Box() {}
        Box(const Vec3& p0, const Vec3& p1, Material* mat) {
            box_min = p0;
            box_max = p1;
            sides.add(new XY_RECT(p0.x, p1.x, p0.y, p1.y, p1.z, mat));
            sides.add(new XY_RECT(p0.x, p1.x, p0.y, p1.y, p0.z, mat));

            sides.add(new XZ_RECT(p0.x, p1.x, p0.z, p1.z, p1.y, mat));
            sides.add(new XZ_RECT(p0.x, p1.x, p0.z, p1.z, p0.y, mat));

            sides.add(new YZ_RECT(p0.y, p1.y, p0.z, p1.z, p1.x, mat));
            sides.add(new YZ_RECT(p0.y, p1.y, p0.z, p1.z, p0.x, mat));
        }

        virtual bool intersect(const Ray& r, double t_min, double t_max, Hit& rec) const override {
            return sides.intersect(r, t_min, t_max, rec);
        }

        virtual bool bounding_box(double time0, double time1, AABB& output_box) const override {
            output_box = AABB(box_min, box_max);
            return true;
        }

        AABB get_aabb() const override {
            return AABB(box_min, box_max);
        }
        int longest_axis() const override {
            return this->get_aabb().longest_axis();
        }

    public:
        Vec3 box_min;
        Vec3 box_max;
        Group sides;
};

class Translate : public Object {
    public:
        Translate(Object* p, const Vec3& displacement): ptr(p), offset(displacement) {}

        virtual bool intersect(const Ray& r, double t_min, double t_max, Hit& rec) const override {
            Ray moved_r(r.origin() - offset, r.direction(), r.time);
            if (!ptr->intersect(moved_r, t_min, t_max, rec))
                return false;

            rec.p += offset;
            rec.set_face_normal(moved_r, rec.norm);

            return true;
        }

        virtual bool bounding_box(double time0, double time1, AABB& output_box) const override{
            if (!ptr->bounding_box(time0, time1, output_box))
                return false;
            output_box = AABB(output_box.min() + offset, output_box.max() + offset);
            return true;
        }

        AABB get_aabb() const override {
            AABB box;
            if (ptr->bounding_box(0, 1, box)) {
                return AABB(box.min() + offset, box.max() + offset);
            } else {
                return box;
            }
        }
        int longest_axis() const override {
            return this->get_aabb().longest_axis();
        }

    public:
        Object* ptr;
        Vec3 offset;
};

class Rotate_Y : public Object {
    public:
        Rotate_Y(Object* p, double angle) : ptr(p) {
            auto radians = degrees_to_radians(angle);
            sin_theta = sin(radians);
            cos_theta = cos(radians);
            hasbox = ptr->bounding_box(0, 1, bbox);

            Vec3 min( MAX_double,  MAX_double, MAX_double);
            Vec3 max(-MAX_double, -MAX_double, -MAX_double);

            for (int i = 0; i < 2; i++) {
                for (int j = 0; j < 2; j++) {
                    for (int k = 0; k < 2; k++) {
                        auto x = i*bbox.max().x + (1-i)*bbox.min().x;
                        auto y = j*bbox.max().y + (1-j)*bbox.min().y;
                        auto z = k*bbox.max().z + (1-k)*bbox.min().z;

                        auto newx =  cos_theta*x + sin_theta*z;
                        auto newz = -sin_theta*x + cos_theta*z;

                        Vec3 tester(newx, y, newz);

                        for (int c = 0; c < 3; c++) {
                            min[c] = fmin(min[c], tester[c]);
                            max[c] = fmax(max[c], tester[c]);
                        }
                    }
                }
            }

            bbox = AABB(min, max);
        }

        virtual bool intersect(const Ray& r, double t_min, double t_max, Hit& rec) const override{
            auto origin = r.origin();
            auto direction = r.direction();

            origin[0] = cos_theta*r.origin()[0] - sin_theta*r.origin()[2];
            origin[2] = sin_theta*r.origin()[0] + cos_theta*r.origin()[2];

            direction[0] = cos_theta*r.direction()[0] - sin_theta*r.direction()[2];
            direction[2] = sin_theta*r.direction()[0] + cos_theta*r.direction()[2];

            Ray rotated_r(origin, direction, r.time);

            if (!ptr->intersect(rotated_r, t_min, t_max, rec))
                return false;

            auto p = rec.p;
            auto normal = rec.norm;

            p[0] =  cos_theta*rec.p[0] + sin_theta*rec.p[2];
            p[2] = -sin_theta*rec.p[0] + cos_theta*rec.p[2];

            normal[0] =  cos_theta*rec.norm[0] + sin_theta*rec.norm[2];
            normal[2] = -sin_theta*rec.norm[0] + cos_theta*rec.norm[2];

            rec.p = p;
            rec.set_face_normal(rotated_r, normal);

            return true;
        }

        virtual bool bounding_box(double time0, double time1, AABB& output_box) const override {
            output_box = bbox;
            return hasbox;
        }

        AABB get_aabb() const override {
            return bbox;
        }
        int longest_axis() const override {
            return this->get_aabb().longest_axis();
        }

    public:
        Object* ptr;
        double sin_theta;
        double cos_theta;
        bool hasbox;
        AABB bbox;
};

#endif