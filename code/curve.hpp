#ifndef __CURVE_H__
#define __CURVE_H__

#include "utils.hpp"
#include "object.hpp"
#include <vector>
#include <utility>
#include <iostream>

#include <algorithm>
#include <tuple>

// TODO (PA3): Implement Bernstein class to compute spline basis function.
//       You may refer to the python-script for implementation.

// The CurvePoint object stores information about a point on a curve
// after it has been tesselated: the vertex (V) and the tangent (T)
// It is the responsiblility of functions that create these objects to fill in all the data.
struct CurvePoint {
    Vec3 V; // Vertex
    Vec3 T; // Tangent  (unit)
};

const int resolution = 10;

class Curve : public Object {
protected:
    std::vector<Vec3> controls;
    
public:
    explicit Curve(std::vector<Vec3> points) : controls(std::move(points)) {
        ymax = ymin = controls[0].y;
        radius = 0;
        for (auto p: controls) {
            ymax = fmax(ymax, p.y);
            ymin = fmin(ymin, p.y);
            radius = fmax(radius, fabs(p.x));
            radius = fmax(radius, fabs(p.z));
        }
    }

    virtual bool intersect(const Ray &r, double tmin, double tmax, Hit &h) const override {
        return false;
    }

    std::vector<Vec3> &getControls() {
        return controls;
    }
    virtual bool bounding_box(double t0, double t1, AABB& box) const override{
        return false;
    }

    virtual AABB get_aabb() const {
        return AABB(Vec3(-radius, ymin, -radius), Vec3(radius, ymax, radius));
    }

    virtual int longest_axis() const {
        double x = 2 * radius;
        double y = ymax - ymin;
        double z = 2 * radius;
        if (x > y && x > z)
            return 0; // x is the longest
        else if (y > x && y > z)
            return 1; // y is the longest
        else
            return 2; // z is the longest
    }

    virtual CurvePoint getPoint(double mu) = 0;
    double ymin, ymax, radius;
};


class BezierCurve : public Curve {
public:
    explicit BezierCurve(const std::vector<Vec3> &points) : Curve(points) {
        if (points.size() < 4 || points.size() % 3 != 1) {
            printf("Number of control points of BezierCurve must be 3n+1!\n");
            exit(0);
        }
    }
    virtual CurvePoint getPoint(double mu) {
            int n = controls.size() - 1;
            double t = mu + 0.5 / (double)resolution;
            Vec3 f = Vec3();
            for (int i = 0; i <= n; ++i) {
                f += controls[i] * BezierFunc(n, i, t);
            }
            Vec3 f_prime = Vec3();
            for (int i = 0; i <= n - 1; i++) {
                f_prime += (controls[i+1] - controls[i]) * double(n) * BezierFunc(n-1, i, t);
            }
            CurvePoint tmp_curPoint;
            tmp_curPoint.V = f;
            tmp_curPoint.T = f_prime;
            return tmp_curPoint;
    }


private:
    double BezierFunc(int n, int i, double t) {
        double result = 0.0;
        if (i > n || i < 0) {
            result = 0.0;
        } else {
            result = pow(1-t, n-i) * pow(t, i);
            long long mul_fac = calc(n, i);
            result = result * double(mul_fac);
        }
        return result;
    }
    long long calc(int n, int i) {
        long long res = 1;
        for (int j = 0; j < i; j++) {
            res *= (long long) (n - j);
        }
        for (int j = 2; j <= i; j++) {
            res /= (long long)j;
        }
        return res;
    }

    virtual AABB get_aabb() const override {
        return Curve::get_aabb();
    }

    virtual int longest_axis() const override {
        return Curve::longest_axis();
    }
};

const double EPIS = 1e-5;

class RevSurface : public Object {
    Curve *pCurve;
    AABB nbox;
    const int NEWTON_STEPS = 20;
    const double NEWTON_EPS = 1e-4;
    void get_UV(const Ray& r, double t, double &theta, double &mu) const {   
        Vec3 pt = r.origin() + r.direction() * t - center;
        theta = atan2(-pt.z, pt.x) + PI;
        mu = (pCurve->ymax - pt.y) / (pCurve->ymax - pCurve->ymin);
    }
    Vec3 center;

public:
    RevSurface(Curve *pCurve, Material* mat, Vec3 c) : pCurve(pCurve), center(c) {
        material = mat;
        // Check flat.
        for (const auto &cp : pCurve->getControls()) {
            if (cp.z != 0.0) {
                printf("Profile of revSurface must be flat on xy plane.\n");
                exit(0);
            }
        }
        Vec3 minV(-pCurve->radius, pCurve->ymin - 1, -pCurve->radius);
        Vec3 maxV(pCurve->radius, pCurve->ymax + 1, pCurve->radius);
        nbox = AABB(minV + center, maxV + center);
    }

    ~RevSurface() {
        delete pCurve;
    }

    bool newtonFunc(const Ray &r, double &t, double &theta, double &mu,
                Vec3 &normal, Vec3 &point) const {
        Vec3 dmu, dtheta;
        for (int i = 0; i < NEWTON_STEPS; ++i) {
            if (theta < 0.0) theta += 2 * PI;
            if (theta >= 2 * PI) theta = fmod(theta, 2 * PI);
            if (mu >= 1) mu = 1.0 - EPIS;
            if (mu <= 0) mu = EPIS;
            point = getPoint(theta, mu, dtheta, dmu);
            Vec3 f = r.origin() + r.direction() * t - point;
            double dist2 = f.len2();
            normal = dmu % dtheta;
            if (dist2 < NEWTON_EPS) return true;
            double D = r.direction().dot(normal);
            t -= dmu.dot(dtheta % f) / D;
            mu -= r.direction().dot(dtheta % f) / D;
            theta += r.direction().dot(dmu % f) / D;
        }
        return false;
    }

    Vec3 getPoint(const double &theta, const double &mu, Vec3 &dtheta,
                      Vec3 &dmu) const {
        Vec3 pt;
        Quat4f rot;
        rot.setAxisAngle(theta, Vec3(0, 1, 0));
        Matrix3f rotMat = Matrix3f::rotation(rot);
        CurvePoint cp = pCurve->getPoint(mu);
        pt = rotMat * cp.V + center;
        dmu = rotMat * cp.T;
        dtheta = Vec3(-cp.V.x * sin(theta), 0, -cp.V.x * cos(theta));
        return pt;
    }

    bool intersect(const Ray &r, double t_min, double t_max, Hit &h) const override {
        double t = t_min, theta, mu;
        if (!nbox.hit(r, t, t_max)) return false;
        get_UV(r, t, theta, mu);
        Vec3 normal, point;
        if (!newtonFunc(r, t, theta, mu, normal, point)) {
            return false;
        }
        if (!isnormal(mu) || !isnormal(theta) || !isnormal(t)) return false;
        if (t < 0 || mu < 0 || mu > 1 ||
            t > t_max)
            return false;
        h.t = t;
        h.p = r.point(t);
        h.material = this->material;
        h.norm = normal.normalized();
        h.u = theta / (2 *PI);
        h.v = mu;
        return true;
    }

    virtual bool bounding_box(double t0, double t1, AABB& box) const override{
        box = nbox;
        return true;
    }

    virtual AABB get_aabb() const override {
        return nbox;
    }

    virtual int longest_axis() const override {
        Vec3 lengths = nbox.max() - nbox.min();
        if (lengths.x > lengths.y && lengths.x > lengths.z)
            return 0; // x is the longest
        else if (lengths.y > lengths.x && lengths.y > lengths.z)
            return 1; // y is the longest
        else
            return 2; // z is the longest
    }

};


#endif