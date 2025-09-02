#ifndef __BBOX_H__
#define __BBOX_H__

#include "camera.hpp"
#include "utils.hpp"

class AABB {
    Vec3 minV, maxV;
public:
    AABB() = default;
    AABB(const Vec3& minv, const Vec3& maxv): minV(minv), maxV(maxv) { } 
    Vec3 min() const {return minV;}
    Vec3 max() const {return maxV;} 
    bool hit(const Ray& r, double t_min, double t_max) const {
        for (int a = 0; a < 3; a++) {
            auto invD = 1.0f / r.direction()[a];
            auto t0 = (min()[a] - r.origin()[a]) * invD;
            auto t1 = (max()[a] - r.origin()[a]) * invD;
            if (invD < 0.0f)
                std::swap(t0, t1);
            t_min = t0 > t_min ? t0 : t_min;
            t_max = t1 < t_max ? t1 : t_max;
            if (t_max <= t_min)
                return false;
        }
        return true;
    }

    int longest_axis() const {
        double x = maxV.x - minV.x;
        double y = maxV.y - minV.y;
        double z = maxV.z - minV.z;

        if (x > y && x > z) {
            return 0; // X-axis is longest
        } else if (y > x && y > z) {
            return 1; // Y-axis is longest
        } else {
            return 2; // Z-axis is longest
        }
    }
};

AABB combine_box(const AABB& a, const AABB& b) {
    Vec3 minV (
        fmin(a.min().x, b.min().x),
        fmin(a.min().y, b.min().y),
        fmin(a.min().z, b.min().z)
    );
    Vec3 maxV (
        fmax(a.max().x, b.max().x),
        fmax(a.max().y, b.max().y),
        fmax(a.max().z, b.max().z)
    );
    return AABB(minV, maxV);
}

#endif
