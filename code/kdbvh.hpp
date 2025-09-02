#ifndef __KDBVHNODE_H__
#define __KDBVHNODE_H__

#include "object.hpp"
#include "aabb.hpp"
#include <vector>
#include <algorithm>

class KDBVHNode: public Object {
public:
    KDBVHNode* left;
    KDBVHNode* right;
    AABB box;
    std::vector<Object*> objects;

    KDBVHNode(std::vector<Object*> objs) : objects(objs), left(NULL), right(NULL) {
        if (objs.size() == 1) {
            box = objs[0]->get_aabb();
        } else if (objs.size() > 1) {
            box = combine_box(objs[0]->get_aabb(), objs[1]->get_aabb());

            for (size_t i = 2; i < objs.size(); i++) {
                box = combine_box(box, objs[i]->get_aabb());
            }
        }
    }

    void split() {
        if (objects.size() < 2) {
            return;
        }

        int axis = box.longest_axis();
        double midPoint;
        if (axis == 0) {
            midPoint = (box.min().x + box.max().x) / 2.0;
        } else if (axis == 1) {
            midPoint = (box.min().y + box.max().y) / 2.0;
        } else {  // axis == 2
            midPoint = (box.min().z + box.max().z) / 2.0;
        }

        std::vector<Object*> left_objs;
        std::vector<Object*> right_objs;

        for (auto obj : objects) {
            AABB obj_box = obj->get_aabb();
            double val;
            if (axis == 0) {
                val = obj_box.min().x;
            } else if (axis == 1) {
                val = obj_box.min().y;
            } else {  // axis == 2
                val = obj_box.min().z;
            }
            
            if (val < midPoint) {
                left_objs.push_back(obj);
            } else {
                right_objs.push_back(obj);
            }
        }

        if (left_objs.size() == 0 || right_objs.size() == 0) {
            left_objs.insert(left_objs.end(), right_objs.begin(), right_objs.end());
            right_objs.clear();
        }

        if (left_objs.size() > 0) {
            left = new KDBVHNode(left_objs);
            left->split();
        }

        if (right_objs.size() > 0) {
            right = new KDBVHNode(right_objs);
            right->split();
        }
    }

    virtual bool intersect(const Ray& r, double t_min, double t_max, Hit& rec) const override {
        if (!box.hit(r, t_min, t_max)) {
            return false;
        }

        bool hit_left = (left != NULL) && left->intersect(r, t_min, t_max, rec);
        bool hit_right = (right != NULL) && right->intersect(r, t_min, hit_left ? rec.t : t_max, rec);

        return hit_left || hit_right;
    }

    virtual bool bounding_box(double t0, double t1, AABB& out_box) const override {
        out_box = box;
        return true;
    }

    virtual AABB get_aabb() const override {
        return box;
    }

    virtual int longest_axis() const override {
        Vec3 diagonal = box.max() - box.min();
        if (diagonal.x > diagonal.y && diagonal.x > diagonal.z)
            return 0; // x-axis is longest
        else if (diagonal.y > diagonal.z)
            return 1; // y-axis is longest
        else
            return 2; // z-axis is longest
    }
};

#endif
