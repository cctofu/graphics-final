#ifndef __KDTREE_H__
#define __KDTREE_H__

#include "mesh.hpp"

class KDNode {
public:
    KDNode* left;
    KDNode* right;
    AABB box;
    std::vector<Triangle*> triangles;

    KDNode() : left(nullptr), right(nullptr) {}
};

KDNode* buildKDTree(Group* triangles, int depth = 0) {
    KDNode* node = new KDNode();
    AABB bbox;

    if (!triangles || triangles->size() == 0) {
        return node;
    }

    if (triangles->size() == 1) {
        node->triangles.push_back(dynamic_cast<Triangle*>(triangles->list[0]));
        triangles->list[0]->bounding_box(0, 0, node->box);
        return node;
    }

    if (triangles->bounding_box(0, 0, bbox)) {
        node->box = bbox;
    }

    Group* leftTriangles = new Group();
    Group* rightTriangles = new Group();

    int axis = depth % 3;
    double mid = (bbox.max()[axis] + bbox.min()[axis]) * 0.5;

    for (auto object : triangles->list) {
        AABB triangleBox;
        if (object->bounding_box(0, 0, triangleBox)) {
            if (triangleBox.min()[axis] > mid) {
                rightTriangles->add(object);
            } else if (triangleBox.max()[axis] < mid) {
                leftTriangles->add(object);
            } else {
                node->triangles.push_back(dynamic_cast<Triangle*>(object));
            }
        }
    }

    if (leftTriangles->list.size() > 0) {
        node->left = buildKDTree(leftTriangles, depth + 1);
    }

    if (rightTriangles->list.size() > 0) {
        node->right = buildKDTree(rightTriangles, depth + 1);
    }

    return node;
}

#endif