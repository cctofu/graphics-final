#ifndef MESH_H
#define MESH_H

#include <vector>
#include "mesh.hpp"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <utility>
#include <sstream>
#include "object.hpp"

using namespace std;

class Mesh : public Object {

public:
    Mesh(const char *filename, Material *m, const Vec3& center, const Vec3& scale, double ry){
        material = m;
        std::ifstream f;
        f.open(filename);
        if (!f.is_open()) {
            std::cout << "Cannot open " << filename << "\n";
            return;
        }
        std::string line;
        std::string vTok("v");
        std::string fTok("f");
        char bslash = '/', space = ' ';
        std::string tok;
        int texID;
        while (true) {
            std::getline(f, line);
            if (f.eof()) {
                break;
            }
            if (line.size() < 3) {
                continue;
            }
            if (line.at(0) == '#') {
                continue;
            }
            std::stringstream ss(line);
            ss >> tok;
            if (tok == vTok) {
                Vec3 vec;
                ss >> vec.x >> vec.y >> vec.z;
                double theta = ry * PI / 180.0;
                double x = vec.x * cos(theta) + vec.z * sin(theta);
                double z = vec.x * -sin(theta) + vec.z * cos(theta);
                vec.x = x; vec.z = z;
                vec = vec.mult(scale) + center;
                v.push_back(vec);
            } else if (tok == fTok) {
                if (line.find(bslash) != std::string::npos) {
                    std::replace(line.begin(), line.end(), bslash, space);
                    std::stringstream facess(line);
                    TriangleIndex trig;
                    facess >> tok;
                    for (int ii = 0; ii < 3; ii++) {
                        facess >> trig[ii] >> texID;
                        trig[ii]--;
                    }
                    t.push_back(trig);
                } else {
                    TriangleIndex trig;
                    for (int ii = 0; ii < 3; ii++) {
                        ss >> trig[ii];
                        trig[ii]--;
                    }
                    t.push_back(trig);
                }
            }
        }
        computeNormal();
        f.close();
    }

    struct TriangleIndex {
        TriangleIndex() {
            x[0] = 0; x[1] = 0; x[2] = 0;
        }
        int &operator[](const int i) { return x[i]; }
        // By Computer Graphics convention, counterclockwise winding is front face
        int x[3] = {};
    };

    vector<Vec3> v;
    vector<TriangleIndex> t;
    vector<Vec3> n;
    virtual bool intersect(const Ray &r, double tmin, double t_max, Hit &h) const override{
                bool result = false;
        for (int triId = 0; triId < (int) t.size(); ++triId) {
            TriangleIndex triIndex = t[triId];
            Triangle triangle(v[triIndex[0]],
                            v[triIndex[1]], v[triIndex[2]], material);
            if(triangle.intersect(r, tmin, t_max, h)) {
                result = true;
                t_max = h.t;
            }
        }
        return result;
    }

    virtual bool bounding_box(double t0, double t1, AABB& box) const override{
        Vec3 minV = v[0], maxV = v[0];
        std::vector<Vec3> vv = v;
        for (int i = 1; v.size(); ++i) {
            for (int r = 0; r < 3; r++) {
                minV[r] = fmin(minV[r], vv[i][r]);
                maxV[r] = fmax(maxV[r], vv[i][r]);
            }
        }
        box = AABB(minV, maxV);
        return true;
    }

    Group* get_all_triangles() {
        Group* tmp_list = new Group();
        for (int triId = 0; triId < (int) t.size(); ++triId) {
            TriangleIndex& triIndex = t[triId];
            Triangle* tri = new Triangle(v[triIndex[0]],
                          v[triIndex[1]], v[triIndex[2]], material);
            tmp_list->add(tri);
        }
        return tmp_list;
    }

    virtual AABB get_aabb() const override {
        Vec3 minV = v[0], maxV = v[0];
        for (const auto& vert : v) {
            minV.x = fmin(minV.x, vert.x);
            minV.y = fmin(minV.y, vert.y);
            minV.z = fmin(minV.z, vert.z);
            
            maxV.x = fmax(maxV.x, vert.x);
            maxV.y = fmax(maxV.y, vert.y);
            maxV.z = fmax(maxV.z, vert.z);
        }
        return AABB(minV, maxV);
    }


    Vec3 subtract(const Vec3& v1, const Vec3& v2) const {
        return Vec3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
    }

    virtual int longest_axis() const override {
        AABB box = get_aabb();
        Vec3 extent = subtract(box.max(), box.min());
        if (extent.x > extent.y && extent.x > extent.z) {
            return 0;
        } else if (extent.y > extent.z) {
            return 1;
        } else {
            return 2;
        }
    }

private:
    void computeNormal() {
        n.resize(t.size());
        for (int triId = 0; triId < (int) t.size(); ++triId) {
            TriangleIndex& triIndex = t[triId];
            Vec3 a = v[triIndex[1]] - v[triIndex[0]];
            Vec3 b = v[triIndex[2]] - v[triIndex[0]];
            b = a % b;
            n[triId] = b.normalized();
        }
    }
};

#endif
