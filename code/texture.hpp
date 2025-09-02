#ifndef __TEXTURE_H__
#define __TEXTURE_H__

#include "utils.hpp"

class Texture {
public:
    virtual Vec3 value(double u, double v, const Vec3& p) const = 0;
};

class ConstantTexture: public Texture {
    Vec3 color;

public:
    ConstantTexture() = default;
    ConstantTexture(const Vec3& c): color(c) {}
    virtual Vec3 value(double u, double v, const Vec3& p) const override {
        return color;
    }
};

class ImageTexture: public Texture {
    unsigned char* data;
    int w, h;
public:
    ImageTexture() = default;
    ImageTexture(unsigned char* pix, int width, int height):
            data(pix), w(width), h(height) {}
    virtual Vec3 value(double u, double v, const Vec3& p) const override {
        int x = u * w, y = (1-v) * h - 0.001;
        x = x < 0 ? 0 : (x > w - 1 ? w - 1 : x);
        y = y < 0 ? 0 : (y > h - 1 ? h - 1 : y);
        double r = int(data[3*x+3*w*y]) / 255.0;
        double g = int(data[3*x+3*w*y+1]) / 255.0;
        double b = int(data[3*x+3*w*y+2]) / 255.0;
        return Vec3(r, g, b);

    }
};

#endif