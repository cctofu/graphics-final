#include "utils.hpp"
#include "object.hpp"
#include "camera.hpp"
#include "material.hpp"
#include "image.hpp"
#include "texture.hpp"
#include <cstring>
#include <cmath>
#include "scene_parser.hpp"

const int max_depth = 20;

Vec3 get_color(const Ray &r, Object *objs, const Vec3 &bg, int depth, unsigned short* Xi, int maxDepth = 5) {
    if (depth >= maxDepth) {
        return Vec3();
    }
    
    Hit hit;

    if (!objs->intersect(r, 0.001, MAX_double, hit)) {
        return bg;
    }

    Ray s_ray;
    Vec3 dir = r.direction();
    Vec3 attenuation;
    Vec3 illuminated = hit.material->illuminate(hit.u, hit.v, hit.p);

    if (!hit.material->scatter(r, hit, attenuation, s_ray)) {
        return illuminated;
    }

    Vec3 f = attenuation;
    double p = std::max({f.x, f.y, f.z});

    if (depth > maxDepth) {
        if (erand48(Xi) < p) {
            f *= (1 / p);
        } else {
            return illuminated;
        }
    }

    Vec3 new_color = get_color(s_ray, objs, bg, depth + 1, Xi).mult(f);

    return new_color + illuminated;
}

int main(int argc, char **argv) {
    if (argc != 4) {
        fprintf(stderr, "Usage: ./main <input scene file> <output bmp file> <samp:int>\n");
        return 1;
    }
    SceneParser parser = SceneParser(argv[1]);
    Camera* camera = parser.getCamera();
    int w = camera->width, h =camera->height, samps = atoi(argv[3]) / 4; // # samples
    Image image(w, h);
    image.setAllPixel(Vec3());
    Group* world = parser.getGroup();
    Vec3 color;

#pragma omp parallel for schedule(dynamic, 1) private(color) // OpenMP
    for (int y = 0; y < h; y++){
        fprintf(stderr, "\rRendering (%d spp) %5.2f%%", samps, 100. * y / (h - 1));
        for (unsigned short x = 0, Xi[3] = {0, 0, (unsigned short)(y * y * y)}; x < w; x++){
            color = Vec3();
            for (int sy = 0, i = (h - y - 1) * w + x; sy < 2; sy++)
                for (int sx = 0; sx < 2; sx++){
                    Vec3 samp_color;
                    for (int s = 0; s < samps; s++) {
                        double r1 = 2 * erand48(Xi), dx = r1 < 1 ? sqrt(r1) - 1 : 1 - sqrt(2 - r1);
                        double r2 = 2 * erand48(Xi), dy = r2 < 1 ? sqrt(r2) - 1 : 1 - sqrt(2 - r2);
                        double u = double(x + (sx + 0.5 + dx)/2) / double(w);
                        double v = double(y + (sy + 0.5 + dy)/2) / double(h);
                        Ray ray = camera->generate_ray(u, v);
                        
                        samp_color += get_color(ray, world, parser.getBackgroundColor(), 0, Xi) * 1.0/ samps;
                    }
                    color += samp_color.clip() * 0.25;
                }
            image.setPixel(x, y, color);
        }
    }
    image.SaveImage(argv[2]);
    return 0;
}