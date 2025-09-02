#include "utils.hpp"
#include "object.hpp"
#include "camera.hpp"
#include "material.hpp"
#include "image.hpp"
#include "kdbvh.hpp"
#include "texture.hpp"
#include <cstring>
#include "scene_parser.hpp"

const int max_depth = 50;

using namespace std;

Group moving_scene() {
    Group world;
    auto ground_material = new Diffuse(new ConstantTexture(Vec3(0.5, 0.5, 0.5)));
    Object *ground = new Sphere(1000, Vec3(0, -1000, 0), ground_material);
    world.add(ground);

    for (int a = -11; a < 11; a++) {
        for (int b = -11; b < 11; b++) {
            auto choose_mat = drand48();
            Vec3 center(a + 0.9 * drand48(), 0.2, b + 0.9 * drand48());
            if ((center - Vec3(4, 0.2, 0)).len() > 0.9) {
                Material *sphere_material = nullptr;
                Object *new_obj = nullptr;
                auto albedo = new ConstantTexture(Vec3(drand48(), drand48(), drand48()));
                sphere_material = new Diffuse(albedo);
                new_obj = new MovSphere(center, center + Vec3(0,0.5 * drand48(),0), 0.0, 1.0, 0.2, sphere_material);
                world.add(new_obj);
            }
        }
    }

    auto material3 = new Specular(new ConstantTexture(Vec3(0.7, 0.6, 0.5)), 0.0);
    world.add(new Sphere(1.0, Vec3(4, 1, 0), material3));

    return world;
}

Group final_scene() {
    Group box_list;
    Material* ground_mat = new Diffuse(new ConstantTexture(Vec3(0.48, 0.83, 0.53)));
    const int side = 20;
    for(int i = 0; i < side; i++){
        for(int j = 0; j < side; j++){
            double w = 100.0;
            double x0 = -1000.0 + i * w;
            double z0 = -1000.0 + j * w;
            double y0 = 0.0;
            double x1 = x0 + w;
            double y1 = random_double(1, 101);
            double z1 = z0 + w;
            Object*ground=new Box(Vec3(x0, y0, z0),Vec3(x1, y1, z1), ground_mat);
            box_list.add(ground);
        }
    }

    Group world;

    world.add(new KDBVHNode(box_list.getList()));
    auto light = new DiffuseLight(new ConstantTexture(Vec3(7, 7, 7)));
    world.add(new XZ_RECT(123,423,147,412,554,light));

    auto center1 = Vec3(400,400,200);
    auto center2 = Vec3(30,0,0) + center1;
    auto moving_sphere_material = new Diffuse(new ConstantTexture(Vec3(0.7,0.3,0.1)));
    world.add(new MovSphere(center1, center2, 0, 1, 50, moving_sphere_material));

    world.add(new Sphere(double(50), Vec3(260,150,45), new Refract(1.5)));
    world.add(new Sphere((double)50, Vec3(0,150,145), new Specular(new ConstantTexture(Vec3(0.8,0.8,0.9)), 1.0)));

    auto boundary = new Sphere(70, Vec3(360,150,145), new Refract(1.5));
    world.add(boundary);
    world.add(new ConstantMedium(boundary,0.2, Vec3(0.2,0.4,0.9)));
    boundary = new Sphere(5000, Vec3(0, 0, 0), new Refract(1.5));
    world.add(new ConstantMedium(boundary, .0001, Vec3(1,1,1)));

    int width, height, comp;
    unsigned char* data = stbi_load("texture/earth.jpeg", &width, &height, &comp, 0);
    auto emat = new Diffuse(new ImageTexture(data, width, height));
    world.add(new Sphere(100,Vec3(400, 200, 400),emat));

    Group boxes2;

    auto white = new Diffuse(new ConstantTexture(Vec3(.73, .73, .73)));
    int ns = 1000;
    for (int j = 0; j < ns; j++) {
        boxes2.add(new Sphere(10, Vec3(random_double(0,165), random_double(0,165), random_double(0,165)), white));
    }

    world.add(new Translate(new Rotate_Y(new KDBVHNode(boxes2.getList()), 15), Vec3(-100,270,395)));

    return world;
}

Group cornell_box() {
    Group world;

    auto red = new Diffuse(new ConstantTexture(Vec3(.65, .05, .05)));
    auto white = new Diffuse(new ConstantTexture(Vec3(.73, .73, .73)));
    auto green = new Diffuse(new ConstantTexture(Vec3(.12, .45, .15)));
    auto light = new DiffuseLight(new ConstantTexture(Vec3(15, 15, 15)));

    world.add(new YZ_RECT(0, 555, 0, 555, 555, green));
    world.add(new YZ_RECT(0, 555, 0, 555, 0, red));

    world.add(new XZ_RECT(213, 343, 227, 332, 554, light));
    world.add(new XZ_RECT(0, 555, 0, 555, 0, white));
    world.add(new XZ_RECT(0, 555, 0, 555, 555, white));

    world.add(new XY_RECT(0, 555, 0, 555, 555, white));

    return world;
}

Group cornell_smoke() {
    Group objects;

    auto red = new Diffuse(new ConstantTexture(Vec3(.65, .05, .05)));
    auto white = new Diffuse(new ConstantTexture(Vec3(.73, .73, .73)));
    auto green = new Diffuse(new ConstantTexture(Vec3(.12, .45, .15)));
    auto light = new DiffuseLight(new ConstantTexture(Vec3(7, 7, 7)));

    objects.add(new YZ_RECT(0, 555, 0, 555, 555, green));
    objects.add(new YZ_RECT(0, 555, 0, 555, 0, red));
    objects.add(new XZ_RECT(113, 443, 127, 432, 554, light));
    objects.add(new XZ_RECT(0, 555, 0, 555, 555, white));
    objects.add(new XZ_RECT(0, 555, 0, 555, 0, white));
    objects.add(new XY_RECT(0, 555, 0, 555, 555, white));

    Object* box1 = new Box(Vec3(0,0,0), Vec3(165,330,165), white);
    box1 = new Rotate_Y(box1, 15);
    box1 = new Translate(box1, Vec3(265,0,295));

    Object* box2 = new Box(Vec3(0,0,0), Vec3(165,165,165), white);
    box2 = new Rotate_Y(box2, -18);
    box2 = new Translate(box2, Vec3(130,0,65));

    objects.add(new ConstantMedium(box1, 0.01, new ConstantTexture(Vec3(0,0,0))));
    objects.add(new ConstantMedium(box2, 0.01, new ConstantTexture(Vec3(1,1,1))));

    return objects;
}


Group single() {
    Group world;

    auto ground_material = new Diffuse(new ConstantTexture(Vec3(0.5, 0.5, 0.5)));
    auto red = new Diffuse(new ConstantTexture(Vec3(.65, .05, .05)));

    world.add(new Sphere(0.7, Vec3(0,0.2,-1), red));
    world.add(new Sphere(100, Vec3(0,-100.5,-1), ground_material));

    return world;
}

Group single_sticker() {
    Group world;
    auto ground_material = new Diffuse(new ConstantTexture(Vec3(0.5, 0.5, 0.5)));
    int width, height, comp;
    unsigned char* data = stbi_load("texture/marble.jpeg", &width, &height, &comp, 0);
    auto emat = new Diffuse(new ImageTexture(data, width, height));

    world.add(new Sphere(0.7, Vec3(0,0.2,-1), emat));
    world.add(new Sphere(100, Vec3(0,-100.5,-1), ground_material));

    return world;
}

Group single_mesh() {
    Group world;

    auto red = new Diffuse(new ConstantTexture(Vec3(.65, .05, .05)));
    auto white = new Diffuse(new ConstantTexture(Vec3(.73, .73, .73)));
    auto green = new Diffuse(new ConstantTexture(Vec3(.12, .45, .15)));
    auto light = new DiffuseLight(new ConstantTexture(Vec3(15, 15, 15)));

    world.add(new YZ_RECT(0, 555, 0, 555, 555, green));
    world.add(new YZ_RECT(0, 555, 0, 555, 0, red));

    world.add(new XZ_RECT(213, 343, 227, 332, 554, light));
    world.add(new XZ_RECT(0, 555, 0, 555, 0, white));
    world.add(new XZ_RECT(0, 555, 0, 555, 555, white));

    world.add(new XY_RECT(0, 555, 0, 555, 555, white));

    world.add(new Mesh("objects/bunny_200.obj", red,Vec3(100,330,165), Vec3(10000,10000,10000), 30));

    return world;
}

Group single_curve() {
    Group world;

    auto red = new Diffuse(new ConstantTexture(Vec3(.65, .05, .05)));
    auto white = new Diffuse(new ConstantTexture(Vec3(.73, .73, .73)));
    auto green = new Diffuse(new ConstantTexture(Vec3(.12, .45, .15)));
    auto light = new DiffuseLight(new ConstantTexture(Vec3(15, 15, 15)));

    world.add(new YZ_RECT(0, 555, 0, 555, 555, green));
    world.add(new YZ_RECT(0, 555, 0, 555, 0, red));

    world.add(new XZ_RECT(213, 343, 227, 332, 554, light));
    world.add(new XZ_RECT(0, 555, 0, 555, 0, white));
    world.add(new XZ_RECT(0, 555, 0, 555, 555, white));

    world.add(new XY_RECT(0, 555, 0, 555, 555, white));

    std::vector<Vec3> curvelist;
    curvelist.push_back(Vec3(-1, 2, 0));
    curvelist.push_back(Vec3(0, 0, 0 ));
    curvelist.push_back(Vec3(-4, -1, 0));
    curvelist.push_back(Vec3(-4, -1, 0));
    Curve* curve = new BezierCurve(curvelist);
    world.add(new RevSurface(curve, red, Vec3(165,330,165)));

    return world;
}

Camera* getCam(int w, int h) {
    Vec3 lookfrom(278, 278, -800);
    Vec3 lookat(278, 278, 0);
    Vec3 vup(0, 1, 0);
    double dist_to_focus = 10.0;
    double aperture = 0;
    double t1 = 0.0, t2 = 1.0;
    return new Camera(w, h, lookfrom, lookat, vup, 40 * PI / 180, aperture, dist_to_focus, t1, t2);
}

Vec3 get_color(const Ray &r, Object *objs, Vec3 bg, int depth){
    Hit hit;
    if (objs->intersect(r, 0.001, MAX_double, hit))
    {
        Ray s_ray;
        Vec3 dir = r.direction();
        Vec3 attenuation;
        Vec3 illuminated = hit.material->illuminate(hit.u, hit.v, hit.p);
        if (depth > 0 && hit.material->scatter(r, hit, attenuation, s_ray))
        {
            Vec3 new_color = get_color(s_ray, objs, bg, depth - 1).mult(attenuation);
            return new_color + illuminated;
        }
        else
            return illuminated;
    }

    else return bg;
}

int main(int argc, char **argv) {
    if (argc != 2) {
        fprintf(stderr, "Usage: ./main <samples_per_pixel:int>\n");
        return 1;
    }
    int w = 600, h = 600, samps = atoi(argv[1]); // # samples
    Image image(w, h);
    image.setAllPixel(Vec3());
    //DECIDE SCENE
    Group world = single_curve();
    Camera* camera = getCam(w, h);
    Vec3 color;
#pragma omp parallel for schedule(dynamic, 1) private(color) // OpenMP
    for (int y = 0; y < h; y++) {
        fprintf(stderr, "\rRendering (%d spp) %5.2f%%", samps, 100. * y / (h - 1));
        for (unsigned short x = 0; x < w; x++) {
            color = Vec3(0, 0, 0);
            for (int s = 0; s < samps; s++) {
                double u = double(x + drand48()) / double(w - 1);
                double v = double(y + drand48()) / double(h - 1);
                Ray ray = camera->generate_ray(u, v);
                color += get_color(ray, &world, Vec3(1,1, 1.0), max_depth);
            }
            image.setPixel(x, y, color / samps);
        }
    }
    image.SaveImage("test.png");
    return 0;
}