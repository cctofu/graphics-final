# Ray Tracing Realistic Image Renderer

This project is my **Spring 2023 Computer Graphics course** final assignment. It implements a **ray tracing–based realistic image renderer**, built on previous homework frameworks, with extended rendering features and complex effects.

---

## 📂 Project Structure

- `aabb.hpp`, `bvh_node.hpp`  
  Used for ray-object intersection acceleration (AABB bounding box, BVH hierarchy).
- `constant_medium.hpp`  
  Isotropic scattering medium, used for volumetric light/smoke effects.
- `curve.hpp`  
  Bezier curves and surfaces of revolution with intersection methods.
- `material.hpp`  
  Defines multiple materials (diffuse, refraction, reflection, light sources, isotropic scatter).
- `mesh.hpp`  
  Triangle mesh support with BVH + KD-tree acceleration.
- `shape.hpp`  
  Basic geometric primitives (sphere, triangle, rectangle) and their intersections.
- `texture.hpp`  
  Multiple texture types (solid color, checkerboard, Perlin noise, image textures).
- `utils.hpp`  
  Vector classes, matrix utilities, and helper functions.

Other resources:  
- `textures/` → texture images  
- `mesh/` → `.obj` models from Stanford public resources  
- `testcases/` → test input files  
- `external/` → file I/O utilities  

---

## ⚡ Rendering Acceleration

- Used **OpenMP** for parallelization, significantly reducing rendering time.  
- Addressed compiler compatibility issues on macOS where `g++`/`clang` lack OpenMP support.  

---

## 🎨 Implemented Features

### Basic Features
- Refraction, reflection, and shadows  
- Parametric surfaces (Newton method for intersections)

### Beginner-Level Features
- Anti-aliasing (multi-sample averaging)  
- Texture mapping (coordinate mapping, filtering)

### Intermediate-Level Features
- Diffuse reflection (rejection sampling method)  
- Depth of field (aperture and focal length simulation)  
- Motion blur (shutter-time interval sampling)  
- Soft shadows (natural in PT algorithm)

### Advanced Features
- Intersection acceleration (AABB, BVH, KD-tree)  
- Volumetric light (probabilistic scattering in media)

---

## 🌌 Results

Rendered example outputs:

- Final render  
  ![Final Result](/results/final.jpeg)

- Scene 1  
  ![Scene 1](/results/scene1.png)

- Scene 2  
  ![Scene 2](/results/scene2.png)

---

## 📝 Conclusion

In this project, I implemented a wide range of ray tracing features from basic to advanced. While some additional features could not be finished due to time constraints, and the complex scenes had limited sampling, the renderer successfully achieved realistic effects such as **reflection, refraction, depth of field, motion blur, and volumetric lighting**.  

Through this assignment, I gained a deeper appreciation of the complexity behind computer graphics rendering. Each successfully rendered image was both motivating and rewarding.  

Special thanks to my professor and TAs for their guidance throughout the semester, which gave me a solid foundation in computer graphics.  

---
