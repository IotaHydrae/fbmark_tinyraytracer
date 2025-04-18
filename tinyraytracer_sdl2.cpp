#include <tuple>
#include <vector>
#include <algorithm>
#include <cmath>

#include <chrono>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/fb.h>
#include <SDL2/SDL.h>

struct vec3 {
    float x=0, y=0, z=0;
          float& operator[](const int i)       { return i==0 ? x : (1==i ? y : z); }
    const float& operator[](const int i) const { return i==0 ? x : (1==i ? y : z); }
    vec3  operator*(const float v) const { return {x*v, y*v, z*v};       }
    float operator*(const vec3& v) const { return x*v.x + y*v.y + z*v.z; }
    vec3  operator+(const vec3& v) const { return {x+v.x, y+v.y, z+v.z}; }
    vec3  operator-(const vec3& v) const { return {x-v.x, y-v.y, z-v.z}; }
    vec3  operator-()              const { return {-x, -y, -z};          }
    float norm() const { return std::sqrt(x*x+y*y+z*z); }
    vec3 normalized() const { return (*this)*(1.f/norm()); }
};

vec3 cross(const vec3 v1, const vec3 v2) {
    return { v1.y*v2.z - v1.z*v2.y, v1.z*v2.x - v1.x*v2.z, v1.x*v2.y - v1.y*v2.x };
}

struct Material {
    float refractive_index = 1;
    float albedo[4] = {2,0,0,0};
    vec3 diffuse_color = {0,0,0};
    float specular_exponent = 0;
};

struct Sphere {
    vec3 center;
    float radius;
    Material material;
};

constexpr Material      ivory = {1.0, {0.9,  0.5, 0.1, 0.0}, {0.4, 0.4, 0.3},   50.};
constexpr Material      glass = {1.5, {0.0,  0.9, 0.1, 0.8}, {0.6, 0.7, 0.8},  125.};
constexpr Material red_rubber = {1.0, {1.4,  0.3, 0.0, 0.0}, {0.3, 0.1, 0.1},   10.};
constexpr Material     mirror = {1.0, {0.0, 16.0, 0.8, 0.0}, {1.0, 1.0, 1.0}, 1425.};

constexpr Sphere spheres[] = {
    {{-3,    0,   -16}, 2,      ivory},
    {{-1.0, -1.5, -12}, 2,      glass},
    {{ 1.5, -0.5, -18}, 3, red_rubber},
    {{ 7,    5,   -18}, 4,     mirror}
};

constexpr vec3 lights[] = {
    {-20, 20,  20},
    { 30, 50, -25},
    { 30, 20,  30}
};

vec3 reflect(const vec3 &I, const vec3 &N) {
    return I - N*2.f*(I*N);
}

vec3 refract(const vec3 &I, const vec3 &N, const float eta_t, const float eta_i=1.f) { // Snell's law
    float cosi = - std::max(-1.f, std::min(1.f, I*N));
    if (cosi<0) return refract(I, -N, eta_i, eta_t); // if the ray comes from the inside the object, swap the air and the media
    float eta = eta_i / eta_t;
    float k = 1 - eta*eta*(1 - cosi*cosi);
    return k<0 ? vec3{1,0,0} : I*eta + N*(eta*cosi - std::sqrt(k)); // k<0 = total reflection, no ray to refract. I refract it anyways, this has no physical meaning
}

std::tuple<bool,float> ray_sphere_intersect(const vec3 &orig, const vec3 &dir, const Sphere &s) { // ret value is a pair [intersection found, distance]
    vec3 L = s.center - orig;
    float tca = L*dir;
    float d2 = L*L - tca*tca;
    if (d2 > s.radius*s.radius) return {false, 0};
    float thc = std::sqrt(s.radius*s.radius - d2);
    float t0 = tca-thc, t1 = tca+thc;
    if (t0>.001) return {true, t0};  // offset the original point by .001 to avoid occlusion by the object itself
    if (t1>.001) return {true, t1};
    return {false, 0};
}

std::tuple<bool,vec3,vec3,Material> scene_intersect(const vec3 &orig, const vec3 &dir) {
    vec3 pt, N;
    Material material;

    float nearest_dist = 1e10;
    if (std::abs(dir.y)>.001) { // intersect the ray with the checkerboard, avoid division by zero
        float d = -(orig.y+4)/dir.y; // the checkerboard plane has equation y = -4
        vec3 p = orig + dir*d;
        if (d>.001 && d<nearest_dist && std::abs(p.x)<10 && p.z<-10 && p.z>-30) {
            nearest_dist = d;
            pt = p;
            N = {0,1,0};
            material.diffuse_color = (int(.5*pt.x+1000) + int(.5*pt.z)) & 1 ? vec3{.3, .3, .3} : vec3{.3, .2, .1};
        }
    }

    for (const Sphere &s : spheres) { // intersect the ray with all spheres
        auto [intersection, d] = ray_sphere_intersect(orig, dir, s);
        if (!intersection || d > nearest_dist) continue;
        nearest_dist = d;
        pt = orig + dir*nearest_dist;
        N = (pt - s.center).normalized();
        material = s.material;
    }
    return { nearest_dist<1000, pt, N, material };
}

vec3 cast_ray(const vec3 &orig, const vec3 &dir, const int depth=0) {
    auto [hit, point, N, material] = scene_intersect(orig, dir);
    if (depth>4 || !hit)
        return {0.2, 0.7, 0.8}; // background color

    vec3 reflect_dir = reflect(dir, N).normalized();
    vec3 refract_dir = refract(dir, N, material.refractive_index).normalized();
    vec3 reflect_color = cast_ray(point, reflect_dir, depth + 1);
    vec3 refract_color = cast_ray(point, refract_dir, depth + 1);

    float diffuse_light_intensity = 0, specular_light_intensity = 0;
    for (const vec3 &light : lights) { // checking if the point lies in the shadow of the light
        vec3 light_dir = (light - point).normalized();
        auto [hit, shadow_pt, trashnrm, trashmat] = scene_intersect(point, light_dir);
        if (hit && (shadow_pt-point).norm() < (light-point).norm()) continue;
        diffuse_light_intensity  += std::max(0.f, light_dir*N);
        specular_light_intensity += std::pow(std::max(0.f, -reflect(-light_dir, N)*dir), material.specular_exponent);
    }
    return material.diffuse_color * diffuse_light_intensity * material.albedo[0] + vec3{1., 1., 1.}*specular_light_intensity * material.albedo[1] + reflect_color*material.albedo[2] + refract_color*material.albedo[3];
}

int main() {
    const vec3 target = {0, 0, -15};
    const vec3 up = {1, 1, 1};
    const int total_frames = 100;
    const float radius = 45.f;
    const float fov = 1.05; // 60 degrees field of view in radians
    float angle = 0.0f;
    int width = 480;
    int height = 320;

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL_Init Error: " << SDL_GetError() << std::endl;
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow("TinyRaytracer Benchmark", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        width, height, SDL_WINDOW_SHOWN);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_Texture* texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB24, SDL_TEXTUREACCESS_STREAMING,
            width, height);

    std::vector<vec3> framebuffer(width*height);
    std::vector<uint8_t> rgb888(width * height * 3);

    std::cout << "Starting Benchmark..." << std::endl;

    auto start_time = std::chrono::steady_clock::now();
    for (int frame_count = 0; frame_count < total_frames; frame_count++) {

        vec3 eye = {
            radius * std::sin(angle),
            0,
            radius * std::cos(angle)
        };

        vec3 z = (eye - target).normalized();
        vec3 x = cross(up, z).normalized();
        vec3 y = cross(z, x);
#pragma omp parallel for
        for (int pix = 0; pix < width * height; pix++) {
            int pixel_x = pix % width;
            int pixel_y = pix / width;

            float dir_x = (pixel_x + 0.5) -  width / 2.;
            float dir_y = -(pixel_y + 0.5) + height / 2.;
            float dir_z = -height / (2. * tan(fov / 2.));

            vec3 dir_cam = vec3{dir_x, dir_y, dir_z}.normalized();
            vec3 ray_dir = (x * dir_cam.x + y * dir_cam.y + z * dir_cam.z).normalized();

            framebuffer[pix] = cast_ray(eye, ray_dir);
        }

        for (int i = 0; i < framebuffer.size(); i++) {
            float max = std::max(1.f, std::max(framebuffer[i][0], std::max(framebuffer[i][1], framebuffer[i][2])));
            float r = framebuffer[i][0] / max;
            float g = framebuffer[i][1] / max;
            float b = framebuffer[i][2] / max;

            rgb888[i * 3 + 0] = static_cast<uint8_t>(r * 255);
            rgb888[i * 3 + 1] = static_cast<uint8_t>(g * 255);
            rgb888[i * 3 + 2] = static_cast<uint8_t>(b * 255);
        }

        SDL_UpdateTexture(texture, nullptr, rgb888.data(), width * 3);
        SDL_RenderClear(renderer);
        SDL_RenderCopy(renderer, texture, nullptr, nullptr);
        SDL_RenderPresent(renderer);

        angle += 0.05f;
        SDL_PollEvent(nullptr);
    }

    auto end_time = std::chrono::steady_clock::now();
    float elapsed_sec = std::chrono::duration<float>(end_time - start_time).count();

    std::cout << "Rendered " << total_frames << " frames in " << elapsed_sec << " seconds.\n";
    std::cout << "Average FPS: " << (total_frames / elapsed_sec) << "\n";

    float pixel_fillrate = total_frames * width * height / elapsed_sec;

    if (pixel_fillrate > 1000) {
        std::cout << "Pixel Fillrate: " << pixel_fillrate / 1000 << " KPixel/s" << "\n";;
    } else if (pixel_fillrate > 1000000) {
        std::cout << "Pixel Fillrate: " << pixel_fillrate / 1000000 << " MPixel/s" << "\n";;
    } else if (pixel_fillrate > 1000000000) {
        std::cout << "Pixel Fillrate: " << pixel_fillrate / 1000000000 << " GPixel/s" << "\n";;
    } else {
        std::cout << "Pixel Fillrate: " << pixel_fillrate << " Pixel/s" << "\n";
    }

    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}

