/**
@file main.cpp
*/

#include <iostream>
#include <fstream>
#include <cmath>
#include <complex>
#include <random>
#include <ctime>
#include <vector>
#include <tuple>
#include <iostream>
#include <vector>
#include <thread>
#include <fstream>
#include <chrono>

#include "glm/glm.hpp"
#include "glm/gtx/transform.hpp"

#include "Image.h"
#include "DepthImage.h"
#include "NormalImage.h"
#include "Material.h"

using namespace std;

thread_local std::mt19937 gen(std::random_device{}());
thread_local std::uniform_real_distribution<double> dist(0.0, 1.0);

/**
 Class representing a single ray.
 */
class Ray
{
public:
    glm::vec3 origin; ///< Origin of the ray
    glm::vec3 direction; ///< Direction of the ray
    /**
     Contructor of the ray
     @param origin Origin of the ray
     @param direction Direction of the ray
     */
    Ray(glm::vec3 origin, glm::vec3 direction) : origin(origin), direction(direction)
    {
    }
};

class Object;

/**
 Structure representing the even of hitting an object
 */
struct Hit
{
    bool hit; ///< Boolean indicating whether there was or there was no intersection with an object
    glm::vec3 normal; ///< Normal vector of the intersected object at the intersection point
    glm::vec3 intersection; ///< Point of Intersection
    float distance; ///< Distance from the origin of the ray to the intersection point
    Object* object; ///< A pointer to the intersected object
};

/**
 General class for the object
 */
class Object
{
protected:
    glm::mat4 transformationMatrix;
    ///< Matrix representing the transformation from the local to the global coordinate system
    glm::mat4 inverseTransformationMatrix;
    ///< Matrix representing the transformation from the global to the local coordinate system
    glm::mat4 normalMatrix; ///< Matrix for transforming normal vectors from the local to the global coordinate system

public:
    Material material; ///< Structure describing the material of the object
    /** A function computing an intersection, which returns the structure Hit */
    virtual Hit intersect(Ray ray) = 0;

    /** Function that returns the material struct of the object*/
    Material getMaterial()
    {
        return material;
    }

    /** Function that set the material
     @param material A structure describing the material of the object
    */
    void setMaterial(Material material)
    {
        this->material = material;
    }

    /** Functions for setting up all the transformation matrices
    @param matrix The matrix representing the transformation of the object in the global coordinates */
    void setTransformation(glm::mat4 matrix)
    {
        transformationMatrix = matrix;

        inverseTransformationMatrix = glm::inverse(matrix);
        normalMatrix = glm::transpose(inverseTransformationMatrix);
    }
};


/**
 Implementation of the class Object for sphere shape.
 */
class Sphere : public Object
{
private:
    float radius; ///< Radius of the sphere
    glm::vec3 center; ///< Center of the sphere

public:
    /**
     The constructor of the sphere
     @param radius Radius of the sphere
     @param center Center of the sphere
     */
    Sphere(float radius, glm::vec3 center, Material material) : radius(radius), center(center)
    {
        this->material = material;
    }

    /** Implementation of the intersection function*/
    Hit intersect(Ray ray)
    {
        glm::vec3 c = center - ray.origin;

        float cdotc = glm::dot(c, c);
        float cdotd = glm::dot(c, ray.direction);

        Hit hit;

        float D = 0;
        if (cdotc > cdotd * cdotd)
        {
            D = sqrt(cdotc - cdotd * cdotd);
        }
        if (D <= radius)
        {
            hit.hit = true;
            float t1 = cdotd - sqrt(radius * radius - D * D);
            float t2 = cdotd + sqrt(radius * radius - D * D);

            float t = t1;
            if (t < 0) t = t2;
            if (t < 0)
            {
                hit.hit = false;
                return hit;
            }

            hit.intersection = ray.origin + t * ray.direction;
            hit.normal = glm::normalize(hit.intersection - center);
            hit.distance = glm::distance(ray.origin, hit.intersection);
            hit.object = this;
        }
        else
        {
            hit.hit = false;
        }
        return hit;
    }
};


class Plane : public Object
{
private:
    glm::vec3 normal;
    glm::vec3 point;

public:
    Plane(glm::vec3 point, glm::vec3 normal) : point(point), normal(normal)
    {
    }

    Plane(glm::vec3 point, glm::vec3 normal, Material material) : point(point), normal(normal)
    {
        this->material = material;
    }

    Hit intersect(Ray ray)
    {
        Hit hit;
        hit.hit = false;

        /* Exercise 1 - Plane-ray intersection */
        auto d_n_dot = glm::dot(ray.direction, normal);
        if (d_n_dot == 0.0f)
        {
            return hit;
        }
        auto p = point - ray.origin;
        auto t = glm::dot(p, normal) / d_n_dot;
        if (t < 0)
        {
            return hit;
        }
        hit.hit = true;
        hit.distance = t;
        hit.intersection = t * ray.direction + ray.origin;
        hit.normal = d_n_dot < 0 ? normal : -normal;
        hit.normal = glm::normalize(hit.normal);
        hit.object = this;
        return hit;
    }
};

class Rectangle : public Object
{
private:
    // edge1 and edge2 are orthogonal
    glm::vec3 corner;
    glm::vec3 edge1;
    glm::vec3 edge2;
    glm::vec3 normal;
    Plane* plane;
    float edge1_dot;
    float edge2_dot;
    float area;

public:
    Rectangle(glm::vec3 corner, glm::vec3 edge1, glm::vec3 edge2, Material material)
        : corner(corner), edge1(edge1), edge2(edge2)
    {
        if (glm::dot(edge1, edge2) != 0)
        {
            throw std::invalid_argument("Edges have to be orthogonal.");
        }
        this->normal = glm::normalize(glm::cross(edge1, edge2));
        plane = new Plane(corner, this->normal, material);
        edge1_dot = glm::dot(edge1, edge1);
        edge2_dot = glm::dot(edge2, edge2);
        area = sqrt(edge1_dot) * sqrt(edge2_dot);
        this->material = material;
    }

    Hit intersect(Ray ray)
    {
        Hit hit = plane->intersect(ray);
        if (!hit.hit)
        {
            return hit;
        }
        glm::vec3 w = hit.intersection - corner;
        float u = glm::dot(w, edge1);
        float v = glm::dot(w, edge2);
        if (u < 0 || u > edge1_dot || v < 0 || v > edge2_dot)
        {
            hit.hit = false;
            return hit;
        }
        hit.object = this;
        return hit;
    }

    glm::vec3 random_point()
    {
        return corner + static_cast<float>(dist(gen)) * edge1 + static_cast<float>(dist(gen)) * edge2;
    }

    float get_area()
    {
        return area;
    }

    glm::vec3 get_normal()
    {
        return normal;
    }
};

class Cube : public Object
{
public:
    Cube() = default;

    Cube(Material material)
    {
        this->material = material;
    }

    Hit intersect(Ray ray)
    {
        Hit hit;
        Ray local_ray(
            inverseTransformationMatrix * glm::vec4(ray.origin, 1.0),
            inverseTransformationMatrix * glm::vec4(ray.direction, 0.0)
        );

        float t_x1 = (0 - local_ray.origin.x) / local_ray.direction.x;
        float t_x2 = (1 - local_ray.origin.x) / local_ray.direction.x;

        float t_min_x = min(t_x1, t_x2);
        float t_max_x = max(t_x1, t_x2);

        float t_y1 = (0 - local_ray.origin.y) / local_ray.direction.y;
        float t_y2 = (1 - local_ray.origin.y) / local_ray.direction.y;

        float t_min_y = min(t_y1, t_y2);
        float t_max_y = max(t_y1, t_y2);

        float t_z1 = (0 - local_ray.origin.z) / local_ray.direction.z;
        float t_z2 = (1 - local_ray.origin.z) / local_ray.direction.z;

        float t_min_z = min(t_z1, t_z2);
        float t_max_z = max(t_z1, t_z2);

        float t_enter = max(max(t_min_x, t_min_y), t_min_z);
        float t_exit = min(min(t_max_x, t_max_y), t_max_z);

        if (t_exit >= t_enter && t_exit >= 0.0f)
        {
            float t = t_enter >= 0.0f ? t_enter : t_exit;
            hit.hit = true;
            hit.object = this;
            glm::vec3 local_p = local_ray.origin + t * local_ray.direction;
            hit.intersection = transformationMatrix * glm::vec4(local_p, 1.0);
            int hit_axis = 0;
            if (t == t_x1 || t == t_x2) hit_axis = 0;
            else if (t == t_y1 || t == t_y2) hit_axis = 1;
            else hit_axis = 2;
            glm::vec3 normal(0.0f);
            if (hit_axis == 0) normal.x = -glm::sign(local_ray.direction.x);
            if (hit_axis == 1) normal.y = -glm::sign(local_ray.direction.y);
            if (hit_axis == 2) normal.z = -glm::sign(local_ray.direction.z);
            hit.normal = normalMatrix * glm::vec4(normal, 0.0);
            hit.normal = glm::normalize(hit.normal);
            hit.distance = glm::dot(hit.intersection - ray.origin, ray.direction)
                / glm::dot(ray.direction, ray.direction);
        }
        return hit;
    }
};


vector<Object*> objects; ///< A list of all objects in the scene
Rectangle* light;

glm::vec3 alignWithNormal(glm::vec3 sample, glm::vec3 normal)
{
    glm::vec3 up(0, 1, 0);
    float cosTheta = glm::dot(up, normal);
    if (cosTheta > 0.9999f)
    {
        return sample;
    }
    if (cosTheta < -0.9999f)
    {
        return -sample;
    }
    auto angle = glm::acos(cosTheta);
    auto axis = glm::cross(up, normal);
    glm::mat4 matrix = glm::rotate(angle, axis);
    return matrix * glm::vec4(sample, 0);
}


/**
 * Generates a point on the surface of a unit hemisphere centered around normal
 * with cosine-weighted distribution. Formula from https://graphicscompendium.com/raytracing/19-monte-carlo
 */
glm::vec3 cosWeightHemisphereSample(glm::vec3 normal)
{
    auto u = dist(gen);
    auto v = dist(gen);
    auto r = sqrt(u);
    auto theta = 2 * M_PI * v;
    glm::vec3 rawSamplePoint(glm::cos(theta) * r, sqrt(1 - u), glm::sin(theta) * r);
    return alignWithNormal(rawSamplePoint, normal);
}


/**
 * Generates a point on the surface of a unit hemisphere centered around normal
 * with power-cosine-weighted distribution.
 */
glm::vec3 powerCosWeightHemisphereSample(glm::vec3 normal, float power)
{
    auto e0 = dist(gen);
    auto e1 = dist(gen);
    auto theta = glm::acos(pow(e0, 1.0 / (power + 1)));
    auto phi = 2 * M_PI * e1;
    glm::vec3 rawSamplePoint(
        glm::cos(phi) * glm::sin(theta),
        glm::cos(theta),
        glm::sin(phi) * glm::sin(theta)
    );
    return alignWithNormal(rawSamplePoint, normal);
}


/**
 *
 * @param point
 * @param normal
 * @param object
 * @param reflected_dir
 * @return 0. color
 *         1. direction to light sample
 *         2. distance from point to light sample squared
 *         3. cos(theta) at the light sample
 */
tuple<glm::vec3, glm::vec3, float, float> directLight(glm::vec3 point, glm::vec3 normal, Object* object,
                                                      glm::vec3 reflected_dir)
{
    glm::vec3 light_point = light->random_point();
    glm::vec3 light_dir = glm::normalize(light_point - point);
    Ray light_ray(point + 0.001f * light_dir, light_dir);

    Hit light_ray_hit;
    light_ray_hit.hit = false;
    light_ray_hit.distance = INFINITY;
    for (auto& object : objects)
    {
        Hit hit = object->intersect(light_ray);
        if (hit.hit == true && hit.distance < light_ray_hit.distance)
            light_ray_hit = hit;
    }
    glm::vec3 illumination(0.0f);
    float light_distance_squared = pow(glm::length(light_point - point), 2);
    float cos_theta_prime = abs(glm::dot(-light->get_normal(), light_dir));
    if (light_ray_hit.object == light)
    {
        float cos_theta = max(0.0f, glm::dot(normal, light_dir)); // the cos/dot is sometimes negative
        glm::vec3 brdf;
        if (object->getMaterial().isDiffuse())
        {
            brdf = object->material.getReflectivity() * glm::vec3(1.0f / M_PI);
        }
        if (object->getMaterial().isSpecular())
        {
            brdf = glm::vec3(
                (object->getMaterial().getShininess() + 2) / (2 * M_PI)
                *
                pow(
                    max(0.0f, glm::dot(reflected_dir, light_dir)),
                    object->material.getShininess()
                )
            );
        }
        illumination = brdf
            * light->getMaterial().getEmittance()
            * light->get_area()
            * cos_theta
            * cos_theta_prime
            / light_distance_squared;
    }
    return make_tuple(
        illumination,
        light_dir,
        light_distance_squared,
        cos_theta_prime
    );
}


/**
 Functions that computes a color along the ray
 @param ray Ray that should be traced through the scene (it is required to have normalized direction).
 @param recursion_depth the depth of this call
 @return 0. Color at the intersection point,
        1. depth of the intersection point,
        2. surface normal at the intersection point,
 */
tuple<glm::vec3, float, glm::vec3> trace_ray(Ray ray, int recursion_depth)
{
    Hit closest_hit;
    closest_hit.hit = false;
    closest_hit.distance = INFINITY;
    closest_hit.normal = glm::vec3(0.0f);

    for (int k = 0; k < objects.size(); k++)
    {
        Hit hit = objects[k]->intersect(ray);
        if (hit.hit == true && hit.distance < closest_hit.distance)
            closest_hit = hit;
    }

    glm::vec3 color(0.0);
    if (closest_hit.hit)
    {
        if (closest_hit.object->material.isEmissive())
        {
            color = closest_hit.object->material.getEmittance();
        }
        else if (recursion_depth < 5)
        {
            glm::vec3 reflected_dir = glm::reflect(ray.direction, closest_hit.normal);

            // ----------- BRDF SAMPLING -------------------------------------------------------------
            glm::vec3 sample_pt;
            Ray indirect_ray(glm::vec3(0), glm::vec3(0));
            tuple<glm::vec3, float, glm::vec3> indirect_trace_ray_result;
            glm::vec3 indirect_light;
            if (closest_hit.object->material.isDiffuse())
            {
                sample_pt = cosWeightHemisphereSample(closest_hit.normal);
                indirect_ray = Ray(closest_hit.intersection + 0.001f * sample_pt, sample_pt);
                indirect_trace_ray_result = trace_ray(indirect_ray, recursion_depth + 1);
                indirect_light = closest_hit.object->material.getReflectivity() * get<0>(indirect_trace_ray_result);
            }
            if (closest_hit.object->material.isSpecular())
            {
                sample_pt = powerCosWeightHemisphereSample(
                    reflected_dir, closest_hit.object->material.getShininess());
                indirect_ray = Ray(closest_hit.intersection + 0.001f * sample_pt, sample_pt);
                indirect_trace_ray_result = trace_ray(indirect_ray, recursion_depth + 1);
                float indirect_cos_theta = max(0.0f, glm::dot(closest_hit.normal, sample_pt));
                indirect_light = glm::vec3(
                        indirect_cos_theta * (closest_hit.object->material.getShininess() + 2) / (closest_hit.object->
                            material.getShininess() + 1)
                    )
                    * get<0>(indirect_trace_ray_result);
            }

            // ----------- LIGHT SAMPLING -------------------------------------------------------------
            auto [direct_light, light_dir, direct_r_squared, direct_cos_theta_prime] =
                directLight(
                    closest_hit.intersection,
                    closest_hit.normal,
                    closest_hit.object,
                    reflected_dir
                );

            // ----------- MIS -------------------------------------------------------------
            if (direct_cos_theta_prime == 0.0f) // OBSERVED to happen
            {
                // don't do light sampling (and MIS) because direct ray is hitting light tangentially (bad sample)
                return make_tuple(indirect_light, closest_hit.distance, closest_hit.normal);
            }
            float p_light_direct = (1 / light->get_area()) * (direct_r_squared / direct_cos_theta_prime);
            float p_light_indirect = 0;
            Hit h = light->intersect(indirect_ray);
            if (h.hit)
            {
                // indirect ray is in direction of the light, so probability that light sampling generated the indirect
                // ray is not 0
                float indirect_r_squared = pow(h.distance, 2);
                float indirect_cos_theta_prime = glm::dot(h.normal, -indirect_ray.direction);
                // if (indirect_cos_theta_prime == 0.0f) // NOT observed to happen
                // {
                //     // don't do BRDF sampling (and MIS) because indirect ray is hitting light tangentially (bad sample)
                //     return make_tuple(direct_light, closest_hit.distance, closest_hit.normal);
                // }
                p_light_indirect = (1 / light->get_area()) * (indirect_r_squared / indirect_cos_theta_prime);
            }

            float p_brdf_direct;
            float p_brdf_indirect;
            if (closest_hit.object->material.isDiffuse())
            {
                float direct_cos_theta = max(0.0f, glm::dot(closest_hit.normal, light_dir));
                // cos/dot CAN be negative
                float indirect_cos_theta = glm::dot(closest_hit.normal, sample_pt);
                // sampling around normal => cos/dot never negative
                p_brdf_direct = direct_cos_theta / M_PI;
                p_brdf_indirect = indirect_cos_theta / M_PI;
            }
            if (closest_hit.object->material.isSpecular())
            {
                p_brdf_direct =
                    (closest_hit.object->getMaterial().getShininess() + 1)
                    * pow(
                        max(0.0f, glm::dot(reflected_dir, light_dir)),
                        // cos/dot CAN be negative
                        closest_hit.object->getMaterial().getShininess()
                    )
                    / (2.0f * M_PI);
                p_brdf_indirect =
                    (closest_hit.object->getMaterial().getShininess() + 1)
                    * pow(
                        glm::dot(reflected_dir, sample_pt),
                        // sampling around reflected_dir => cos/dot never negative
                        closest_hit.object->getMaterial().getShininess()
                    )
                    / (2.0f * M_PI);
            }

            float w_direct = p_light_direct / (p_light_direct + p_brdf_direct);
            float w_indirect = p_brdf_indirect / (p_brdf_indirect + p_light_indirect);
            color = w_direct * direct_light + w_indirect * indirect_light;
        }
    }
    return make_tuple(color, closest_hit.distance, closest_hit.normal);
}


/**
 Function defining the scene
 */
void sceneDefinition()
{
    Material light_material = Material::Emissive(glm::vec3(1.0f));
    Material green_material = Material::Diffuse(glm::vec3(0.12f, 0.45f, 0.15f));
    Material red_material = Material::Diffuse(glm::vec3(0.65f, 0.05f, 0.05f));
    Material white_material = Material::Diffuse(glm::vec3(0.73f, 0.73f, 0.73f));
    Material glossy_material = Material::Specular(10.0f);
    Material reflective_material = Material::Specular(200.0f);

    light = new Rectangle(glm::vec3(34.3, 55.4, 33.2), glm::vec3(-13.0, 0, 0), glm::vec3(0, 0, -10.5),
                          light_material);
    objects.push_back(light);
    objects.push_back(new Rectangle(glm::vec3(55.5, 0, 0), glm::vec3(0, 55.5, 0), glm::vec3(0, 0, 55.5),
                                    green_material));
    objects.push_back(new Rectangle(glm::vec3(0, 0, 0), glm::vec3(0, 55.5, 0), glm::vec3(0, 0, 55.5),
                                    red_material));
    objects.push_back(new Rectangle(glm::vec3(0, 0, 0), glm::vec3(55.5, 0, 0), glm::vec3(0, 0, 55.5),
                                    white_material));
    objects.push_back(new Rectangle(glm::vec3(55.5, 55.5, 55.5), glm::vec3(-55.5, 0, 0), glm::vec3(0, 0, -55.5),
                                    white_material));
    objects.push_back(new Rectangle(glm::vec3(0, 0, 55.5), glm::vec3(55.5, 0, 0), glm::vec3(0, 55.5, 0),
                                    white_material));

    auto sphere = new Sphere(10.0, glm::vec3(40.0, 10.0, 35.0), glossy_material);
    objects.push_back(sphere);

    auto sphere2 = new Sphere(10.0, glm::vec3(16.0, 10.0, 35.0), reflective_material);
    objects.push_back(sphere2);
}


glm::vec3 toneMapping(glm::vec3 intensity)
{
    float gamma = 1.0 / 2.0;
    float alpha = 12.0f;
    return glm::clamp(alpha * glm::pow(intensity, glm::vec3(gamma)), glm::vec3(0.0), glm::vec3(1.0));
}

tuple<glm::vec3, float, glm::vec3> pixel_trace_ray(Ray ray)
{
    int samples_per_pixel = 256;
    auto [color, depth, normal] = trace_ray(ray, 0);
    for (int i = 0; i < samples_per_pixel - 1; ++i)
    {
        color += get<0>(trace_ray(ray, 0));
    }
    color = color * glm::vec3(1.0f / samples_per_pixel);
    color = toneMapping(color);
    return make_tuple(color, depth, normal);
}


int width = 1024; //width of the image
int height = 1024; // height of the image
float fov = 40; // field of view
// precomputed values for pixel ray computation
float s = 2 * tan(0.5 * fov / 180 * M_PI) / width;
float X = s * width / 2;
float Y = s * height / 2;

void compute_range(Image& image, DepthImage& depth_image, NormalImage& normal_image, int startRow, int endRow)
{
    for (int j = startRow; j < endRow; ++j)
    {
        for (int i = 0; i < width; ++i)
        {
            float dx = X - i * s - s / 2;
            float dy = Y - j * s - s / 2;
            float dz = 1;

            glm::vec3 origin(27.8, 27.8, -80);
            glm::vec3 direction(dx, dy, dz);
            direction = glm::normalize(direction);

            Ray ray(origin, direction);
            auto [color, depth, normal] = pixel_trace_ray(ray);
            image.setPixel(i, j, color);
            depth_image.setPixel(i, j, depth);
            normal_image.setPixel(i, j, normal);
        }
    }
}

int main(int argc, const char* argv[])
{
    auto start = chrono::high_resolution_clock::now();

    sceneDefinition(); // Let's define a scene

    Image image(width, height); // Create an image where we will store the result
    DepthImage depth_image(width, height); // Create an image for result depth map
    NormalImage normal_image(width, height); // Create an image for result normal map

    int numThreads = thread::hardware_concurrency();
    vector<thread> threads;
    int rowsPerThread = height / numThreads;
    for (int i = 0; i < numThreads; ++i)
    {
        int startRow = i * rowsPerThread;
        int endRow = (i == numThreads - 1) ? height : startRow + rowsPerThread;
        threads.emplace_back(compute_range, ref(image), ref(depth_image), ref(normal_image), startRow, endRow);
    }

    for (auto& t : threads) t.join();

    auto end = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed = end - start;
    cout << "It took " << elapsed.count() << " seconds to render the image." << endl;
    cout << "I could render at " << 1.0 / elapsed.count() << " frames per second." << endl;

    // Writing the final results of the rendering
    if (argc == 2)
    {
        image.writeImage(argv[1]);
    }
    else
    {
        image.writeImage("./result.ppm");
    }
    depth_image.writeRaw("./result_depth.raw");
    normal_image.writeRaw("./result_normal.raw");

    return 0;
}
