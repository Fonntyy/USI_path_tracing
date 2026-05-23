#ifndef Material_h
#define Material_h

#include "glm/glm.hpp"

enum class MaterialType
{
    Diffuse,
    Specular,
    Emissive
};

class Material
{
private:
    MaterialType type;
    glm::vec3 reflectivity;
    glm::vec3 emittance;
    float shininess;

    Material(MaterialType t, glm::vec3 r, glm::vec3 e, float s)
        : type(t), reflectivity(r), emittance(e), shininess(s)
    {
    }

public:
    Material() : type(MaterialType::Diffuse),
                 reflectivity(glm::vec3(0.5f)),
                 emittance(glm::vec3(0.0f)),
                 shininess(0.0f)
    {
    }

    static Material Diffuse(glm::vec3 reflectivity)
    {
        return Material(MaterialType::Diffuse, reflectivity, glm::vec3(0.0f), 0.0f);
    }

    static Material Emissive(glm::vec3 emittance)
    {
        return Material(MaterialType::Emissive, glm::vec3(0.0f), emittance, 0.0f);
    }

    static Material Specular(float shininess)
    {
        return Material(MaterialType::Specular, glm::vec3(0.0f), glm::vec3(0.0f), shininess);
    }

    MaterialType getMaterialType()
    {
        return type;
    }

    glm::vec3 getReflectivity()
    {
        return reflectivity;
    }

    glm::vec3 getEmittance()
    {
        return emittance;
    }

    float getShininess()
    {
        return shininess;
    }

    bool isDiffuse()
    {
        return type == MaterialType::Diffuse;
    }

    bool isEmissive()
    {
        return type == MaterialType::Emissive;
    }

    bool isSpecular()
    {
        return type == MaterialType::Specular;
    }
};

#endif /* Material_h */
