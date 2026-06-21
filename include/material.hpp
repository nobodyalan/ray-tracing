#ifndef MATERIAL_H
#define MATERIAL_H

#include <cassert>
#include <vecmath.h>
#include "ray.hpp"
#include "hit.hpp"
#include <iostream>

class Material
{
public:
    explicit Material(const Vector3f &d_color, const Vector3f &s_color = Vector3f::ZERO, float s = 0,
                      const Vector3f &r_color = Vector3f::ZERO, float i = 1, const Vector3f &e_color = Vector3f::ZERO, 
                      float t = 1.0f, float _roughness = 0.3f) : 
                      diffuseColor(d_color), specularColor(s_color), shininess(s), 
                      reflectiveColor(r_color), indexOfRefraction(i), emissionColor(e_color), transparency(t), roughness(_roughness)
    {
    }
    
    virtual ~Material() = default;
    
    virtual float getTransparency() const { return transparency; }
    virtual Vector3f getEmission() const { return emissionColor; }
    virtual Vector3f getDiffuseColor() const { return diffuseColor; }
    virtual float getRoughness() const { return roughness; }
    virtual Vector3f getSpecularColor() const { return specularColor; }
    virtual float getShininess() const { return shininess; }
    virtual Vector3f getReflectiveColor() const { return reflectiveColor; }
    virtual float getRefractiveIndex() const { return indexOfRefraction; }

protected:
    Vector3f diffuseColor;
    Vector3f specularColor;
    float shininess;
    Vector3f reflectiveColor;
    float indexOfRefraction;
    Vector3f emissionColor;
    float transparency;
    float roughness;
};

#endif