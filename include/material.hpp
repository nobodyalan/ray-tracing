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
    
    // 💡 新增：默认构造函数，供解析 .mtl 时动态创建并用 set 赋值
    Material() {
        diffuseColor = Vector3f::ZERO;
        specularColor = Vector3f::ZERO;
        shininess = 0.0f;
        indexOfRefraction = 1.0f; // 绝缘体默认折射率为 1.0
        transparency = 0.0f;
        emissionColor = Vector3f::ZERO;
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

    void setDiffuseColor(const Vector3f &color) { diffuseColor = color; }
    void setSpecularColor(const Vector3f &color) { specularColor = color; }
    void setShininess(float s) { shininess = s; }
    void setRefractiveIndex(float r_idx) { indexOfRefraction = r_idx; }
    void setTransparency(float t) { transparency = t; }
    void setEmission(const Vector3f &e) { emissionColor = e; }

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