#ifndef LIGHT_H
#define LIGHT_H

#include <Vector3f.h>
#include "object3d.hpp"
#include <cmath>
#include <algorithm>

class Light {
public:
    Light() = default;
    virtual ~Light() = default;

    virtual void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const = 0;

    virtual bool sampleNEE(const Vector3f &surfacePos, const Vector3f &surfaceNormal,
                           float u, float v,
                           Vector3f &dirToLight, Vector3f &radianceFactor, float &distance) const = 0;
};


class DirectionalLight : public Light {
public:
    DirectionalLight() = delete;
    DirectionalLight(const Vector3f &d, const Vector3f &c) {
        direction = d.normalized();
        color = c;
    }
    ~DirectionalLight() override = default;

    void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        dir = -direction;
        col = color;
    }

    // 💡 方向光 NEE 适配：无限远，方向恒定，无距离衰减
    bool sampleNEE(const Vector3f &surfacePos, const Vector3f &surfaceNormal,
                   float u, float v,
                   Vector3f &dirToLight, Vector3f &radianceFactor, float &distance) const override {
        dirToLight = -direction;
        distance = 1e5f; // 用一个足够远的值代表无限远，以便 shadowRay 求交边界覆盖
        radianceFactor = color;
        return true;
    }

private:
    Vector3f direction;
    Vector3f color;
};

class PointLight : public Light {
public:
    PointLight() = delete;
    PointLight(const Vector3f &p, const Vector3f &c) {
        position = p;
        color = c;
    }
    ~PointLight() override = default;

    void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        dir = (position - p);
        dir = dir / dir.length();
        col = color;
    }
    const Vector3f &getPosition() const { return position; }

    // 💡 点光源 NEE 适配：精准连线，自带 1/dist^2 物理距离衰减
    bool sampleNEE(const Vector3f &surfacePos, const Vector3f &surfaceNormal,
                   float u, float v,
                   Vector3f &dirToLight, Vector3f &radianceFactor, float &distance) const override {
        Vector3f toLight = position - surfacePos;
        distance = toLight.length();
        if (distance < 1e-4f) return false;
        
        dirToLight = toLight.normalized();
        // 根据大作业 4.3 节加分项的标准 PBR 拆分推导：点光源辐射度随距离平方衰减
        radianceFactor = color / (distance * distance); 
        return true;
    }

private:
    Vector3f position;
    Vector3f color;
};

class SpotLight : public Light {
public:
    SpotLight() = delete;
    SpotLight(const Vector3f &p, const Vector3f &d, const Vector3f &c, float innerAngleDeg, float outerAngleDeg) {
        position = p;
        direction = d.normalized();
        color = c;
        // 将角度转为弧度，并预存余弦值以在 NEE 中进行极速判定
        cosInner = cosf(innerAngleDeg * M_PI / 180.0f);
        cosOuter = cosf(outerAngleDeg * M_PI / 180.0f);
    }
    ~SpotLight() override = default;

    void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        dir = (position - p).normalized();
        col = color;
    }

    // 💡 聚光灯 NEE 适配：融合距离反比衰减与圆锥角余弦衰减
    bool sampleNEE(const Vector3f &surfacePos, const Vector3f &surfaceNormal,
                   float u, float v,
                   Vector3f &dirToLight, Vector3f &radianceFactor, float &distance) const override {
        Vector3f toLight = position - surfacePos;
        distance = toLight.length();
        if (distance < 1e-4f) return false;
        
        dirToLight = toLight.normalized();
        
        // 1. 计算当前光线发射方向（从光源指向着色点，即 -dirToLight）与聚光灯中轴线的夹角余弦
        float cosTheta = Vector3f::dot(-dirToLight, direction);
        
        // 2. 如果夹角已经超出了外圆锥角，说明完全照不到
        if (cosTheta < cosOuter) return false;
        
        // 3. 计算 Unity 标准的内外圆锥角平滑过渡衰减因子 (Smoothstep)
        float spotAttenuation = 1.0f;
        if (cosTheta < cosInner) {
            spotAttenuation = (cosTheta - cosOuter) / (cosInner - cosOuter);
            // 采用三次方插值平滑过渡
            spotAttenuation = spotAttenuation * spotAttenuation * (3.0f - 2.0f * spotAttenuation);
        }
        
        // 4. 融合距离反比衰减与聚光圆锥衰减
        radianceFactor = (color * spotAttenuation) / (distance * distance);
        return true;
    }

private:
    Vector3f position;
    Vector3f direction;
    Vector3f color;
    float cosInner;
    float cosOuter;
};

class AreaLight : public Light {
public:
    AreaLight(const Vector3f &p, const Vector3f &d1, const Vector3f &d2, const Vector3f &c) :
        position(p), dir1(d1), dir2(d2), color(c) {}

    void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        dir = (position - p);
        dir = dir / dir.length();
        col = color;
    }

    Vector3f getSamplePoint(float u, float v) const {
        return position + u * dir1 + v * dir2;
    }
    const Vector3f &getColor() const { return color; }

    // 💡 面光源 NEE 适配：完美整合面积、面光源法线与夹角余弦因子
    bool sampleNEE(const Vector3f &surfacePos, const Vector3f &surfaceNormal,
                   float u, float v,
                   Vector3f &dirToLight, Vector3f &radianceFactor, float &distance) const override {
        Vector3f lightPos = position + u * dir1 + v * dir2;
        Vector3f toLight = lightPos - surfacePos;
        distance = toLight.length();
        if (distance < 1e-4f) return false;
        
        dirToLight = toLight.normalized();
        
        Vector3f lightNormal = Vector3f::cross(dir1, dir2).normalized();
        float cosThetaY = std::abs(Vector3f::dot(lightNormal, -dirToLight));
        float area = Vector3f::cross(dir1, dir2).length();
        
        // 返回包含几何因子的完整面光源贡献
        radianceFactor = color * cosThetaY * area / (distance * distance);
        return true;
    }

private:
    Vector3f position;
    Vector3f dir1, dir2;
    Vector3f color;
};

#endif // LIGHT_H