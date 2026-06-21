#ifndef SPHERE_H
#define SPHERE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>

class Sphere : public Object3D
{
public:
    Sphere()
    {
        center = Vector3f(0, 0, 0);
        radius = 1.0f;
    }

    Sphere(const Vector3f &_center, float _radius, Material *material) : Object3D(material)
    {
        center = _center;
        radius = _radius;
    }

    ~Sphere() override = default;

    bool intersect(const Ray &r, Hit &h, float tmin) override
    {
        Vector3f L = r.getOrigin() - center;
        Vector3f D = r.getDirection();
        
        float a = D.squaredLength();
        float b_prime = Vector3f::dot(D, L);
        float c = L.squaredLength() - radius * radius;

        float discriminant = b_prime * b_prime - a * c;

        if (discriminant < 0)
        {
            return false;
        }

        float sqrt_d = sqrtf(discriminant);

        float t = (-b_prime - sqrt_d) / a;
        if (t < tmin)
        {
            t = (-b_prime + sqrt_d) / a;
        }

        if (t >= tmin && t < h.getT())
        {
            Vector3f hitPoint = r.pointAtParameter(t);
            // 💡 严格遵循物理固有属性：法线永远由球心指向球表面（绝对朝外）
            Vector3f normal = (hitPoint - center).normalized();
            
            h.set(t, material, normal);
            return true;
        }

        return false;
    }

protected:
    Vector3f center;
    float radius;
};

#endif