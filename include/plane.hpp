#ifndef PLANE_H
#define PLANE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>

// TODO: Implement Plane representing an infinite plane
// function: ax+by+cz=d
// choose your representation , add more fields and fill in the functions

class Plane : public Object3D {
public:
    Plane() {

    }

    Plane(const Vector3f &_normal, float _d, Material *_m) : Object3D(_m) {
        norm = _normal;
        d = _d;
    }

    ~Plane() override = default;

    bool intersect(const Ray &r, Hit &h, float tmin) override {
        float t;
        if (fabs(Vector3f::dot(norm, r.getDirection())) > 1e-6) {
            t = (d - Vector3f::dot(norm, r.getOrigin())) / Vector3f::dot(norm, r.getDirection());
            if (t > tmin && t < h.getT()) {
                h.set(t, material, norm);
                return true;
            }
        }
        return false;
    }

protected:
    Vector3f norm;
    float d;


};

#endif //PLANE_H
		

