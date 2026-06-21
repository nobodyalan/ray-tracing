#ifndef GROUP_H
#define GROUP_H

#include "object3d.hpp"
#include "ray.hpp"
#include "hit.hpp"
#include <iostream>
#include <vector>

class Group : public Object3D
{

public:
    Group()
    {
        objects.clear();
    }

    explicit Group(int num_objects)
    {
        objects.clear();
        objects.resize(num_objects);
    }

    ~Group() override
    {
        for (auto obj : objects)
        {
            delete obj;
        }
    }

    bool intersect(const Ray &r, Hit &h, float tmin) override
    {
        bool intersected = false;
        for (auto obj : objects)
        {
            intersected = obj->intersect(r, h, tmin) || intersected;
        }
        return intersected;
    }

    void addObject(int index, Object3D *obj)
    {
        objects[index] = obj;
    }

    int getGroupSize()
    {
        return objects.size();
    }

protected:
    std::vector<Object3D *> objects;
};

#endif
