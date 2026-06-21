#ifndef CUBE_H
#define CUBE_H

#include "object3d.hpp"
#include <vecmath.h>
#include <cmath>
#include <algorithm>

class Cube : public Object3D
{
public:
    Cube()
    {
        // 默认单位立方体，中心在原点
        min_p = Vector3f(-0.5f, -0.5f, -0.5f);
        max_p = Vector3f(0.5f, 0.5f, 0.5f);
    }

    Cube(const Vector3f &_min_p, const Vector3f &_max_p, Material *material) : Object3D(material)
    {
        min_p = _min_p;
        max_p = _max_p;
    }

    ~Cube() override = default;

    bool intersect(const Ray &r, Hit &h, float tmin) override
    {
        Vector3f O = r.getOrigin();
        Vector3f D = r.getDirection();

        float t_enter = -1e38f;
        float t_exit = 1e38f;
        Vector3f normal_enter = Vector3f::ZERO;

        // 分别对 X, Y, Z 三个轴向的 Slab（平行平面对）进行求交判定
        for (int i = 0; i < 3; ++i) {
            if (fabs(D[i]) < 1e-6f) {
                // 如果光线平行于当前的轴向平面
                if (O[i] < min_p[i] || O[i] > max_p[i]) {
                    return false; // 原点在平面外，绝无交点
                }
            } else {
                // 计算与当前轴向两个平面的交点距离 t1 和 t2
                float invD = 1.0f / D[i];
                float t1 = (min_p[i] - O[i]) * invD;
                float t2 = (max_p[i] - O[i]) * invD;

                // 确定进入和离开平面的方向
                float t_near = t1;
                float t_far = t2;
                float sign = -1.0f; // 标记法线方向的负号
                if (t1 > t2) {
                    std::swap(t_near, t_far);
                    sign = 1.0f; // 如果反向，法线朝向正方向
                }

                // 动态更新整条光线穿过长方体的公共交集区间 [t_enter, t_exit]
                if (t_near > t_enter) {
                    t_enter = t_near;
                    // 精准记录当前最新进入长方体时，撞击的是哪一个轴向的面，用以产生法线
                    normal_enter = Vector3f::ZERO;
                    normal_enter[i] = sign; 
                }
                if (t_far < t_exit) {
                    t_exit = t_far;
                }

                // 一旦进入时间大于离开时间，说明光线在三维空间中和各个平面“擦肩而过”，并未进入核心交集
                if (t_enter > t_exit) {
                    return false;
                }
            }
        }

        // 判定最近的有效交点是否落在当前射线合法区间 [tmin, hit.getT()) 内部
        if (t_enter >= tmin && t_enter < h.getT()) {
            h.set(t_enter, material, normal_enter.normalized());
            return true;
        }

        return false;
    }

protected:
    Vector3f min_p; // 长方体左下后对角点
    Vector3f max_p; // 长方体右上前对角点
};

#endif // CUBE_H