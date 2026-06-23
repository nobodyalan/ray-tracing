#ifndef HIT_H
#define HIT_H

#include <vecmath.h>
#include "ray.hpp"

class Material;

class Hit {
public:

    // constructors
    Hit() {
        material = nullptr;
        t = 1e38;
        uv = Vector2f::ZERO; // 💡 增量初始化
    }

    Hit(float _t, Material *m, const Vector3f &n) {
        t = _t;
        material = m;
        normal = n;
        uv = Vector2f::ZERO; // 💡 增量初始化
    }

    Hit(const Hit &h) {
        t = h.t;
        material = h.material;
        normal = h.normal;
        uv = h.uv; // 💡 增量拷贝
    }

    // destructor
    ~Hit() = default;

    float getT() const {
        return t;
    }

    Material *getMaterial() const {
        return material;
    }

    const Vector3f &getNormal() const {
        return normal;
    }

    // 💡 新增：允许外界（Triangle求交时）记录插值出的 2D 纹理坐标
    void setUV(const Vector2f& _uv) { 
        uv = _uv; 
    }

    // 💡 新增：允许外界（着色器采样时）获取该交点处的 2D 纹理坐标
    Vector2f getUV() const { 
        return uv; 
    }

    void set(float _t, Material *m, const Vector3f &n) {
        t = _t;
        material = m;
        normal = n;
    }
public:
    void* getIntersectedObject() const { return object_ptr; }
    void setIntersectedObject(void* ptr) { object_ptr = ptr; }

// 确保在 Hit 构造函数或 reset 里面将其初始化为 nullptr：
    void reset() {
        t = 1e38;
        material = nullptr;
        object_ptr = nullptr; // 💡 新增初始化
    }
private:
    float t;
    Material *material;
    Vector3f normal;
    Vector2f uv; // 💡 新增：交点处的 UV 纹理坐标账本
    void* object_ptr = nullptr;
};

inline std::ostream &operator<<(std::ostream &os, const Hit &h) {
    os << "Hit <" << h.getT() << ", " << h.getNormal() << ">";
    return os;
}

#endif // HIT_H