#ifndef MESH_H
#define MESH_H

#include <vector>
#include "object3d.hpp"
#include "triangle.hpp"
#include "material.hpp"

class Mesh : public Object3D {

public:
    // 💡 对齐构造函数
    Mesh(const char *filename, Material *m) {
        this->material = m;
        initialize(filename);
    }

    // 💡 对齐定义：让结构体名称和字段与 mesh.cpp 的 t.v[i] 严格一致
    struct TriangleIndexed {
        int v[3];
    };

    std::vector<Vector3f> v;
    std::vector<TriangleIndexed> v_indices; // 存储顶点索引
    std::vector<Triangle*> t_faces;         // 💡 新增对齐：存储真正实例化、带材质的三角形指针

    bool intersect(const Ray &r, Hit &h, float tmin) override;

private:
    void initialize(const char *filename);
};

#endif