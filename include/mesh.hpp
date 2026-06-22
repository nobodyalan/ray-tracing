#ifndef MESH_H
#define MESH_H

#include <vector>
#include <algorithm>
#include "object3d.hpp"
#include "triangle.hpp"
#include "material.hpp"

static Vector3f vmax(const Vector3f& a, const Vector3f& b) {
    return Vector3f(std::max(a[0], b[0]), std::max(a[1], b[1]), std::max(a[2], b[2]));
}

static Vector3f vmin(const Vector3f& a, const Vector3f& b) {
    return Vector3f(std::min(a[0], b[0]), std::min(a[1], b[1]), std::min(a[2], b[2]));
}

// 💡 轴对齐包围盒 AABB 结构体
struct AABB {
    Vector3f min_p = Vector3f(1e30f, 1e30f, 1e30f);
    Vector3f max_p = Vector3f(-1e30f, -1e30f, -1e30f);

    AABB() {}
    AABB(const Vector3f& min_v, const Vector3f& max_v) : min_p(min_v), max_p(max_v) {}

    void integrate(const Vector3f& p) {
        min_p = vmin(min_p, p);
        max_p = vmax(max_p, p);
    }

    // 高精 Slab 算法：判定光线是否击中包围盒
    bool intersect(const Ray& r, float tmin, float tmax) const {
        Vector3f orig = r.getOrigin();
        Vector3f dir = r.getDirection();
        
        float t0 = tmin;
        float t1 = tmax;
        for (int i = 0; i < 3; ++i) {
            float invDir = 1.0f / (dir[i] == 0.0f ? 1e-6f : dir[i]);
            float tNear = (min_p[i] - orig[i]) * invDir;
            float tFar = (max_p[i] - orig[i]) * invDir;
            if (tNear > tFar) std::swap(tNear, tFar);
            
            t0 = tNear > t0 ? tNear : t0;
            t1 = tFar < t1 ? tFar : t1;
            if (t0 > t1) return false;
        }
        return true;
    }
};

// 💡 BVH 树节点结构
struct BVHNode {
    AABB box;
    BVHNode* left = nullptr;
    BVHNode* right = nullptr;
    Triangle* tri = nullptr; // 如果是叶子节点则指向三角形，否则为 nullptr

    ~BVHNode() {
        delete left;
        delete right;
    }
};

class Mesh : public Object3D {
public:
    Mesh(const char *filename, Material *m) {
        this->material = m;
        initialize(filename);
    }

    ~Mesh() {
        delete bvh_root;
        for (auto tri : t_faces) delete tri;
    }

    struct TriangleIndexed {
        int v[3];
    };

    std::vector<Vector3f> v;
    std::vector<TriangleIndexed> v_indices;
    std::vector<Triangle*> t_faces;
    std::vector<Vector2f> vt; 
    // 💡 新增：法线账本
    std::vector<Vector3f> vn;
    // 💡 修正：索引账本需要能同时记录 v, vt, vn 的索引
    // 因为 obj 文件中，一个顶点可能对应不同的纹理坐标或法线
    struct FaceIndex {
        int v_idx[3];
        int vt_idx[3];
        int vn_idx[3];
    };
    std::vector<FaceIndex> faces;
    BVHNode* bvh_root = nullptr; // BVH 根节点指针

    bool intersect(const Ray &r, Hit &h, float tmin) override;

private:
    void initialize(const char *filename);
    BVHNode* buildBVH(std::vector<Triangle*>& tris, int start, int end);
    bool intersectBVH(BVHNode* node, const Ray& r, Hit& h, float tmin) const;
};

#endif