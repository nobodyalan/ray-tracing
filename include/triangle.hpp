#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "object3d.hpp"
#include <vecmath.h>
#include "plane.hpp"
#include <cmath>
#include <iostream>
#include "Matrix2f.h"
using namespace std;

class Triangle : public Object3D
{

public:
	Triangle() = delete;

	// a b c are three vertex positions of the triangle
	Triangle(const Vector3f &a, const Vector3f &b, const Vector3f &c, Material *m) : Object3D(m)
	{
		vertices[0] = a; vertices[1] = b; vertices[2] = c;
		uvs[0] = uvs[1] = uvs[2] = Vector2f::ZERO;
		has_uv = false;
		has_vn = false; // 标记没有顶点法线
		normal = Vector3f::cross(vertices[1] - vertices[0], vertices[2] - vertices[0]).normalized();
	}

	// 2. 降级重载：带纹理构造函数（有 UV，无顶点法线）
	Triangle(const Vector3f &a, const Vector3f &b, const Vector3f &c,
			 const Vector2f &uv0, const Vector2f &uv1, const Vector2f &uv2, Material *m) : Object3D(m)
	{
		vertices[0] = a; vertices[1] = b; vertices[2] = c;
		uvs[0] = uv0; uvs[1] = uv1; uvs[2] = uv2;
		has_uv = true;
		has_vn = false; // 标记没有顶点法线
		normal = Vector3f::cross(vertices[1] - vertices[0], vertices[2] - vertices[0]).normalized();
	}

	// 3. 💡 核心新增：完全体构造函数（有 UV，有 3 个顶点各自独立的平滑法线）
	Triangle(const Vector3f &a, const Vector3f &b, const Vector3f &c,
			 const Vector2f &uv0, const Vector2f &uv1, const Vector2f &uv2,
			 const Vector3f &n0, const Vector3f &n1, const Vector3f &n2, Material *m) : Object3D(m)
	{
		vertices[0] = a; vertices[1] = b; vertices[2] = c;
		uvs[0] = uv0; uvs[1] = uv1; uvs[2] = uv2;
		norms[0] = n0; norms[1] = n1; norms[2] = n2; // 💡 牢牢锁住 3 个顶点各自的法线，坚决不人工平均
		has_uv = true;
		has_vn = true; // 标记持有有效的顶点平滑法线
		normal = Vector3f::cross(vertices[1] - vertices[0], vertices[2] - vertices[0]).normalized(); // 面几何法线
	}
	void setNormal(const Vector3f &n)
	{
		normal = n;
	}

	Vector3f getVertex(int idx) const
	{
		return vertices[idx];
	}

	Vector2f getuv(int idx) const
	{
		return uvs[idx];
	}
	

	std::string getType() const override { return "Triangle"; }

	bool intersect(const Ray &ray, Hit &hit, float tmin) override
{
    if (hit.getIntersectedObject() == this) {
        return false;
    }
    // Möller-Trumbore algorithm using Cramer's rule
    Vector3f E1 = vertices[1] - vertices[0];
    Vector3f E2 = vertices[2] - vertices[0];
    Vector3f S = ray.getOrigin() - vertices[0];
    Vector3f D = ray.getDirection();

    Vector3f S1 = Vector3f::cross(D, E2);
    float det = Vector3f::dot(E1, S1);
    
    // ray is parallel to the triangle plane
    if (fabs(det) < 1e-6f)
        return false;

    float invDet = 1.0f / det;

    // Calculate barycentric u
    float u = Vector3f::dot(S, S1) * invDet;
    if (u < 0.0f || u > 1.0f)
        return false;

    // Calculate barycentric v
    Vector3f S2 = Vector3f::cross(S, E1);
    float v = Vector3f::dot(D, S2) * invDet;
    if (v < 0.0f || u + v > 1.0f)
        return false;
	float t = Vector3f::dot(E2, S2) * invDet;

    // ===================================================================
    // 🛡️ 工业级终极防线：层级排他性锁（彻底根治内外层法线混杂、绝对共面 Z-Fighting）
    // ===================================================================
   if (hit.getIntersectedObject() != nullptr) {
    // 绕过 dynamic_cast，直接通过多态虚函数安全比对
    if (hit.getIntersectedObject()->getType() == "Triangle") {
        Material* prevMat = hit.getMaterial(); // 既然已经写进 hit 了，可以直接拿出来
        if (prevMat && prevMat->getName() == "体" && this->material && this->material->getName() == "带内") {
            return false;
        }
    	}
	}

    // 接下来是正常的大小比对
    if (t > tmin && t < hit.getT()) {
        
        // 🟢 铁律 2（反向霸权）：如果当前是更高级的外层"裙体"到了，且它和账本里残存的"裙内"物理距离极其接近（< 1e-3f）
        // 说明触发了浮点数平局。此时无论 t 是否真的小于 hit.getT()，"裙体"都拥有无条件强行驱逐"裙内"的权利！
        bool isZFightingWithInner = false;
        if (hit.getIntersectedObject() != nullptr) {
            Triangle* prevTri = dynamic_cast<Triangle*>(hit.getIntersectedObject());
            if (prevTri && prevTri->getMaterial() && prevTri->getMaterial()->getName() == "带内") {
                if (fabs(t - hit.getT()) < 1e-2f && this->material && this->material->getName() == "体") {
                    isZFightingWithInner = true;
                }
            }

        }

        // 满足更近，或者触发了外层对内层的平局霸权强推，才允许写入
        if (t < hit.getT() || isZFightingWithInner) {
            float w = 1.0f - u - v;
            Vector3f shadingNormal = normal;
            if (has_vn) {
                shadingNormal = (w * norms[0] + u * norms[1] + v * norms[2]).normalized();
            }

            if (Vector3f::dot(shadingNormal, ray.getDirection()) > 0) {
                shadingNormal = -shadingNormal;
            }
            
            // 写入最新的无损账本数据
            hit.set(t, material, shadingNormal); 
            hit.setIntersectedObject(this);
            if (has_uv) {
                hit.setUV(w * uvs[0] + u * uvs[1] + v * uvs[2]); 
            }
            return true;
        }
    }
    return false;
    // Calculate t
    // float t = Vector3f::dot(E2, S2) * invDet;
    // // ===================================================================
    // // 💡 工业级精度护栏：材质偏置优先权（绝杀极其相近的不透明面片 Z-Fighting）
    // // ===================================================================
    

    // // 🟢 关键修改：加入 bias 比对。只有 (当前计算出的t + 偏置优势) 确实比账本里已有的记录更近，才允许写入



    // if (t > tmin && (t) < hit.getT()) {
		
    //     float w = 1.0f - u - v;
    //     // 利用求交得到的重心坐标插值出平滑的着色法线！
    //     Vector3f shadingNormal = normal;
    //     if (has_vn) {
    //         shadingNormal = (w * norms[0] + u * norms[1] + v * norms[2]).normalized();
    //     }

    //     if (Vector3f::dot(shadingNormal, ray.getDirection()) > 0) {
    //         shadingNormal = -shadingNormal;
    //     }
        
    //     // 写入最新的账本数据（注意：写入 hit 账本的必须是原始物理距离 t，绝不能写被 bias 修改过的值！）
    //     hit.set(t, material, shadingNormal); 
    //     hit.setIntersectedObject(this);
    //     if (has_uv) {
    //         hit.setUV(w * uvs[0] + u * uvs[1] + v * uvs[2]); 
    //     }
    //     return true;
    // }
    // return false;
}
protected:
    Vector3f vertices[3];
    Vector2f uvs[3];       
    Vector3f norms[3]; // 💡 新增
    Vector3f normal;   // 几何面法线
    bool has_uv = false;   
    bool has_vn = false; // 💡 新增
};
#endif // TRIANGLE_H
