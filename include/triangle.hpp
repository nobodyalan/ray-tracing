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
		vertices[0] = a;
		vertices[1] = b;
		vertices[2] = c;
		uvs[0] = uvs[1] = uvs[2] = Vector2f::ZERO; // 默认初始化为0
		has_uv = false;                            // 标记没有材质贴图坐标
		normal = Vector3f::cross(vertices[1] - vertices[0], vertices[2] - vertices[0]).normalized();
	}

	// 💡 增量重载：支持接收 3 个顶点 UV 坐标的全新构造函数，供带有纹理贴图的高模网格使用
	Triangle(const Vector3f &a, const Vector3f &b, const Vector3f &c,
			 const Vector2f &uv0, const Vector2f &uv1, const Vector2f &uv2, Material *m) : Object3D(m)
	{
		vertices[0] = a;
		vertices[1] = b;
		vertices[2] = c;
		uvs[0] = uv0;
		uvs[1] = uv1;
		uvs[2] = uv2;
		has_uv = true;                             // 标记持有有效的纹理贴图坐标
		normal = Vector3f::cross(vertices[1] - vertices[0], vertices[2] - vertices[0]).normalized();
	}

	void setNormal(const Vector3f &n)
	{
		normal = n;
	}

	Vector3f getVertex(int idx) const
	{
		return vertices[idx];
	}

	bool intersect(const Ray &ray, Hit &hit, float tmin) override
	{
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

		// Calculate t
		float t = Vector3f::dot(E2, S2) * invDet;

		// Check if t is within valid range and closer than previous hits
		if (t > tmin && t < hit.getT())
		{
			// Normal orientation: optionally flip normal to face the ray
			Vector3f tempnormal = normal;
			if (Vector3f::dot(tempnormal, D) > 0)
			{
				tempnormal = -tempnormal;
			}
			hit.set(t, material, tempnormal);

			if (has_uv) {
				float w = 1.0f - u - v; // 计算第三个顶点的插值权重
				Vector2f interpolatedUV = w * uvs[0] + u * uvs[1] + v * uvs[2];
				hit.setUV(interpolatedUV); 
			}

			return true;
		}
		return false;
	}

protected:
	Vector3f vertices[3];
	Vector2f uvs[3];       // 💡 新增：存放三个顶点各自对应的 2D 纹理贴图坐标
	Vector3f normal;
	bool has_uv = false;   // 💡 新增：标记当前面片是否带有 UV
};
#endif // TRIANGLE_H
