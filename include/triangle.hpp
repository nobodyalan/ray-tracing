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
		normal = Vector3f::cross(vertices[1] - vertices[0], vertices[2] - vertices[0]).normalized();
	}
	void setNormal(const Vector3f &n)
	{
		normal = n;
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
			return true;
		}

		return false;
	}
	Vector3f getVertex(int i) const {
        return vertices[i];
    }

protected:
	Vector3f vertices[3];
	Vector3f normal;
};

#endif // TRIANGLE_H
