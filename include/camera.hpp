#ifndef CAMERA_H
#define CAMERA_H

#include "ray.hpp"
#include <vecmath.h>
#include <float.h>
#include <cmath>

class Camera
{
public:
    Camera(const Vector3f &center, const Vector3f &direction, const Vector3f &up, int imgW, int imgH)
    {
        this->center = center;
        this->direction = direction.normalized();
        this->horizontal = Vector3f::cross(this->direction, up).normalized();
        this->up = Vector3f::cross(this->horizontal, this->direction);
        this->width = imgW;
        this->height = imgH;
    }

    // Generate rays for each screen-space coordinate
    virtual Ray generateRay(const Vector2f &point) = 0;
    virtual ~Camera() = default;

    int getWidth() const { return width; }
    int getHeight() const { return height; }

protected:
    // Extrinsic parameters
    Vector3f center;
    Vector3f direction;
    Vector3f up;
    Vector3f horizontal;
    // Intrinsic parameters
    int width;
    int height;
};

class PerspectiveCamera : public Camera
{

public:
    PerspectiveCamera(const Vector3f &center, const Vector3f &direction,
                      const Vector3f &up, int imgW, int imgH, float angle) : Camera(center, direction, up, imgW, imgH)
    {
        // angle is in radian.
        float halfW = imgW / 2.0f;
        float halfH = imgH / 2.0f;
        fx = halfW / tan(angle / 2);
        fy = halfH / tan(angle / 2);
    }

    Ray generateRay(const Vector2f &point) override
    {
        // 按照公式: x = (u - cx) / fx, y = (cy - v) / fy, z = 1 (对应的底向量是 direction)
        // 旋转矩阵 R = [horizontal, -up, direction]
        // 因此 R * d_Rc = x * horizontal + y * (-up) + z * direction
        float x = (point.x() - width / 2.0f) / fx;
        float y = (height / 2.0f - point.y()) / fy;
        Vector3f dirRw = x * horizontal - y * up + direction;
        return Ray(center, dirRw.normalized());
    }
    float fx;
    float fy;
};

#endif // CAMERA_H
