#pragma once

#include "image.hpp"
#include <algorithm>
#include <queue>
#include <cstdio>

class Element
{
public:
    virtual void draw(Image &img) = 0;
    virtual ~Element() = default;
    bool checkValid(int x, int y, Image &img)
    {
        return x >= 0 && x < img.Width() && y >= 0 && y < img.Height();
    }
};

class Line : public Element
{

public:
    int xA, yA;
    int xB, yB;
    Vector3f color;
    void draw(Image &img) override
    {
        // TODO: Implement Bresenham Algorithm
        int x0 = xA, y0 = yA, x1 = xB, y1 = yB;
        if (x0 > x1)
        {
            std::swap(x0, x1);
            std::swap(y0, y1);
        }
        int dx = std::abs(x0 - x1);
        int dy = std::abs(y0 - y1);
        int sign = (y1 - y0) > 0 ? 1 : -1;
        if (dx == 0 && dy == 0)
        {
            img.SetPixel(xA, yA, color);
            return;
        }
        if (dx == 0)
        {
            if (y0 > y1)
                std::swap(y0, y1);
            for (int y = y0; y <= y1; ++y)
            {
                img.SetPixel(x0, y, color);
            }
            return;
        }
        if (dy == 0)
        {
            if (x0 > x1)
                std::swap(x0, x1);
            for (int x = x0; x <= x1; ++x)
            {
                img.SetPixel(x, y0, color);
            }
            return;
        }
        if (dy <= dx)
        {
            int bias = -dx;
            int add = 2 * dy;
            int y = y0;
            for (int x = x0; x <= x1; ++x)
            {
                img.SetPixel(x, y, color);
                bias += add;
                if (bias > 0)
                {
                    y += sign;
                    bias -= 2 * dx;
                }
            }
        }
        else
        {
            if (y0 > y1)
            {
                std::swap(x0, x1);
                std::swap(y0, y1);
            }
            int bias = -dy;
            int add = 2 * dx;
            int x = x0;
            for (int y = y0; y <= y1; ++y)
            {
                img.SetPixel(x, y, color);
                bias += add;
                if (bias > 0)
                {
                    x += sign;
                    bias -= 2 * dy;
                }
            }
        }

        printf("Draw a line from (%d, %d) to (%d, %d) using color (%f, %f, %f)\n", xA, yA, xB, yB,
               color.x(), color.y(), color.z());
    }
};

class Circle : public Element
{

public:
    int cx, cy;
    int radius;
    Vector3f color;
    void CirclePoints(Image &img, int x, int y)
    {
        img.SetPixel(cx + x, cy + y, color);
        img.SetPixel(cx - x, cy + y, color);
        img.SetPixel(cx + x, cy - y, color);
        img.SetPixel(cx - x, cy - y, color);
        img.SetPixel(cx + y, cy + x, color);
        img.SetPixel(cx - y, cy + x, color);
        img.SetPixel(cx + y, cy - x, color);
        img.SetPixel(cx - y, cy - x, color);
    }
    void draw(Image &img) override
    {
        int x = 0, y = radius;
        int d = 5 - 4 * radius;
        CirclePoints(img, x, y);
        while (x <= y)
        {
            if (d < 0)
            {
                d += 8 * x + 12;
            }
            else
            {
                d += 8 * (x - y) + 20;
                y--;
            }
            x++;
            CirclePoints(img, x, y);
        }
        printf("Draw a circle with center (%d, %d) and radius %d using color (%f, %f, %f)\n", cx, cy, radius,
               color.x(), color.y(), color.z());
    }
};

class Fill : public Element
{

public:
    int cx, cy;
    Vector3f color;
    void draw(Image &img) override
    {
        Vector3f oldColor = img.GetPixel(cx, cy);
        if (oldColor.x() == color.x() && oldColor.y() == color.y() && oldColor.z() == color.z())
        {
            return;
        }
        int xr, xl;
        std::queue<std::pair<int, int>> q;
        q.push({cx, cy});
        bool spanNeedFill = false;
        while (!q.empty())
        {
            std::pair<int, int> p = q.front();
            q.pop();
            int x = p.first, y = p.second;
            while (checkValid(x, y, img) && img.GetPixel(x, y) == oldColor)
            {
                img.SetPixel(x, y, color);
                ++x;
            }
            xr = x - 1;
            x = p.first - 1;
            while (checkValid(x, y, img) && img.GetPixel(x, y) == oldColor)
            {
                img.SetPixel(x, y, color);
                --x;
            }
            xl = x + 1;
            y = y + 1;
            x = xl;
            while (x <= xr)
            {
                if (y >= img.Height())
                {
                    break;
                }
                spanNeedFill = false;
                while (checkValid(x, y, img) && img.GetPixel(x, y) == oldColor)
                {
                    spanNeedFill = true;
                    x++;
                }
                if (spanNeedFill)
                {
                    q.push({x - 1, y});
                    spanNeedFill = false;
                }

                while (checkValid(x, y, img) && img.GetPixel(x, y) != oldColor && x <= xr)
                {
                    ++x;
                }
            }
            y = y - 2;
            x = xl;
            while (x <= xr)
            {
                if (y < 0)
                {
                    break;
                }
                spanNeedFill = false;
                while (checkValid(x, y, img) && img.GetPixel(x, y) == oldColor)
                {
                    spanNeedFill = true;
                    ++x;
                }
                if (spanNeedFill)
                {
                    q.push({x - 1, y});
                    spanNeedFill = false;
                }
                while (checkValid(x, y, img) && img.GetPixel(x, y) != oldColor && x <= xr)
                {
                    ++x;
                }
            }
        }
        // TODO: Flood fill
        printf("Flood fill source point = (%d, %d) using color (%f, %f, %f)\n", cx, cy,
               color.x(), color.y(), color.z());
    }
};