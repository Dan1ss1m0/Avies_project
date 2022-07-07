#ifndef FLIGHTCONTROLLER_STRUCTS_H
#define FLIGHTCONTROLLER_STRUCTS_H

#include "libs.h"

enum PathType
{
    curve,
    polyline
};

struct CubicPoly {
    float c0, c1, c2, c3;

    float eval(float t) {
        float t2 = t * t;
        float t3 = t2 * t;
        return c0 + c1 * t + c2 * t2 + c3 * t3;
    }
};

struct Vec2D {
    Vec2D(double _x, double _y) : x(_x), y(_y) {}

    Vec2D() : x(0), y(0) {}

    double x, y;
};

struct Vec3D {
    Vec2D vec2D;
    double H;

    Vec3D(double _x, double _y, double _h) : vec2D(_x, _y), H(_h) {}
    Vec3D() : vec2D(0, 0), H(0) {}

    [[nodiscard]] double x() const { return vec2D.x; }

    [[nodiscard]] double y() const { return vec2D.y; }

    [[nodiscard]] double h() const { return H; }

    [[nodiscard]] Vec3D make_similar() const {
        return Vec3D{double(vec2D.x * 0.99 + 0.01), double(vec2D.y * 0.99 + 0.01), H * 0.99 + 0.01};
    }

    double lenght (Vec3D vec)
    {
        float len = cos(vec2D.y)*6360*2*3.14 / 360;
        return sqrt(pow((vec2D.x - vec.x())*111003, 2) + pow((vec2D.y - vec.y())*len, 2) + pow(H - vec.h(), 2));
    }
};

#endif //FLIGHTCONTROLLER_STRUCTS_H
