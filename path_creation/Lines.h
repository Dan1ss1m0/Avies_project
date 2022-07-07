#ifndef FLIGHTCONTROLLER_LINES_H
#define FLIGHTCONTROLLER_LINES_H

#include "libs.h"
#include "structs.h"

class Line
{
protected:
    // длина шага цепи в метрах
    float path_step;
    std::vector<Vec3D> cruve;
    std::vector<Vec3D> points;
public:
    std::vector<Vec3D> get_path()
    {
        return points;
    }
};

class Polyline: public Line{
public:
    explicit Polyline(const std::vector<Vec3D>& points_, float ps=10) {
        points = points_;
        path_step = ps;

        if (points_.size() < 2) {
            throw "InitError: at least 2 points are required to construct the curve";
        }
        build();
    }
    Polyline () = default;

private:
    void build()
    {
        for (int j = 0; j < points.size() - 1; j++) {
            int sampling = points[j].lenght(points[j + 1]) / path_step;
            auto step = float(1. / sampling);

            double dx = (points[j + 1].vec2D.x - points[j].vec2D.x) / sampling;
            double dy = (points[j + 1].vec2D.y - points[j].vec2D.y) / sampling;
            double dh = (points[j + 1].H - points[j].H) / sampling;

            for (int i = 0; i <= sampling; ++i)
            {
                cruve.push_back( Vec3D{points[j].vec2D.x + i * dx, points[j].vec2D.y + i * dy, points[j].H + i * dh});
            }
        }


        std::ofstream file;
        file.open("/home/crucian/CLionProjects/FlightController/log_Polyline.txt");
        for (auto &i: cruve)
            file << std::to_string(i.x()) << " " << std::to_string(i.y()) << " " << std::to_string(i.h()) << std::endl;
        file.close();
    }
};

class CatmullROM: public Line{
public:
    explicit CatmullROM(const std::vector<Vec3D>& points_, float ps=10) : start_p(points_[0].make_similar()),
                                                                          end_p(points_[points_.size() - 1].make_similar()) {
        points = points_;
        path_step = ps;
        if (points_.size() < 2) {
            throw "InitError: at least 2 points are required to construct the curve";
        }
        build_curve2D();
    }
    CatmullROM () {}

private:
    Vec3D start_p, end_p;

    void build_curve2D();

    static void InitCubicPoly(float x0, float x1, float t0, float t1, CubicPoly &p) {
        p.c0 = x0;
        p.c1 = t0;
        p.c2 = -3 * x0 + 3 * x1 - 2 * t0 - t1;
        p.c3 = 2 * x0 - 2 * x1 + t0 + t1;
    }

    static void InitCatmullRom(float x0, float x1, float x2, float x3, CubicPoly &p) {
        // standard Catmull-Rom spline: interpolate between x1 and x2 with previous/following points x0/x3
        // (we don't need this here, but it's for illustration)
        // Catmull-Rom with tension 0.5
        InitCubicPoly(x1, x2, 0.5f * (x2 - x0), 0.5f * (x3 - x1), p);
    }

    static void InitNonuniformCatmullRom(float x0, float x1, float x2, float x3, float dt0, float dt1, float dt2, CubicPoly &p);

    static float VecDistSquared(const Vec2D &p, const Vec2D &q) {
        float dx = q.x - p.x;
        float dy = q.y - p.y;
        return dx * dx + dy * dy;
    }

    static void InitCentripetalCR(const Vec2D &p0, const Vec2D &p1, const Vec2D &p2, const Vec2D &p3, CubicPoly &px, CubicPoly &py);
};

#endif //FLIGHTCONTROLLER_LINES_H
