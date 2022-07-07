#include "Lines.h"

void CatmullROM::build_curve2D() {
    double prev_h = points[0].H;

    for (int j = 0; j < points.size() - 1; j++) {
        int sampling = points[j].lenght(points[j + 1]) / path_step;
        auto step = float(1. / sampling);

        CubicPoly px{}, py{};
        CubicPoly pi{}, ph{};

        Vec3D p0, p3;
        if (!j)
            p0 = start_p;
        else
            p0 = points[j - 1];

        if (j == points.size() - 2)
            p3 = end_p;
        else
            p3 = points[j + 2];

        auto jj = static_cast<double>(j);
        InitCentripetalCR(Vec2D{jj, p0.H}, Vec2D{jj + 1, points[j].H}, Vec2D{jj + 2, points[j + 1].H},
                          Vec2D{jj + 3, p3.H}, pi, ph);
        InitCentripetalCR(p0.vec2D, points[j].vec2D, points[j + 1].vec2D, p3.vec2D, px, py);
        for (int i = 0; i <= sampling; ++i)
            cruve.emplace_back(px.eval(step * i), py.eval(step * i), ph.eval(step * i));
        prev_h = points[j + 1].H;
    }

    std::ofstream file;
    file.open("/home/crucian/CLionProjects/FlightController/log_CatmullROM.txt");
    for (auto &i: cruve)
        file << std::to_string(i.x()) << " " << std::to_string(i.y()) << " " << std::to_string(i.h()) << std::endl;
    file.close();
}

void CatmullROM::InitCentripetalCR(const Vec2D &p0, const Vec2D &p1, const Vec2D &p2, const Vec2D &p3,
                                   CubicPoly &px, CubicPoly &py) {
    float dt0 = powf(VecDistSquared(p0, p1), 0.25f);
    float dt1 = powf(VecDistSquared(p1, p2), 0.25f);
    float dt2 = powf(VecDistSquared(p2, p3), 0.25f);

    // safety check for repeated points
    if (dt1 < 1e-4f) dt1 = 1.0f;
    if (dt0 < 1e-4f) dt0 = dt1;
    if (dt2 < 1e-4f) dt2 = dt1;

    InitNonuniformCatmullRom(p0.x, p1.x, p2.x, p3.x, dt0, dt1, dt2, px);
    InitNonuniformCatmullRom(p0.y, p1.y, p2.y, p3.y, dt0, dt1, dt2, py);
}

void CatmullROM::InitNonuniformCatmullRom(float x0, float x1, float x2, float x3, float dt0, float dt1, float dt2, CubicPoly &p) {
// compute coefficients for a nonuniform Catmull-Rom spline
// compute tangents when parameterized in [t1,t2]
        float t1 = (x1 - x0) / dt0 - (x2 - x0) / (dt0 + dt1) + (x2 - x1) / dt1;
        float t2 = (x2 - x1) / dt1 - (x3 - x1) / (dt1 + dt2) + (x3 - x2) / dt2;

// rescale tangents for parametrization in [0,1]
        t1 *= dt1;
        t2 *= dt1;

        InitCubicPoly(x1, x2, t1, t2, p);
}
