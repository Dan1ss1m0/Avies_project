#ifndef FLIGHTCONTROLLER_STRUCTS_H
#define FLIGHTCONTROLLER_STRUCTS_H

#include "libs.h"
#include <ctime>

enum PathType {
    curve,
    polyline
};

//struct for calculating cubic polynomes (is used in CatmullROM building)
struct CubicPoly {
    double c0, c1, c2, c3;

    double eval(double t) const {
        double t2 = t * t;
        double t3 = t2 * t;
        return c0 + c1 * t + c2 * t2 + c3 * t3;
    }
};

//struct for working with a three-dimensional point
struct Vec3D {
    double x, y, h;

    Vec3D(double _x, double _y, double _h) : x(_x), y(_y), h(_h) {}

    Vec3D() {}

    static double similar_digit(double d) { return d * 0.99999 + 0.0001; }

    //(is used in creating the start- and end-points in CatmullROM building)
    Vec3D make_similar() const {
        return Vec3D{similar_digit(x), similar_digit(y), similar_digit(h)};
    }

    double lenght(Vec3D vec) const {
        double degree_to_rad = double(M_PI / 180);

        double d_lat = (x - vec.x) * degree_to_rad;
        double d_long = (y - vec.y) * degree_to_rad;

        double a =
                pow(sin(d_lat / 2), 2) + cos(x * degree_to_rad) * cos(vec.x * degree_to_rad) * pow(sin(d_long / 2), 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));
        double merid_circ_len = 6367;
        double km_to_m = 1000;
        double m = merid_circ_len * c * km_to_m;
        return m;
    }
};

//struct Log: writes logs to a file with a special structure
struct Log {
private:
    std::ofstream file;
    int shift = 0;
public:
    Log(std::string file_name) {
        file.open(file_name);
    }

    void add_log(std::string text) {
        for (int i = 0; i < shift; i++)
            file << "|\t";
        file << text << "\n";
    }

    void start_process(std::string text) {
        add_log(text);
        shift++;
    }

    void finish_process(std::string text = "finished successfully") {
        shift--;
        add_log(text);
    }

    ~Log() {
        file.close();
    }

};

std::string add_time(std::string s);

#endif //FLIGHTCONTROLLER_STRUCTS_H
