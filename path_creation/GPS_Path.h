#ifndef FLIGHTCONTROLLER_GPS_PATH_H
#define FLIGHTCONTROLLER_GPS_PATH_H

#include "libs.h"
#include "structs.h"
#include "Lines.h"

//class for building a path based on gps points
class GPS_Path {
private:
    std::vector<Vec3D> control_points;
    Line path_line;
    std::string files_path;
    int step_in_miters;

public:
    GPS_Path(const std::string &, const std::string &, PathType, int);

    //creates file, where gps data written in NMEA $GPRMC format: "ddmm.mmmm,N,dddmm.mmmm,M"
    void write_in_NMEA();

    //returns a list of three-dimensional points that describe the trajectory, compiled about the reference points
    std::vector<Vec3D> get_path() { return path_line.get_path(); }

    //returns a point such that the length of the path to it is the smallest, but not less than [m]
    Vec3D get_point(int m) { return control_points[(m+step_in_miters-1) / step_in_miters ]; }
};

#endif //FLIGHTCONTROLLER_GPS_PATH_H
