#ifndef FLIGHTCONTROLLER_GPS_PATH_H
#define FLIGHTCONTROLLER_GPS_PATH_H

#include "libs.h"
#include "structs.h"
#include "Lines.h"

class GPS_Path {
private:
    std::vector<Vec3D> control_points;
    Line path_line;
    int step_in_miters;

public:
    explicit GPS_Path(const std::string &, PathType, int);
    void write_in_NMEA(const std::string &);
    Vec3D get_point ( int m )
    {
                return control_points[m/step_in_miters];
    }
};

#endif //FLIGHTCONTROLLER_GPS_PATH_H
