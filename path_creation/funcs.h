#ifndef FLIGHTCONTROLLER_FUNCS_H
#define FLIGHTCONTROLLER_FUNCS_H

#include "structs.h"

std::string add_time(std::string);

ErrorPair calc_errors(Vec3D, Vec3D, Vec3D, Vec2D);


#endif //FLIGHTCONTROLLER_FUNCS_H
