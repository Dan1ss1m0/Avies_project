#include "funcs.h"

//add current date and time to string as a start
std::string add_time(std::string s) {
    time_t now = time(0);
    std::string t = ctime(&now);
    return t + ' ' + s;
}

//calculate dh and dFi
ErrorPair calc_errors(Vec3D target_point, Vec3D curr_pos, Vec3D curr_angels, Vec2D null_yaw_vector) {
    Vec2D curr_tar{target_point.x - curr_pos.x, target_point.y - curr_pos.y};
    double Fi = acos((curr_tar.x * null_yaw_vector.x + curr_tar.y * null_yaw_vector.y) /
                     (curr_tar.len() * null_yaw_vector.len()));
    return ErrorPair{target_point.h - curr_pos.h,
                     Fi - curr_angels.x}; //in a connected frame of reference, the x-axis runs from the tail to the nose
}