#include "GPS_Path.h"

GPS_Path::GPS_Path(const std::string &file_name, PathType type, int sim = 1000) : step_in_miters(sim) {
    std::ifstream file(file_name);
    std::string line;
    getline(file, line);

    LOG("INIT GPS_Path");
    while (getline(file, line)) {
        float x, y, h;
        std::istringstream(line) >> x >> y >> h;
        control_points.emplace_back(Vec3D{x, y, h});
    }
    file.close();

    LOG("LINE BUILDING");
    if (type == PathType::curve)
        path_line = static_cast<Line>(CatmullROM{control_points, float(step_in_miters)});
    if (type == PathType::polyline)
        path_line = static_cast<Line>(Polyline{control_points, float(step_in_miters)});
    FIN();
    FIN();
};

void GPS_Path::write_in_NMEA(const std::string &file_name) {
    std::ofstream file;
    file.open(file_name);
    LOG("WRITING IN NMEA");

    if (!file.is_open()) {
        std::cout << "FileWriting error\n";
        return;
    }
    for (auto &control_point: control_points) {
        int N_int = int(control_point.x() / 1);
        std::string N = std::to_string(N_int) +
                        std::to_string((control_point.x() - float(N_int)) * 60);
        int M_int = int(control_point.y() / 1);
        std::string M = std::to_string(M_int) +
                        std::to_string((control_point.y() - float(M_int)) * 60);
        file << N << ",N," << M << ",M" << std::endl;
    }
    file.close();
    FIN();
}