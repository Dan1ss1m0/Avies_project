#include "GPS_Path.h"

std::string add_time(std::string s) {
    time_t now = time(0);
    std::string t = ctime(&now);
    return t + ' ' + s;
}
Log logs("/home/crucian/CLionProjects/FlightController/" + add_time("LOG")+".txt");

GPS_Path::GPS_Path(const std::string &f_p, const std::string &reading_file_name, PathType type, int sim = 1000) : step_in_miters(sim), files_path(f_p){
    logs.start_process("INIT GPS_Path");

    std::ifstream file(files_path+reading_file_name);
    std::string line;
    getline(file, line);
    while (getline(file, line)) {
        double x, y, h;
        std::istringstream(line) >> x >> y >> h;
        control_points.emplace_back(Vec3D{x, y, h});
    }
    file.close();

    logs.start_process("LINE BUILDING");
    time_t now = time(0);
    char* t = ctime(&now);

    if (type == PathType::curve)
        path_line = static_cast<Line>(CatmullROM{control_points, files_path+add_time("CatmellROM_path")+".txt", float(step_in_miters)});
    if (type == PathType::polyline)
        path_line = static_cast<Line>(Polyline{control_points, files_path+add_time("Polyline_path")+".txt", float(step_in_miters)});
    logs.finish_process();
    logs.finish_process();
};

void GPS_Path::write_in_NMEA() {


    std::ofstream file;
    file.open(files_path+add_time("GPS_text_NMEA")+".txt");
    logs.add_log("WRITING IN NMEA");

    if (!file.is_open()) {
        logs.add_log("ERROR: FileWriting error");
        return;
    }
    for (auto &control_point: control_points) {
        int N_int = int(control_point.x/ 1);
        std::string N = std::to_string(N_int) +
                        std::to_string((control_point.x- double(N_int)) * 60);
        int M_int = int(control_point.y/ 1);
        std::string M = std::to_string(M_int) +
                        std::to_string((control_point.y- double(M_int)) * 60);
        file << N << ",N," << M << ",M" << std::endl;
    }
    file.close();
    logs.finish_process();
}