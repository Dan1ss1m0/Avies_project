#include "GPS_Path.h"


Log logs{};

int main() {
    logs.init("/home/crucian/CLionProjects/FlightController/" + add_time("LOG") + ".txt");
    GPS_Path GPS{"/home/crucian/CLionProjects/FlightController/", "GPS_text.txt", PathType::curve, 5, 5, 10};
}
