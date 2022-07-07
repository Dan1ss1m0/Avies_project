#include "GPS_Path.h"


int main() {
    GPS_Path GPS{"/home/crucian/CLionProjects/FlightController/GPS_text.txt", PathType::polyline, 1000};
    GPS.write_in_NMEA("/home/crucian/CLionProjects/FlightController/GPS_text_NMEA.txt");
}