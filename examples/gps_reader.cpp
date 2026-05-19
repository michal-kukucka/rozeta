#include <iostream>
#include <rozeta/gps.hpp>
int main(){ rozeta::gps::NmeaParser parser; auto fix=parser.parseLine("$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"); std::cout << fix.latitude << "," << fix.longitude << " satellites=" << fix.satellite_count << "\n"; }
