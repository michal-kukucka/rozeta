#include <iostream>
#include <rozeta/lidar.hpp>
int main(){ std::vector<rozeta::lidar::ScanPoint> points{{-40,0.7,true},{0,2.0,true},{35,0.9,true}}; std::cout << rozeta::lidar::renderConsoleScan(points) << "\n"; }
