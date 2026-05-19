#include <iostream>
#include <rozeta/odometry.hpp>
int main(){ rozeta::odometry::DifferentialOdometry odo({0.5,1024,0.25}); auto pose=odo.updateTicks(1024,1024); std::cout << "pose x=" << pose.x << " y=" << pose.y << " heading=" << pose.heading << "\n"; }
