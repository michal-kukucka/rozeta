#include <rozeta/odometry.hpp>

#include <iostream>

int main() {
    rozeta::odometry::DifferentialOdometry odometry({0.5, 1024, 0.25});
    auto pose = odometry.updateTicks(1024, 1024);

    std::cout << "pose x=" << pose.x
              << " y=" << pose.y
              << " heading=" << pose.heading << "\n";
}
