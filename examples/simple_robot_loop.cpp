#include <rozeta/gps.hpp>
#include <rozeta/lidar.hpp>
#include <rozeta/motors.hpp>
#include <rozeta/navigation.hpp>
#include <rozeta/obstacle_detection.hpp>
#include <rozeta/odometry.hpp>

#include <iostream>
#include <vector>

int main() {
    rozeta::motors::MockMotorController motors;
    rozeta::odometry::DifferentialOdometry odometry({0.5, 1024, 0.25});
    rozeta::Pose2D pose = odometry.updateTicks(128, 128);

    std::vector<rozeta::lidar::ScanPoint> scan{
        {0, 2.0, true},
        {45, 1.5, true},
    };
    auto obstacles = rozeta::obstacle_detection::fromLidar(scan, 1.0);

    rozeta::navigation::SimpleNavigator navigator;
    auto decision = navigator.goToWaypoint(pose, {5, 0, 0}, obstacles);
    if (decision.emergency_stop) {
        motors.emergencyStop();
    } else {
        motors.setSpeed(decision.motor.left_speed, decision.motor.right_speed);
    }

    std::cout << "decision=" << decision.reason
              << " left=" << motors.lastCommand().left_speed
              << " right=" << motors.lastCommand().right_speed << "\n";
}
