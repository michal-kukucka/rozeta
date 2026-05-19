#include <iostream>
#include <rozeta/gps.hpp>
#include <rozeta/lidar.hpp>
#include <rozeta/logging.hpp>
#include <rozeta/motors.hpp>
#include <rozeta/navigation.hpp>
#include <rozeta/obstacle_detection.hpp>
#include <rozeta/odometry.hpp>
int main(){
    rozeta::logging::log(rozeta::logging::Level::Info, "robotour", "demo start");
    rozeta::gps::NmeaParser gps;
    auto fix = gps.parseLine("$GPRMC,092751.000,A,4800.0000,N,01700.0000,E,0.50,90.0,280511,,,A*43");
    rozeta::odometry::DifferentialOdometry odom({0.52, 1024, 0.25});
    auto pose = odom.updateTicks(64, 64);
    std::vector<rozeta::lidar::ScanPoint> scan{{0,1.8,true},{-35,0.9,true},{45,2.4,true}};
    auto obstacles = rozeta::obstacle_detection::fromLidar(scan, 1.0);
    rozeta::navigation::SimpleNavigator nav({0.20, 0.5, 0.8});
    auto decision = nav.goToWaypoint(pose, {3.0, 0.0, 0.0}, obstacles);
    rozeta::motors::MockMotorController motors;
    if(decision.emergency_stop) motors.emergencyStop(); else motors.setSpeed(decision.motor.left_speed, decision.motor.right_speed);
    std::cout << "GPS " << fix.latitude << "," << fix.longitude << " pose=" << pose.x << "," << pose.y << " decision=" << decision.reason << " motor=" << motors.lastCommand().left_speed << "," << motors.lastCommand().right_speed << "\n";
    rozeta::logging::log(rozeta::logging::Level::Info, "robotour", "demo end");
}
