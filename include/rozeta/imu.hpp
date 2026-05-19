#pragma once
#include <rozeta/core.hpp>
namespace rozeta::imu {
struct ImuSample { Vector3 accelerometer_mps2{}; Vector3 gyroscope_radps{}; Vector3 magnetometer_uT{}; double heading_rad{0}; Timestamp timestamp{now()}; };
class ImuSensor { public: virtual ~ImuSensor()=default; virtual Status open(const std::string& device)=0; virtual ImuSample read()=0; };
bool tiltDetected(const ImuSample& sample, double threshold_mps2=4.0);
bool collisionDetected(const ImuSample& sample, double threshold_mps2=25.0);
}