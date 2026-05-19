# Robotour use case

Rozeta is inspired by the Buchlovice/Robotour style workflow, but rebuilt as C/C++ modules for Linux-first robots.

## Autonomous loop

`examples/robotour_demo.cpp` demonstrates the intended sequence:

1. Read GPS.
2. Read odometry.
3. Read LiDAR and obstacle sensors.
4. Update robot state.
5. Make navigation decision.
6. Send motor commands.
7. Log information.

## Competition-oriented priorities

- safe stop behavior before clever navigation
- mock/demo mode for development without hardware
- structured logs for later replay and analysis
- clean replacement of sensor backends
- offline maps and waypoint route following prepared as next milestones

## Next milestones

1. Serial motor backend and hardware calibration tool.
2. YDLIDAR X4 backend.
3. GPS serial receiver and checksum validation.
4. Simplified offline OSM path loader.
5. Camera/OpenCV optional backend.
6. Kinect/libfreenect depth backend.
7. IMU fusion with odometry/GPS.
