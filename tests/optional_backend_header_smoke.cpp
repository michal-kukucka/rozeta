#include <rozeta/camera.hpp>
#include <rozeta/kinect.hpp>
#include <rozeta/lidar.hpp>
#include <rozeta/mission.hpp>
#include <rozeta/ui.hpp>

#include <type_traits>

int main() {
#ifdef ROZETA_WITH_OPENCV
    static_assert(std::is_base_of<rozeta::camera::Camera, rozeta::camera::OpenCvCamera>::value,
                  "OpenCvCamera must remain a camera backend");
    static_assert(std::is_base_of<rozeta::mission::QrDecoder,
                                  rozeta::mission::OpenCvQrDecoder>::value,
                  "OpenCvQrDecoder must remain a QR decoder backend");
#endif

#ifdef ROZETA_WITH_KINECT
    static_assert(std::is_base_of<rozeta::kinect::KinectSensor,
                                  rozeta::kinect::FreenectKinectSensor>::value,
                  "FreenectKinectSensor must remain a Kinect backend");
#endif

#ifdef ROZETA_WITH_LDROBOT_LIDAR
    static_assert(std::is_base_of<rozeta::lidar::LidarScanner,
                                  rozeta::lidar::LdRobotLidarScanner>::value,
                  "LdRobotLidarScanner must remain a LiDAR backend");
#endif

    rozeta::ui::Viewport viewport{640, 480, 16};
    return viewport.width == 640 && viewport.height == 480 ? 0 : 1;
}
