#include <rozeta/camera.hpp>

#include <iostream>

int main() {
    rozeta::camera::CameraConfig config;
    std::cout << "Camera interface skeleton: "
              << config.width << "x" << config.height
              << " @ " << config.fps << " fps\n";
}
