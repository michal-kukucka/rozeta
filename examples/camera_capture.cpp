#include <iostream>
#include <rozeta/camera.hpp>
int main(){ rozeta::camera::CameraConfig cfg; std::cout << "Camera interface skeleton: " << cfg.width << "x" << cfg.height << " @ " << cfg.fps << " fps\n"; }
