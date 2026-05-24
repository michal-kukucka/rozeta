#include <rozeta/kinect.hpp>
#include <rozeta/obstacle_detection.hpp>

#include <iostream>
#include <stdexcept>
#include <string>

namespace {

void printUsage(const char* program) {
    std::cerr << "usage: " << program << " --sample <depth.csv> [threshold_m]\n";
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 3 || std::string(argv[1]) != "--sample") {
        printUsage(argv[0]);
        return 2;
    }

    const double threshold_m = argc >= 4 ? std::stod(argv[3]) : 1.5;
    try {
        const auto frame = rozeta::kinect::loadDepthCsv(argv[2]);
        const auto info = rozeta::obstacle_detection::fromDepthFrame(frame, threshold_m);

        std::cout << "depth_obstacle_console\n";
        std::cout << "nearest_m=" << info.nearestDistance << "\n";
        std::cout << "ahead=" << (info.obstacleAhead ? "true" : "false") << "\n";
        std::cout << "left=" << (info.obstacleLeft ? "true" : "false") << "\n";
        std::cout << "right=" << (info.obstacleRight ? "true" : "false") << "\n";
    } catch (const std::exception& error) {
        std::cerr << error.what() << "\n";
        return 1;
    }

    return 0;
}
