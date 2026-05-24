#include <rozeta/camera.hpp>

#include <exception>
#include <iostream>
#include <string>
#include <vector>

namespace {

class MockCamera final : public rozeta::camera::Camera {
public:
    rozeta::Status open(const rozeta::camera::CameraConfig& config) override {
        config_ = config;
        opened_ = true;
        return rozeta::Status::okStatus();
    }

    rozeta::camera::Frame capture() override {
        rozeta::camera::Frame frame;
        if (!opened_) {
            return frame;
        }

        frame.metadata.width = config_.width;
        frame.metadata.height = config_.height;
        frame.metadata.fps = config_.fps;
        frame.metadata.timestamp = rozeta::now();
        frame.bytes.resize(rozeta::camera::expectedFrameByteSize(frame.metadata, 3, 1), 128);
        return frame;
    }

    void close() override {
        opened_ = false;
    }

private:
    rozeta::camera::CameraConfig config_{};
    bool opened_{false};
};

void printUsage(const char* program) {
    std::cout << "Usage: " << program << " [--mock|--opencv] [--device N] [--width W] [--height H]\n";
}

bool parseIntArg(int argc, char** argv, int& index, int& output) {
    if (index + 1 >= argc) {
        return false;
    }
    output = std::stoi(argv[++index]);
    return true;
}

int captureOneFrame(rozeta::camera::Camera& camera, const rozeta::camera::CameraConfig& config) {
    const auto open_status = camera.open(config);
    if (!open_status.ok()) {
        std::cerr << "Camera open failed: " << open_status.message << "\n";
        return 1;
    }

    const auto frame = camera.capture();
    camera.close();

    const auto frame_status = rozeta::camera::validateFrame(frame, 3, 1);
    if (!frame_status.ok()) {
        std::cerr << "Camera frame validation failed: " << frame_status.message << "\n";
        return 1;
    }

    std::cout << "Captured " << frame.metadata.width << "x" << frame.metadata.height
              << " BGR frame with " << frame.bytes.size() << " bytes at "
              << frame.metadata.fps << " fps\n";
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    bool use_mock = true;
    bool use_opencv = false;
    rozeta::camera::CameraConfig config;

    try {
        for (int index = 1; index < argc; ++index) {
            const std::string arg = argv[index];
            if (arg == "--mock") {
                use_mock = true;
                use_opencv = false;
            } else if (arg == "--opencv") {
                use_mock = false;
                use_opencv = true;
            } else if (arg == "--device" && parseIntArg(argc, argv, index, config.device_index)) {
            } else if (arg == "--width" && parseIntArg(argc, argv, index, config.width)) {
            } else if (arg == "--height" && parseIntArg(argc, argv, index, config.height)) {
            } else if (arg == "--help") {
                printUsage(argv[0]);
                return 0;
            } else {
                printUsage(argv[0]);
                return 1;
            }
        }
    } catch (const std::exception& error) {
        std::cerr << "Invalid argument: " << error.what() << "\n";
        return 1;
    }

    if (use_mock) {
        MockCamera camera;
        return captureOneFrame(camera, config);
    }

    if (use_opencv) {
#ifdef ROZETA_WITH_OPENCV
        rozeta::camera::OpenCvCamera camera;
        return captureOneFrame(camera, config);
#else
        std::cerr << "OpenCV camera backend is disabled. Reconfigure with -DROZETA_WITH_OPENCV=ON.\n";
        return 1;
#endif
    }

    printUsage(argv[0]);
    return 1;
}
