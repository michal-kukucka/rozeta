#include <rozeta/camera.hpp>

#ifdef ROZETA_WITH_OPENCV

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

namespace rozeta::camera {

struct OpenCvCamera::Impl {
    cv::VideoCapture capture;
    CameraConfig config{};
    bool opened{false};
};

OpenCvCamera::OpenCvCamera()
    : impl_(std::make_unique<Impl>()) {}

OpenCvCamera::~OpenCvCamera() {
    close();
}

OpenCvCamera::OpenCvCamera(OpenCvCamera&&) noexcept = default;
OpenCvCamera& OpenCvCamera::operator=(OpenCvCamera&&) noexcept = default;

Status OpenCvCamera::open(const CameraConfig& config) {
    if (config.width <= 0 || config.height <= 0 || config.fps <= 0.0) {
        return Status::error(ErrorCode::InvalidArgument, "OpenCV camera config must use positive width, height and FPS");
    }

    close();
    impl_->config = config;

    if (!impl_->capture.open(config.device_index)) {
        return Status::error(ErrorCode::HardwareUnavailable, "OpenCV could not open camera device");
    }

    impl_->capture.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(config.width));
    impl_->capture.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(config.height));
    impl_->capture.set(cv::CAP_PROP_FPS, config.fps);
    impl_->opened = true;
    return Status::okStatus();
}

Frame OpenCvCamera::capture() {
    Frame frame;
    if (!impl_->opened) {
        return frame;
    }

    cv::Mat bgr;
    if (!impl_->capture.read(bgr) || bgr.empty()) {
        return frame;
    }

    cv::Mat contiguous_bgr;
    if (bgr.channels() == 3) {
        contiguous_bgr = bgr.isContinuous() ? bgr : bgr.clone();
    } else if (bgr.channels() == 4) {
        cv::cvtColor(bgr, contiguous_bgr, cv::COLOR_BGRA2BGR);
    } else if (bgr.channels() == 1) {
        cv::cvtColor(bgr, contiguous_bgr, cv::COLOR_GRAY2BGR);
    } else {
        return frame;
    }

    const auto byte_count = contiguous_bgr.total() * contiguous_bgr.elemSize();
    frame.bytes.assign(contiguous_bgr.data, contiguous_bgr.data + byte_count);
    frame.metadata.width = contiguous_bgr.cols;
    frame.metadata.height = contiguous_bgr.rows;
    frame.metadata.fps = impl_->config.fps;
    frame.metadata.timestamp = now();
    return frame;
}

void OpenCvCamera::close() {
    if (impl_ && impl_->capture.isOpened()) {
        impl_->capture.release();
    }
    if (impl_) {
        impl_->opened = false;
    }
}

} // namespace rozeta::camera

#endif
