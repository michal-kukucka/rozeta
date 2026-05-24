#include <rozeta/kinect.hpp>

#include <libfreenect.h>

#include <utility>

namespace rozeta::kinect {
namespace {

struct FreenectContextHandle {
    freenect_context* context{nullptr};

    ~FreenectContextHandle() {
        if (context != nullptr) {
            freenect_shutdown(context);
        }
    }
};

} // namespace

struct FreenectKinectSensor::Impl {
    int device_index{0};
    freenect_context* context{nullptr};
    freenect_device* device{nullptr};
};

Status probeFreenectRuntime() {
    FreenectContextHandle handle;
    if (freenect_init(&handle.context, nullptr) != 0) {
        return Status::error(ErrorCode::HardwareUnavailable, "unable to initialize libfreenect");
    }

    const int device_count = freenect_num_devices(handle.context);
    if (device_count <= 0) {
        return Status::error(ErrorCode::HardwareUnavailable, "no Kinect/libfreenect devices found");
    }

    return Status::okStatus();
}

FreenectKinectSensor::FreenectKinectSensor(int device_index)
    : impl_(std::make_unique<Impl>(Impl{device_index, nullptr, nullptr})) {}

FreenectKinectSensor::~FreenectKinectSensor() {
    close();
}

FreenectKinectSensor::FreenectKinectSensor(FreenectKinectSensor&& other) noexcept
    : impl_(std::move(other.impl_)) {}

FreenectKinectSensor& FreenectKinectSensor::operator=(FreenectKinectSensor&& other) noexcept {
    if (this != &other) {
        close();
        impl_ = std::move(other.impl_);
    }
    return *this;
}

Status FreenectKinectSensor::open() {
    if (impl_->device != nullptr) {
        return Status::okStatus();
    }

    if (freenect_init(&impl_->context, nullptr) != 0) {
        return Status::error(ErrorCode::HardwareUnavailable, "unable to initialize libfreenect");
    }
    freenect_select_subdevices(impl_->context,
                               static_cast<freenect_device_flags>(FREENECT_DEVICE_CAMERA));

    const int device_count = freenect_num_devices(impl_->context);
    if (device_count <= impl_->device_index) {
        close();
        return Status::error(ErrorCode::HardwareUnavailable, "requested Kinect device is unavailable");
    }

    if (freenect_open_device(impl_->context, &impl_->device, impl_->device_index) != 0) {
        close();
        return Status::error(ErrorCode::HardwareUnavailable, "unable to open Kinect device");
    }

    return Status::okStatus();
}

DepthFrame FreenectKinectSensor::depth() {
    DepthFrame frame;
    frame.metadata.width = 640;
    frame.metadata.height = 480;
    frame.metadata.fps = 30;
    return frame;
}

camera::Frame FreenectKinectSensor::rgb() {
    camera::Frame frame;
    frame.metadata.width = 640;
    frame.metadata.height = 480;
    frame.metadata.fps = 30;
    return frame;
}

void FreenectKinectSensor::close() {
    if (!impl_) {
        return;
    }
    if (impl_->device != nullptr) {
        freenect_close_device(impl_->device);
        impl_->device = nullptr;
    }
    if (impl_->context != nullptr) {
        freenect_shutdown(impl_->context);
        impl_->context = nullptr;
    }
}

} // namespace rozeta::kinect
