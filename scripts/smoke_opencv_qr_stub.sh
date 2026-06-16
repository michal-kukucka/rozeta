#!/usr/bin/env bash
set -euo pipefail

# Dependency-free behavior and syntax smoke for the optional OpenCV QR decoder.
# It stubs just the OpenCV objdetect surface used by src/mission.cpp so default
# CI catches guarded-source compile errors and decoder status behavior even when
# OpenCV development packages are absent.

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TMP="${TMPDIR:-/tmp}/rozeta-opencv-qr-stub.$$"
trap 'rm -rf "$TMP"' EXIT

mkdir -p "$TMP/opencv2"
cat > "$TMP/opencv2/objdetect.hpp" <<'HDR'
#pragma once
#include <string>

#define CV_8UC1 0

namespace cv {
inline std::string qr_stub_payload;

class Mat {
public:
    Mat(int rows, int cols, int type, void* data)
        : rows_(rows), cols_(cols), type_(type), data_(data) {}
private:
    int rows_;
    int cols_;
    int type_;
    void* data_;
};

class QRCodeDetector {
public:
    std::string detectAndDecode(const Mat&) { return qr_stub_payload; }
};
} // namespace cv
HDR

cat > "$TMP/opencv_qr_stub_test.cpp" <<'CPP'
#include <rozeta/mission.hpp>
#include <opencv2/objdetect.hpp>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

namespace {

bool require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        return false;
    }
    return true;
}

} // namespace

int main() {
    using rozeta::ErrorCode;
    using rozeta::mission::MissionTarget;
    using rozeta::mission::OpenCvQrDecoder;
    using rozeta::mission::QrImage;

    OpenCvQrDecoder decoder;
    std::string payload;

    QrImage bad_dimensions{0, 2, {}};
    auto status = decoder.decode(bad_dimensions, payload);
    if (!require(!status.ok() && status.code == ErrorCode::InvalidArgument,
                 "bad dimensions must fail closed")) {
        return 1;
    }

    QrImage bad_size{2, 2, {0, 1, 2}};
    status = decoder.decode(bad_size, payload);
    if (!require(!status.ok() && status.code == ErrorCode::InvalidArgument,
                 "bad grayscale size must fail closed")) {
        return 1;
    }

    QrImage image{2, 2, {0, 1, 2, 3}};
    cv::qr_stub_payload.clear();
    status = decoder.decode(image, payload);
    if (!require(!status.ok() && status.code == ErrorCode::ParseError,
                 "empty QR decode must return ParseError")) {
        return 1;
    }

    cv::qr_stub_payload = "geo:48.9,17.1";
    MissionTarget target;
    status = rozeta::mission::parseMissionTargetFromQr(image, decoder, target);
    if (!require(status.ok(), "valid QR payload must parse")) {
        return 1;
    }
    if (!require(std::fabs(target.coordinate.latitude - 48.9) < 1e-9,
                 "latitude mismatch")) {
        return 1;
    }
    if (!require(std::fabs(target.coordinate.longitude - 17.1) < 1e-9,
                 "longitude mismatch")) {
        return 1;
    }

    std::cout << "OpenCV QR stub smoke passed\n";
    return 0;
}
CPP

c++ -std=c++17 -Wall -Wextra -Wpedantic -DROZETA_WITH_OPENCV \
    -I"$ROOT/include" -I"$TMP" "$ROOT/src/mission.cpp" \
    "$TMP/opencv_qr_stub_test.cpp" -o "$TMP/opencv_qr_stub_test"
"$TMP/opencv_qr_stub_test"
