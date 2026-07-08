#!/usr/bin/env python3
"""Portable OpenCV QR decoder stub smoke test.

This keeps the optional OpenCV QR guarded source buildable in default CI without
requiring OpenCV development packages or a Unix shell.
"""
from __future__ import annotations

import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

HEADER = r'''#pragma once
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
'''

SOURCE = r'''#include <rozeta/mission.hpp>
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
'''


def compiler_command(compiler: str, tmp: Path, output: Path) -> list[str]:
    include_root = ROOT / "include"
    stub_source = tmp / "opencv_qr_stub_test.cpp"
    mission_source = ROOT / "src" / "mission.cpp"
    name = Path(compiler).name.lower()
    if name in {"cl", "cl.exe"} or name.endswith("clang-cl.exe") or name == "clang-cl":
        return [
            compiler,
            "/nologo",
            "/EHsc",
            "/std:c++17",
            "/W4",
            "/DROZETA_WITH_OPENCV",
            f"/I{include_root}",
            f"/I{tmp}",
            str(mission_source),
            str(stub_source),
            f"/Fe:{output}",
        ]
    return [
        compiler,
        "-std=c++17",
        "-Wall",
        "-Wextra",
        "-Wpedantic",
        "-DROZETA_WITH_OPENCV",
        f"-I{include_root}",
        f"-I{tmp}",
        str(mission_source),
        str(stub_source),
        "-o",
        str(output),
    ]


def main() -> int:
    compiler = sys.argv[1] if len(sys.argv) > 1 else os.environ.get("CXX", "c++")
    if not shutil.which(compiler) and not Path(compiler).exists():
        print(f"C++ compiler not found: {compiler}", file=sys.stderr)
        return 1

    compiler_name = Path(compiler).name.lower()
    is_msvc = compiler_name in {"cl", "cl.exe"} or compiler_name.endswith("clang-cl.exe")
    if os.name == "nt" and is_msvc and not os.environ.get("INCLUDE"):
        print("Skipping OpenCV QR stub smoke: MSVC environment is not initialized")
        return 0

    suffix = ".exe" if os.name == "nt" else ""
    with tempfile.TemporaryDirectory(prefix="rozeta-opencv-qr-stub-") as tmp_name:
        tmp = Path(tmp_name)
        (tmp / "opencv2").mkdir(parents=True)
        (tmp / "opencv2" / "objdetect.hpp").write_text(HEADER, encoding="utf-8")
        (tmp / "opencv_qr_stub_test.cpp").write_text(SOURCE, encoding="utf-8")
        output = tmp / f"opencv_qr_stub_test{suffix}"
        subprocess.run(compiler_command(compiler, tmp, output), check=True)
        subprocess.run([str(output)], check=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
