#pragma once

#include <rozeta/core.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace rozeta::mission {

struct MissionTarget {
    GeoCoordinate coordinate{};
    std::string source_text{};
};

struct QrImage {
    int width{0};
    int height{0};
    std::vector<std::uint8_t> grayscale{};
};

class QrDecoder {
public:
    virtual ~QrDecoder() = default;
    virtual Status decode(const QrImage& image, std::string& payload) = 0;
};

Status parseMissionTarget(const std::string& payload, MissionTarget& target);
Status parseMissionTargetFromQr(
    const QrImage& image,
    QrDecoder& decoder,
    MissionTarget& target);

#ifdef ROZETA_WITH_OPENCV
class OpenCvQrDecoder final : public QrDecoder {
public:
    Status decode(const QrImage& image, std::string& payload) override;
};
#endif

} // namespace rozeta::mission
