#pragma once

#include <rozeta/core.hpp>

#include <cstdint>
#include <optional>
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

// ── M11 Robotour mission state machine ───────────────────────────

enum class RobotourPhase {
    ServiceStart,
    ToLoading,
    AtLoading,
    ToUnloading,
    AtUnloading,
    Returning,
    Complete,
    Aborted,
};

enum class MissionAck {
    ServiceComplete,
    LoadComplete,
    UnloadComplete,
};

enum class MissionEventType {
    PhaseChanged,
    ArrivedAtTarget,
    QrScanned,
    OperatorAcknowledged,
};

struct MissionEvent {
    MissionEventType type{MissionEventType::PhaseChanged};
    RobotourPhase phase{RobotourPhase::ServiceStart};
    int leg{0};
    GeoCoordinate position{};
    std::string detail{};
};

struct RobotourMissionConfig {
    GeoCoordinate loading_target{};
    GeoCoordinate unloading_target{};
    GeoCoordinate start_position{};
    double arrival_radius_m{3.0};
};

class RobotourMission {
public:
    explicit RobotourMission(RobotourMissionConfig config = {});

    RobotourPhase phase() const;
    int currentLeg() const;
    bool finished() const;
    GeoCoordinate currentTarget() const;
    const MissionTarget& loadingTarget() const;
    const MissionTarget& unloadingTarget() const;

    void updatePosition(GeoCoordinate position);
    void acknowledge(MissionAck ack);
    void abort();
    Status setLoadingTargetFromPayload(const std::string& payload);
    Status setUnloadingTargetFromPayload(const std::string& payload);
    std::optional<MissionEvent> pollEvent();
    void reset();

private:
    void pushEvent(MissionEvent event);
    void transition(RobotourPhase next);
    bool withinArrivalRadius(const GeoCoordinate& a, const GeoCoordinate& b) const;
    GeoCoordinate legTarget(int leg) const;

    RobotourMissionConfig config_;
    RobotourPhase phase_{RobotourPhase::ServiceStart};
    int leg_{0};
    MissionTarget loading_target_{};
    MissionTarget unloading_target_{};
    std::vector<MissionEvent> events_{};
    std::size_t event_head_{0};
};

} // namespace rozeta::mission
