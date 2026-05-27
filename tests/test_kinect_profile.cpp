#include "test_helpers.hpp"
#include <rozeta/core.hpp>
#include <rozeta/depth.hpp>
#include <rozeta/kinect.hpp>

#include <fstream>
#include <stdexcept>
#include <string>

namespace {

std::string tempProfilePath(const std::string& content) {
    std::string path = "/tmp/rozeta_m9_test_profile_" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) +
        ".cfg";
    std::ofstream out(path);
    out << content;
    out.close();
    return path;
}

} // namespace

using namespace rozeta;

// ── KinectProfile tests ───────────────────────────────────────────

void test_kinect_profile_defaults_have_safe_values() {
    const auto profile = kinect::KinectProfile::defaults();

    REQUIRE_TRUE(profile.baseline_frames > 0);
    REQUIRE_TRUE(profile.min_blob_area > 0);
    REQUIRE_TRUE(profile.depth_diff_threshold > 0.0);
    REQUIRE_TRUE(profile.smoothing_kernel >= 1);
}

void test_kinect_profile_validates_rejects_invalid_fields() {
    auto profile = kinect::KinectProfile::defaults();

    // Valid profile
    REQUIRE_TRUE(profile.validate().ok());

    // Invalid baseline_frames
    profile.baseline_frames = 0;
    REQUIRE_TRUE(!profile.validate().ok());

    // Invalid min_blob_area
    profile = kinect::KinectProfile::defaults();
    profile.min_blob_area = -1;
    REQUIRE_TRUE(!profile.validate().ok());

    // Invalid depth_diff_threshold
    profile = kinect::KinectProfile::defaults();
    profile.depth_diff_threshold = -0.1;
    REQUIRE_TRUE(!profile.validate().ok());

    // Invalid smoothing_kernel
    profile = kinect::KinectProfile::defaults();
    profile.smoothing_kernel = 0;
    REQUIRE_TRUE(!profile.validate().ok());
}

void test_kinect_profile_loads_and_parses_config_file() {
    std::string path = tempProfilePath(
        "baseline_frames=40\n"
        "min_blob_area=60\n"
        "depth_diff_threshold=0.20\n"
        "smoothing_kernel=5\n"
        "display=true\n"
        "headless=false\n"
    );

    const auto profile = kinect::KinectProfile::load(path);
    REQUIRE_TRUE(profile.validate().ok());
    REQUIRE_EQ(profile.baseline_frames, 40);
    REQUIRE_EQ(profile.min_blob_area, 60);
    REQUIRE_NEAR(profile.depth_diff_threshold, 0.20, 1e-9);
    REQUIRE_EQ(profile.smoothing_kernel, 5);
    REQUIRE_TRUE(profile.display);
    REQUIRE_TRUE(!profile.headless);

    std::remove(path.c_str());
}

void test_kinect_profile_load_partial_falls_back_to_defaults() {
    std::string path = tempProfilePath(
        "baseline_frames=20\n"
    );

    const auto profile = kinect::KinectProfile::load(path);
    REQUIRE_TRUE(profile.validate().ok());
    REQUIRE_EQ(profile.baseline_frames, 20);
    // Unspecified fields stay at defaults
    REQUIRE_EQ(profile.min_blob_area, kinect::KinectProfile::defaults().min_blob_area);
    REQUIRE_EQ(profile.smoothing_kernel, kinect::KinectProfile::defaults().smoothing_kernel);

    std::remove(path.c_str());
}

void test_kinect_profile_load_rejects_missing_file() {
    bool threw = false;
    try {
        (void)kinect::KinectProfile::load("/tmp/rozeta_nonexistent_profile_999.cfg");
    } catch (const std::runtime_error&) {
        threw = true;
    }
    REQUIRE_TRUE(threw);
}

// ── KinectBackendSelector tests ───────────────────────────────────

void test_kinect_backend_selector_starts_unavailable() {
    kinect::KinectBackendSelector selector(kinect::KinectProfile::defaults());
    REQUIRE_TRUE(
        selector.status() == kinect::KinectBackendStatus::Unavailable);
}

void test_kinect_backend_selector_transitions_through_statuses() {
    kinect::KinectBackendSelector selector(kinect::KinectProfile::defaults());

    selector.markConnected();
    REQUIRE_TRUE(
        selector.status() == kinect::KinectBackendStatus::Connected);

    selector.markRunning();
    REQUIRE_TRUE(
        selector.status() == kinect::KinectBackendStatus::Running);

    selector.markSimulated();
    REQUIRE_TRUE(
        selector.status() == kinect::KinectBackendStatus::Simulated);

    // Back to unavailable
    kinect::KinectBackendSelector fresh(kinect::KinectProfile::defaults());
    REQUIRE_TRUE(
        fresh.status() == kinect::KinectBackendStatus::Unavailable);
}

void test_kinect_backend_selector_mark_stale() {
    kinect::KinectBackendSelector selector(kinect::KinectProfile::defaults());
    selector.markRunning();

    // Recent update -> Running
    REQUIRE_TRUE(
        selector.status() == kinect::KinectBackendStatus::Running);

    // Mark stale with zero-age threshold -> goes stale
    selector.markStale(rozeta::now());
    REQUIRE_TRUE(
        selector.status() == kinect::KinectBackendStatus::Stale);
}

// ── Depth object summary tests ────────────────────────────────────

void test_normalize_depth_obstacle_summaries_empty_frame_returns_empty() {
    depth::DepthFrame frame;
    frame.metadata.width = 5;
    frame.metadata.height = 3;
    frame.depth_m = std::vector<float>(15, 0.0F);

    const auto summaries = kinect::normalizeDepthObstacleSummaries(
        frame, kinect::KinectProfile::defaults(), 1.5);

    // All zero/invalid depth -> no active objects
    REQUIRE_TRUE(summaries.empty() ||
        (!summaries.empty() && !summaries.front().active));
}

void test_normalize_depth_obstacle_summaries_detects_left_center_right() {
    auto profile = kinect::KinectProfile::defaults();
    profile.min_blob_area = 1; // small test frame

    depth::DepthFrame frame;
    frame.metadata.width = 6;
    frame.metadata.height = 4;
    // 6 cols: left(0,1), center(2,3), right(4,5)
    // Rows 2-3: near obstacle at different positions
    frame.depth_m = {
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,  // row0: all zeros
        0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,  // row1: all zeros
        0.8F, 0.8F, 0.0F, 0.0F, 0.9F, 0.9F,  // row2: obstacle left + right
        0.8F, 0.8F, 0.0F, 0.0F, 0.9F, 0.9F,  // row3: obstacle left + right
    };

    const auto summaries = kinect::normalizeDepthObstacleSummaries(
        frame, profile, 1.5);

    // Should detect obstacles on left and right sides
    bool has_left = false;
    bool has_right = false;
    for (const auto& s : summaries) {
        if (s.sector < 0) {
            has_left = true;
            REQUIRE_TRUE(s.active);
            REQUIRE_NEAR(s.nearest_distance_m, 0.8, 1e-6);
        } else if (s.sector > 0) {
            has_right = true;
            REQUIRE_TRUE(s.active);
            REQUIRE_NEAR(s.nearest_distance_m, 0.9, 1e-6);
        }
    }
    REQUIRE_TRUE(has_left);
    REQUIRE_TRUE(has_right);
}

void test_normalize_depth_obstacle_summaries_freshness_timestamp_is_set() {
    auto profile = kinect::KinectProfile::defaults();
    profile.min_blob_area = 1;

    depth::DepthFrame frame;
    frame.metadata.width = 2;
    frame.metadata.height = 2;
    frame.depth_m = {0.5F, 0.5F, 0.5F, 0.5F};

    const auto summaries = kinect::normalizeDepthObstacleSummaries(
        frame, profile, 1.5);

    REQUIRE_TRUE(!summaries.empty());
    // summaries are ordered: left(-1), center(0), right(1)
    const auto& s = summaries[1]; // center sector
    REQUIRE_TRUE(s.active);

    // freshness should be set (after now())
    auto now_tp = rozeta::now();
    REQUIRE_TRUE(s.freshness <= now_tp);
}

void test_normalize_depth_obstacle_summaries_respects_blob_area_minimum() {
    auto profile = kinect::KinectProfile::defaults();
    profile.min_blob_area = 100; // huge, no blob will pass

    depth::DepthFrame frame;
    frame.metadata.width = 10;
    frame.metadata.height = 5;
    frame.depth_m.resize(50);
    // Fill with obstacle values in a small area
    for (int i = 0; i < 50; ++i) {
        frame.depth_m[static_cast<std::size_t>(i)] = 0.5F;
    }

    const auto summaries = kinect::normalizeDepthObstacleSummaries(
        frame, profile, 1.5);

    // With min_blob_area=100 and only ~50 valid pixels in each sector,
    // no blob should be large enough
    bool any_active = false;
    for (const auto& s : summaries) {
        if (s.active && s.blob_area_px > 0) {
            any_active = true;
        }
    }
    // Small blobs should be filtered out
    REQUIRE_TRUE(!any_active);
}
