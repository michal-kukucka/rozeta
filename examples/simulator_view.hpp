#pragma once

// Optional live window for the simulator.
//
// Built only when ROZETA_WITH_SDL2=ON. Without it this header still compiles
// and every call is a no-op, so the headless simulator never depends on a
// graphics stack - it renders through ui::renderSceneSvg instead.
//
// The viewer deliberately owns no simulation state: it draws whatever
// ui::NavigationScene it is handed, the same structure the SVG renderer takes,
// so both views can never disagree about what a run looked like.

#include <rozeta/ui.hpp>

#include <string>

#ifdef ROZETA_WITH_SDL2
#include <SDL2/SDL.h>

#include <algorithm>
#include <cmath>
#include <vector>
#endif

namespace rozeta_examples {

class SimulatorView {
public:
    SimulatorView() = default;
    ~SimulatorView() { close(); }

    SimulatorView(const SimulatorView&) = delete;
    SimulatorView& operator=(const SimulatorView&) = delete;

#ifdef ROZETA_WITH_SDL2
    static constexpr bool kAvailable = true;

    /// Opens the window. Returns false with a message when SDL cannot start,
    /// which the caller should treat as "carry on headless", not as a failure.
    bool open(int width, int height, std::string& error) {
        if (window_ != nullptr) {
            return true;
        }
        if (SDL_Init(SDL_INIT_VIDEO) != 0) {
            error = SDL_GetError();
            return false;
        }
        window_ = SDL_CreateWindow(
            "rozeta simulator",
            SDL_WINDOWPOS_CENTERED,
            SDL_WINDOWPOS_CENTERED,
            width,
            height,
            SDL_WINDOW_SHOWN);
        if (window_ == nullptr) {
            error = SDL_GetError();
            SDL_Quit();
            return false;
        }
        renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
        if (renderer_ == nullptr) {
            renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_SOFTWARE);
        }
        if (renderer_ == nullptr) {
            error = SDL_GetError();
            close();
            return false;
        }
        width_ = width;
        height_ = height;
        return true;
    }

    bool isOpen() const { return renderer_ != nullptr; }

    /// Pumps events; returns false when the user asked to close the window.
    bool pumpEvents() {
        SDL_Event event;
        while (SDL_PollEvent(&event) != 0) {
            if (event.type == SDL_QUIT) {
                return false;
            }
            if (event.type == SDL_KEYDOWN &&
                (event.key.keysym.sym == SDLK_ESCAPE || event.key.keysym.sym == SDLK_q)) {
                return false;
            }
        }
        return true;
    }

    void draw(const rozeta::ui::NavigationScene& scene) {
        if (renderer_ == nullptr) {
            return;
        }
        const Projection projection(scene, width_, height_);

        SDL_SetRenderDrawColor(renderer_, 15, 23, 42, 255);
        SDL_RenderClear(renderer_);

        // Path network.
        SDL_SetRenderDrawColor(renderer_, 51, 65, 85, 255);
        for (const auto& edge : scene.graph.edges) {
            if (edge.from >= edge.to || edge.to >= scene.graph.vertices.size()) {
                continue;
            }
            drawLine(
                projection,
                scene.graph.vertices[edge.from].coordinate,
                scene.graph.vertices[edge.to].coordinate);
        }

        SDL_SetRenderDrawColor(renderer_, 56, 189, 248, 255);
        drawPolyline(projection, scene.route);
        SDL_SetRenderDrawColor(renderer_, 245, 158, 11, 255);
        drawPolyline(projection, scene.trajectory);

        if (scene.has_robot && !scene.lidar.empty()) {
            SDL_SetRenderDrawColor(renderer_, 34, 197, 94, 190);
            const auto robot = projection.project(scene.robot);
            for (const auto& point : scene.lidar) {
                if (!point.valid || !(point.distance_m > 0.0)) {
                    continue;
                }
                // Scan angles grow clockwise; screen y grows downwards.
                const double angle = scene.robot_heading_rad - point.angle_deg * kPi / 180.0;
                const double length = point.distance_m * projection.pixels_per_meter;
                SDL_RenderDrawLine(
                    renderer_,
                    static_cast<int>(robot.x),
                    static_cast<int>(robot.y),
                    static_cast<int>(robot.x + std::cos(angle) * length),
                    static_cast<int>(robot.y - std::sin(angle) * length));
            }
        }

        if (scene.has_start) {
            SDL_SetRenderDrawColor(renderer_, 74, 222, 128, 255);
            drawMarker(projection, scene.start, 5);
        }
        if (scene.has_goal) {
            SDL_SetRenderDrawColor(renderer_, 239, 68, 68, 255);
            drawMarker(projection, scene.goal, 5);
        }
        if (scene.has_gps) {
            SDL_SetRenderDrawColor(renderer_, 244, 114, 182, 255);
            drawMarker(projection, scene.gps_measurement, 3);
        }
        if (scene.has_robot) {
            SDL_SetRenderDrawColor(renderer_, 226, 232, 240, 255);
            const auto robot = projection.project(scene.robot);
            drawMarker(projection, scene.robot, 6);
            SDL_RenderDrawLine(
                renderer_,
                static_cast<int>(robot.x),
                static_cast<int>(robot.y),
                static_cast<int>(robot.x + std::cos(scene.robot_heading_rad) * 20.0),
                static_cast<int>(robot.y - std::sin(scene.robot_heading_rad) * 20.0));
        }

        // Drive bars: left and right side speed, signed, in the bottom corner.
        drawDriveBar(20, height_ - 40, scene.left_drive);
        drawDriveBar(20, height_ - 24, scene.right_drive);

        SDL_RenderPresent(renderer_);
    }

    void close() {
        if (renderer_ != nullptr) {
            SDL_DestroyRenderer(renderer_);
            renderer_ = nullptr;
        }
        if (window_ != nullptr) {
            SDL_DestroyWindow(window_);
            window_ = nullptr;
            SDL_Quit();
        }
    }

private:
    static constexpr double kPi = 3.141592653589793238462643383279502884;

    struct Point {
        double x{0.0};
        double y{0.0};
    };

    /// Same aspect-preserving projection the SVG renderer uses, so the window
    /// and the written file show the same picture.
    struct Projection {
        rozeta::GeoCoordinate center{};
        double pixels_per_meter{1.0};
        int width{0};
        int height{0};

        Projection(const rozeta::ui::NavigationScene& scene, int w, int h) : width(w), height(h) {
            const auto bounds = rozeta::ui::sceneBounds(scene);
            if (!bounds.valid) {
                return;
            }
            center = {
                (bounds.min.latitude + bounds.max.latitude) / 2.0,
                (bounds.min.longitude + bounds.max.longitude) / 2.0,
                0.0};
            const auto scale = rozeta::geodesy::metersPerDegree(center.latitude);
            const double span_x =
                std::max((bounds.max.longitude - bounds.min.longitude) * scale.longitude, 1.0);
            const double span_y =
                std::max((bounds.max.latitude - bounds.min.latitude) * scale.latitude, 1.0);
            pixels_per_meter =
                std::min((width - 80) / span_x, (height - 120) / span_y);
        }

        Point project(const rozeta::GeoCoordinate& point) const {
            const auto local = rozeta::geodesy::toLocalXy(center, point);
            return {
                width / 2.0 + local.x * pixels_per_meter,
                height / 2.0 - local.y * pixels_per_meter};
        }
    };

    void drawLine(
        const Projection& projection,
        const rozeta::GeoCoordinate& from,
        const rozeta::GeoCoordinate& to) {
        const auto a = projection.project(from);
        const auto b = projection.project(to);
        SDL_RenderDrawLine(
            renderer_,
            static_cast<int>(a.x),
            static_cast<int>(a.y),
            static_cast<int>(b.x),
            static_cast<int>(b.y));
    }

    void drawPolyline(
        const Projection& projection,
        const std::vector<rozeta::GeoCoordinate>& points) {
        for (std::size_t index = 1; index < points.size(); ++index) {
            drawLine(projection, points[index - 1], points[index]);
        }
    }

    void drawMarker(const Projection& projection, const rozeta::GeoCoordinate& point, int radius) {
        const auto screen = projection.project(point);
        SDL_Rect rect{
            static_cast<int>(screen.x) - radius,
            static_cast<int>(screen.y) - radius,
            radius * 2,
            radius * 2};
        SDL_RenderFillRect(renderer_, &rect);
    }

    void drawDriveBar(int x, int y, double value) {
        const int span = 120;
        SDL_SetRenderDrawColor(renderer_, 51, 65, 85, 255);
        SDL_Rect track{x, y, span, 10};
        SDL_RenderFillRect(renderer_, &track);

        const double clamped = std::max(-1.0, std::min(1.0, value));
        const int half = span / 2;
        const int length = static_cast<int>(std::fabs(clamped) * half);
        SDL_SetRenderDrawColor(
            renderer_, clamped >= 0.0 ? 34 : 239, clamped >= 0.0 ? 197 : 68, 94, 255);
        SDL_Rect bar{clamped >= 0.0 ? x + half : x + half - length, y, length, 10};
        SDL_RenderFillRect(renderer_, &bar);
    }

    SDL_Window* window_{nullptr};
    SDL_Renderer* renderer_{nullptr};
    int width_{1000};
    int height_{720};

#else  // ROZETA_WITH_SDL2

public:
    static constexpr bool kAvailable = false;

    bool open(int, int, std::string& error) {
        error = "this build has no window support (configure with -DROZETA_WITH_SDL2=ON)";
        return false;
    }
    bool isOpen() const { return false; }
    bool pumpEvents() { return true; }
    void draw(const rozeta::ui::NavigationScene&) {}
    void close() {}

#endif // ROZETA_WITH_SDL2
};

} // namespace rozeta_examples
