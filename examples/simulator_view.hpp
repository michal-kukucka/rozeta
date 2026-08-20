#pragma once

// Optional live window for the simulator.
//
// Built only when ROZETA_WITH_SDL2=ON. Without it this header still compiles
// and every call is a no-op, so the headless simulator never depends on a
// graphics stack - it renders through ui::renderSceneSvg instead.
//
// The viewer deliberately owns no simulation state: it draws whatever
// ui::NavigationScene it is handed, the same structure the SVG renderer takes,
// so both views can never disagree about what a run looked like. Operator
// input follows the same rule: poll() reports what was pressed or clicked and
// decides nothing, so the application stays the only thing that knows what a
// command means.

#include <rozeta/geodesy.hpp>
#include <rozeta/ui.hpp>

#include <cstdint>
#include <string>
#include <vector>

#ifdef ROZETA_WITH_SDL2
#include <SDL2/SDL.h>

#include <algorithm>
#include <cmath>
#endif

namespace rozeta_examples {

/// What the operator asked for. The viewer names the intent; the application
/// decides whether it is possible and what it does.
enum class ViewCommand {
    Quit,
    EmergencyStop,   ///< Latch or clear the physical E-STOP.
    TogglePause,
    Replan,          ///< Re-plan from the robot's position to the current goal.
    ClearPoints,     ///< Forget an operator-picked start/goal.
    ToggleDrive,
    TogglePosition,
    ToggleRanging,
    ToggleCamera,
    CycleDriveBackend,
    CyclePositionBackend,
    CycleRangingBackend,
    StartRun,
    Abort,
    MarkEvent,
    ToggleRecording,
    SpeedUp,
    SlowDown,
    ToggleHelp,
};

/// One frame of operator input.
struct ViewInput {
    std::vector<ViewCommand> commands{};
    bool has_click{false};
    /// Where the click landed, unprojected through the last drawn frame.
    rozeta::GeoCoordinate click{};
    /// Left button picks the start, right button picks the destination.
    bool click_picks_goal{false};
};

/// A labelled readout drawn over the map.
struct ViewStatusLine {
    std::string label{};
    std::string value{};
    /// 0 neutral, 1 good/connected, 2 warning, 3 fault.
    int severity{0};
};

/// Everything the window draws that is not the scene itself.
struct ViewOverlay {
    std::vector<ViewStatusLine> status{};
    std::vector<std::string> keys{};   ///< Key hints, shown when help is on.
    std::string toast{};               ///< Transient message, e.g. what a key did.
    bool show_help{false};
    bool paused{false};
    bool estop{false};
    /// Operator-picked points, drawn as pending until they are planned.
    bool has_pending_start{false};
    rozeta::GeoCoordinate pending_start{};
    bool has_pending_goal{false};
    rozeta::GeoCoordinate pending_goal{};
};

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
        ViewInput ignored;
        return poll(ignored);
    }

    /// Pumps events and reports what the operator asked for.
    ///
    /// Returns false when the window should close, which is also reported as
    /// ViewCommand::Quit so a caller that only reads the command list does not
    /// miss it. A click is unprojected through the last frame that was drawn,
    /// so it means the place on the map the operator was actually looking at.
    bool poll(ViewInput& out) {
        out = ViewInput{};
        bool open = true;
        SDL_Event event;
        while (SDL_PollEvent(&event) != 0) {
            if (event.type == SDL_QUIT) {
                out.commands.push_back(ViewCommand::Quit);
                open = false;
                continue;
            }
            if (event.type == SDL_MOUSEBUTTONDOWN && has_projection_) {
                if (event.button.button == SDL_BUTTON_LEFT ||
                    event.button.button == SDL_BUTTON_RIGHT) {
                    out.has_click = true;
                    out.click_picks_goal = event.button.button == SDL_BUTTON_RIGHT;
                    out.click = last_projection_.unproject(
                        static_cast<double>(event.button.x),
                        static_cast<double>(event.button.y));
                }
                continue;
            }
            if (event.type != SDL_KEYDOWN) {
                continue;
            }
            switch (event.key.keysym.sym) {
                case SDLK_ESCAPE:
                case SDLK_q:
                    out.commands.push_back(ViewCommand::Quit);
                    open = false;
                    break;
                case SDLK_SPACE: out.commands.push_back(ViewCommand::EmergencyStop); break;
                case SDLK_p: out.commands.push_back(ViewCommand::TogglePause); break;
                case SDLK_r: out.commands.push_back(ViewCommand::Replan); break;
                case SDLK_x: out.commands.push_back(ViewCommand::ClearPoints); break;
                case SDLK_m: out.commands.push_back(ViewCommand::ToggleDrive); break;
                case SDLK_g: out.commands.push_back(ViewCommand::TogglePosition); break;
                case SDLK_l: out.commands.push_back(ViewCommand::ToggleRanging); break;
                case SDLK_c: out.commands.push_back(ViewCommand::ToggleCamera); break;
                case SDLK_1: out.commands.push_back(ViewCommand::CycleDriveBackend); break;
                case SDLK_2: out.commands.push_back(ViewCommand::CyclePositionBackend); break;
                case SDLK_3: out.commands.push_back(ViewCommand::CycleRangingBackend); break;
                case SDLK_s: out.commands.push_back(ViewCommand::StartRun); break;
                case SDLK_a: out.commands.push_back(ViewCommand::Abort); break;
                case SDLK_e: out.commands.push_back(ViewCommand::MarkEvent); break;
                case SDLK_t: out.commands.push_back(ViewCommand::ToggleRecording); break;
                case SDLK_h:
                case SDLK_SLASH: out.commands.push_back(ViewCommand::ToggleHelp); break;
                case SDLK_EQUALS:
                case SDLK_PLUS: out.commands.push_back(ViewCommand::SpeedUp); break;
                case SDLK_MINUS: out.commands.push_back(ViewCommand::SlowDown); break;
                default: break;
            }
        }
        return open;
    }

    void draw(const rozeta::ui::NavigationScene& scene) {
        draw(scene, ViewOverlay{});
    }

    void draw(const rozeta::ui::NavigationScene& scene, const ViewOverlay& overlay) {
        if (renderer_ == nullptr) {
            return;
        }
        const Projection projection(scene, width_, height_);
        // Remembered so a click in the next frame can be turned back into the
        // coordinate the operator was pointing at.
        last_projection_ = projection;
        has_projection_ = true;

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

        // Operator-picked points that have not been planned yet, drawn hollow
        // so they cannot be mistaken for the route the robot is following.
        if (overlay.has_pending_start) {
            SDL_SetRenderDrawColor(renderer_, 74, 222, 128, 255);
            drawRing(projection, overlay.pending_start, 9);
        }
        if (overlay.has_pending_goal) {
            SDL_SetRenderDrawColor(renderer_, 239, 68, 68, 255);
            drawRing(projection, overlay.pending_goal, 9);
        }

        // Drive bars: left and right side speed, signed, in the bottom corner.
        drawDriveBar(20, height_ - 40, scene.left_drive);
        drawDriveBar(20, height_ - 24, scene.right_drive);
        SDL_SetRenderDrawColor(renderer_, 148, 163, 184, 255);
        drawText(150, height_ - 41, "L", 1);
        drawText(150, height_ - 25, "R", 1);

        drawOverlay(overlay);

        SDL_RenderPresent(renderer_);
    }

    /// Holds the last frame until the operator closes the window.
    ///
    /// Redrawing in a bare `while (pumpEvents())` loop pins a core for as long
    /// as the window stays open, so the wait is paced to roughly 60 Hz: the
    /// picture is static, and nothing else in the process needs the CPU.
    void waitForClose(const rozeta::ui::NavigationScene& scene) {
        waitForClose(scene, ViewOverlay{});
    }

    void waitForClose(const rozeta::ui::NavigationScene& scene, ViewOverlay overlay) {
        if (renderer_ == nullptr) {
            return;
        }
        overlay.toast = "run finished - press Q or close the window";
        draw(scene, overlay);
        while (pumpEvents()) {
            SDL_Delay(16);
            draw(scene, overlay);
        }
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

    // ── 5x7 bitmap font ─────────────────────────────────────────
    //
    // The window has no font: SDL2 alone draws lines and rectangles, and
    // pulling in SDL2_ttf to label a status panel would add a dependency to
    // every build that wants a window. Seven bytes per glyph, five bits used
    // per row, is enough to name what the operator is looking at.
    static const std::uint8_t* glyph(char ch) {
        static const std::uint8_t kUnknown[7] = {0x1F, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1F};
        static const std::uint8_t kSpace[7] = {0, 0, 0, 0, 0, 0, 0};
        static const std::uint8_t kUpper[26][7] = {
            {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}, // A
            {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E}, // B
            {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}, // C
            {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E}, // D
            {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}, // E
            {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10}, // F
            {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F}, // G
            {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}, // H
            {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E}, // I
            {0x07, 0x02, 0x02, 0x02, 0x02, 0x12, 0x0C}, // J
            {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11}, // K
            {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F}, // L
            {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11}, // M
            {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11}, // N
            {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}, // O
            {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10}, // P
            {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D}, // Q
            {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11}, // R
            {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E}, // S
            {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}, // T
            {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}, // U
            {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04}, // V
            {0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11}, // W
            {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11}, // X
            {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04}, // Y
            {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F}, // Z
        };
        static const std::uint8_t kDigit[10][7] = {
            {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}, // 0
            {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E}, // 1
            {0x0E, 0x11, 0x01, 0x06, 0x08, 0x10, 0x1F}, // 2
            {0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E}, // 3
            {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}, // 4
            {0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E}, // 5
            {0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E}, // 6
            {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}, // 7
            {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}, // 8
            {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C}, // 9
        };
        static const std::uint8_t kDot[7] = {0, 0, 0, 0, 0, 0x0C, 0x0C};
        static const std::uint8_t kComma[7] = {0, 0, 0, 0, 0x0C, 0x04, 0x08};
        static const std::uint8_t kColon[7] = {0, 0x0C, 0x0C, 0, 0x0C, 0x0C, 0};
        static const std::uint8_t kMinus[7] = {0, 0, 0, 0x1F, 0, 0, 0};
        static const std::uint8_t kPlus[7] = {0, 0x04, 0x04, 0x1F, 0x04, 0x04, 0};
        static const std::uint8_t kSlash[7] = {0x01, 0x02, 0x02, 0x04, 0x08, 0x08, 0x10};
        static const std::uint8_t kLParen[7] = {0x02, 0x04, 0x08, 0x08, 0x08, 0x04, 0x02};
        static const std::uint8_t kRParen[7] = {0x08, 0x04, 0x02, 0x02, 0x02, 0x04, 0x08};
        static const std::uint8_t kLBracket[7] = {0x0E, 0x08, 0x08, 0x08, 0x08, 0x08, 0x0E};
        static const std::uint8_t kRBracket[7] = {0x0E, 0x02, 0x02, 0x02, 0x02, 0x02, 0x0E};
        static const std::uint8_t kPercent[7] = {0x19, 0x1A, 0x02, 0x04, 0x08, 0x0B, 0x13};
        static const std::uint8_t kEquals[7] = {0, 0, 0x1F, 0, 0x1F, 0, 0};
        static const std::uint8_t kQuestion[7] = {0x0E, 0x11, 0x01, 0x02, 0x04, 0, 0x04};
        static const std::uint8_t kBang[7] = {0x04, 0x04, 0x04, 0x04, 0x04, 0, 0x04};
        static const std::uint8_t kLess[7] = {0x02, 0x04, 0x08, 0x10, 0x08, 0x04, 0x02};
        static const std::uint8_t kGreater[7] = {0x08, 0x04, 0x02, 0x01, 0x02, 0x04, 0x08};
        static const std::uint8_t kStar[7] = {0, 0x15, 0x0E, 0x1F, 0x0E, 0x15, 0};
        static const std::uint8_t kHash[7] = {0x0A, 0x0A, 0x1F, 0x0A, 0x1F, 0x0A, 0x0A};
        static const std::uint8_t kUnderscore[7] = {0, 0, 0, 0, 0, 0, 0x1F};

        if (ch >= 'a' && ch <= 'z') {
            ch = static_cast<char>(ch - 'a' + 'A');
        }
        if (ch >= 'A' && ch <= 'Z') {
            return kUpper[ch - 'A'];
        }
        if (ch >= '0' && ch <= '9') {
            return kDigit[ch - '0'];
        }
        switch (ch) {
            case ' ': return kSpace;
            case '.': return kDot;
            case ',': return kComma;
            case ':': return kColon;
            case '-': return kMinus;
            case '+': return kPlus;
            case '/': return kSlash;
            case '(': return kLParen;
            case ')': return kRParen;
            case '[': return kLBracket;
            case ']': return kRBracket;
            case '%': return kPercent;
            case '=': return kEquals;
            case '?': return kQuestion;
            case '!': return kBang;
            case '<': return kLess;
            case '>': return kGreater;
            case '*': return kStar;
            case '#': return kHash;
            case '_': return kUnderscore;
            default: return kUnknown;
        }
    }

    static int textWidth(const std::string& text, int scale) {
        return static_cast<int>(text.size()) * 6 * scale;
    }

    void drawText(int x, int y, const std::string& text, int scale = 2) {
        int cursor = x;
        for (const char ch : text) {
            const std::uint8_t* bits = glyph(ch);
            for (int row = 0; row < 7; ++row) {
                for (int column = 0; column < 5; ++column) {
                    if ((bits[row] & (1u << (4 - column))) == 0u) {
                        continue;
                    }
                    SDL_Rect pixel{
                        cursor + column * scale, y + row * scale, scale, scale};
                    SDL_RenderFillRect(renderer_, &pixel);
                }
            }
            cursor += 6 * scale;
        }
    }

    void fillPanel(int x, int y, int w, int h) {
        SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_BLEND);
        SDL_SetRenderDrawColor(renderer_, 2, 6, 23, 215);
        SDL_Rect panel{x, y, w, h};
        SDL_RenderFillRect(renderer_, &panel);
        SDL_SetRenderDrawColor(renderer_, 51, 65, 85, 255);
        SDL_RenderDrawRect(renderer_, &panel);
        SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_NONE);
    }

    void setSeverityColor(int severity) {
        switch (severity) {
            case 1: SDL_SetRenderDrawColor(renderer_, 74, 222, 128, 255); break;   // good
            case 2: SDL_SetRenderDrawColor(renderer_, 250, 204, 21, 255); break;   // warning
            case 3: SDL_SetRenderDrawColor(renderer_, 248, 113, 113, 255); break;  // fault
            default: SDL_SetRenderDrawColor(renderer_, 226, 232, 240, 255); break;
        }
    }

    void drawOverlay(const ViewOverlay& overlay) {
        const int scale = 2;
        const int line_height = 9 * scale;

        if (!overlay.status.empty()) {
            std::size_t widest_label = 0;
            std::size_t widest_value = 0;
            for (const auto& line : overlay.status) {
                widest_label = std::max(widest_label, line.label.size());
                widest_value = std::max(widest_value, line.value.size());
            }
            const int panel_w =
                textWidth(std::string(widest_label + widest_value + 2, 'M'), scale) + 20;
            const int panel_h = static_cast<int>(overlay.status.size()) * line_height + 16;
            fillPanel(12, 12, panel_w, panel_h);
            int y = 20;
            for (const auto& line : overlay.status) {
                SDL_SetRenderDrawColor(renderer_, 148, 163, 184, 255);
                drawText(20, y, line.label, scale);
                setSeverityColor(line.severity);
                drawText(20 + textWidth(std::string(widest_label + 2, 'M'), scale), y,
                         line.value, scale);
                y += line_height;
            }
        }

        // A latched E-STOP and a pause are states an operator must not have to
        // hunt for, so they are banners rather than another status line.
        int banner_y = 12;
        if (overlay.estop) {
            const std::string text = "E-STOP LATCHED";
            const int w = textWidth(text, 3) + 24;
            SDL_SetRenderDrawColor(renderer_, 127, 29, 29, 255);
            SDL_Rect box{width_ - w - 12, banner_y, w, 34};
            SDL_RenderFillRect(renderer_, &box);
            SDL_SetRenderDrawColor(renderer_, 254, 226, 226, 255);
            drawText(width_ - w, banner_y + 7, text, 3);
            banner_y += 42;
        }
        if (overlay.paused) {
            const std::string text = "PAUSED";
            const int w = textWidth(text, 3) + 24;
            SDL_SetRenderDrawColor(renderer_, 120, 53, 15, 255);
            SDL_Rect box{width_ - w - 12, banner_y, w, 34};
            SDL_RenderFillRect(renderer_, &box);
            SDL_SetRenderDrawColor(renderer_, 254, 243, 199, 255);
            drawText(width_ - w, banner_y + 7, text, 3);
        }

        if (!overlay.toast.empty()) {
            const int w = textWidth(overlay.toast, scale) + 20;
            fillPanel(12, height_ - 62, w, 8 * scale + 12);
            SDL_SetRenderDrawColor(renderer_, 125, 211, 252, 255);
            drawText(20, height_ - 56, overlay.toast, scale);
        }

        if (overlay.show_help && !overlay.keys.empty()) {
            std::size_t widest = 0;
            for (const auto& line : overlay.keys) {
                widest = std::max(widest, line.size());
            }
            const int panel_w = textWidth(std::string(widest, 'M'), scale) + 24;
            const int panel_h = static_cast<int>(overlay.keys.size()) * line_height + 20;
            const int x = std::max(12, (width_ - panel_w) / 2);
            const int y = std::max(12, (height_ - panel_h) / 2);
            fillPanel(x, y, panel_w, panel_h);
            int line_y = y + 12;
            for (const auto& line : overlay.keys) {
                SDL_SetRenderDrawColor(renderer_, 226, 232, 240, 255);
                drawText(x + 12, line_y, line, scale);
                line_y += line_height;
            }
        } else if (!overlay.keys.empty()) {
            SDL_SetRenderDrawColor(renderer_, 100, 116, 139, 255);
            drawText(12, height_ - 14, "H = keys", 1);
        }
    }

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

        Projection() = default;

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

        /// Inverse of project(): turns a pixel the operator clicked back into
        /// the coordinate drawn there. Exact, because the projection is a
        /// scale and a translation about the scene centre.
        rozeta::GeoCoordinate unproject(double x, double y) const {
            if (!(pixels_per_meter > 0.0)) {
                return center;
            }
            const double east_m = (x - width / 2.0) / pixels_per_meter;
            const double north_m = (height / 2.0 - y) / pixels_per_meter;
            return rozeta::geodesy::offsetMeters(center, east_m, north_m);
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

    /// Hollow square: a point the operator has picked but nothing has planned
    /// through yet.
    void drawRing(const Projection& projection, const rozeta::GeoCoordinate& point, int radius) {
        const auto screen = projection.project(point);
        SDL_Rect rect{
            static_cast<int>(screen.x) - radius,
            static_cast<int>(screen.y) - radius,
            radius * 2,
            radius * 2};
        SDL_RenderDrawRect(renderer_, &rect);
        SDL_Rect inner{rect.x + 1, rect.y + 1, rect.w - 2, rect.h - 2};
        SDL_RenderDrawRect(renderer_, &inner);
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
    Projection last_projection_{};
    bool has_projection_{false};

#else  // ROZETA_WITH_SDL2

public:
    static constexpr bool kAvailable = false;

    bool open(int, int, std::string& error) {
        error = "this build has no window support (configure with -DROZETA_WITH_SDL2=ON)";
        return false;
    }
    bool isOpen() const { return false; }
    bool pumpEvents() { return true; }
    bool poll(ViewInput& out) { out = ViewInput{}; return true; }
    void draw(const rozeta::ui::NavigationScene&) {}
    void draw(const rozeta::ui::NavigationScene&, const ViewOverlay&) {}
    void waitForClose(const rozeta::ui::NavigationScene&) {}
    void waitForClose(const rozeta::ui::NavigationScene&, ViewOverlay) {}
    void close() {}

#endif // ROZETA_WITH_SDL2
};

} // namespace rozeta_examples
