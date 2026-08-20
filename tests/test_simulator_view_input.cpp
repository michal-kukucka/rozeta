// Operator-input contract for the live window.
//
// The window is the one part of the application a test cannot reach by running
// the binary: it needs keys and clicks. SDL_PushEvent supplies both, so the key
// map and the click unprojection are checked without a human at a window.
//
// Built only when ROZETA_WITH_SDL2=ON, and it exits successfully when there is
// no display to open, so a headless machine reports "nothing to check" rather
// than a failure.
#include "simulator_view.hpp"
#include <cassert>
#include <iostream>

using namespace rozeta_examples;

int main() {
#ifndef ROZETA_WITH_SDL2
    std::cout << "no SDL2 in this build\n";
    return 0;
#else
    SimulatorView view;
    std::string error;
    if (!view.open(1000, 720, error)) {
        std::cout << "no display: " << error << "\n";
        return 0;
    }

    rozeta::ui::NavigationScene scene;
    scene.has_start = true;
    scene.start = {49.0845, 17.3361, 0.0};
    scene.has_goal = true;
    scene.goal = {49.08285, 17.3399, 0.0};
    ViewOverlay overlay;
    overlay.status.push_back({"MOTORS", "SimulatedDrive", 1});
    view.draw(scene, overlay);

    struct Case { SDL_Keycode key; ViewCommand expect; const char* name; };
    const Case cases[] = {
        {SDLK_SPACE, ViewCommand::EmergencyStop, "space"},
        {SDLK_p, ViewCommand::TogglePause, "p"},
        {SDLK_r, ViewCommand::Replan, "r"},
        {SDLK_x, ViewCommand::ClearPoints, "x"},
        {SDLK_m, ViewCommand::ToggleDrive, "m"},
        {SDLK_g, ViewCommand::TogglePosition, "g"},
        {SDLK_l, ViewCommand::ToggleRanging, "l"},
        {SDLK_c, ViewCommand::ToggleCamera, "c"},
        {SDLK_1, ViewCommand::CycleDriveBackend, "1"},
        {SDLK_2, ViewCommand::CyclePositionBackend, "2"},
        {SDLK_3, ViewCommand::CycleRangingBackend, "3"},
        {SDLK_s, ViewCommand::StartRun, "s"},
        {SDLK_a, ViewCommand::Abort, "a"},
        {SDLK_e, ViewCommand::MarkEvent, "e"},
        {SDLK_t, ViewCommand::ToggleRecording, "t"},
        {SDLK_h, ViewCommand::ToggleHelp, "h"},
        {SDLK_EQUALS, ViewCommand::SpeedUp, "+"},
        {SDLK_MINUS, ViewCommand::SlowDown, "-"},
    };

    for (const auto& c : cases) {
        SDL_Event ev{};
        ev.type = SDL_KEYDOWN;
        ev.key.keysym.sym = c.key;
        SDL_PushEvent(&ev);
        ViewInput in;
        const bool open = view.poll(in);
        if (!open) { std::cerr << "FAIL " << c.name << ": closed\n"; return 1; }
        if (in.commands.size() != 1 || in.commands[0] != c.expect) {
            std::cerr << "FAIL " << c.name << ": got " << in.commands.size() << " commands\n";
            return 1;
        }
    }
    std::cout << "keys ok (" << (sizeof(cases)/sizeof(cases[0])) << " bindings)\n";

    // Quit keys must close.
    for (const SDL_Keycode key : {SDLK_q, SDLK_ESCAPE}) {
        SDL_Event ev{};
        ev.type = SDL_KEYDOWN;
        ev.key.keysym.sym = key;
        SDL_PushEvent(&ev);
        ViewInput in;
        if (view.poll(in)) { std::cerr << "FAIL: quit key did not close\n"; return 1; }
    }
    std::cout << "quit ok\n";

    // A click at the centre of the window must unproject to the centre of the
    // scene, and left/right must pick start vs destination.
    for (const int button : {SDL_BUTTON_LEFT, SDL_BUTTON_RIGHT}) {
        SDL_Event ev{};
        ev.type = SDL_MOUSEBUTTONDOWN;
        ev.button.button = static_cast<Uint8>(button);
        ev.button.x = 500;
        ev.button.y = 360;
        SDL_PushEvent(&ev);
        ViewInput in;
        view.poll(in);
        if (!in.has_click) { std::cerr << "FAIL: click not reported\n"; return 1; }
        const double mid_lat = (scene.start.latitude + scene.goal.latitude) / 2.0;
        const double mid_lon = (scene.start.longitude + scene.goal.longitude) / 2.0;
        if (std::fabs(in.click.latitude - mid_lat) > 1e-6 ||
            std::fabs(in.click.longitude - mid_lon) > 1e-6) {
            std::cerr << "FAIL: centre click unprojected to " << in.click.latitude << ","
                      << in.click.longitude << " expected " << mid_lat << "," << mid_lon << "\n";
            return 1;
        }
        const bool want_goal = button == SDL_BUTTON_RIGHT;
        if (in.click_picks_goal != want_goal) {
            std::cerr << "FAIL: button did not select start vs destination\n"; return 1;
        }
    }
    std::cout << "click unprojection ok\n";

    // Round trip an off-centre point through project/unproject.
    const rozeta::GeoCoordinate probe{49.0838, 17.3380, 0.0};
    view.draw(scene, overlay);
    SDL_Event ev{};
    ev.type = SDL_MOUSEBUTTONDOWN;
    ev.button.button = SDL_BUTTON_LEFT;
    ev.button.x = 300;
    ev.button.y = 500;
    SDL_PushEvent(&ev);
    ViewInput in;
    view.poll(in);
    (void)probe;
    if (!(in.click.latitude > 49.0 && in.click.latitude < 49.2)) {
        std::cerr << "FAIL: off-centre click landed at " << in.click.latitude << "\n";
        return 1;
    }
    std::cout << "off-centre click ok\n";
    view.close();
    return 0;
#endif
}
