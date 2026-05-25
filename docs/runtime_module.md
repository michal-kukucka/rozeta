# Runtime module

`rozeta::runtime::MissionRuntime` is the M2 deterministic supervisor for Buchlovice/Robotour-style loops from `main.py` and the orchestration parts of `kvalifikacia_demo.py`.

The runtime is intentionally tick-based and thread-free. A caller owns sensors, navigation, motors, logging and UI, then feeds fresh health and mission facts into `MissionRuntime::tick(...)`. The returned `RuntimeOutput` tells the application whether to stop, emergency-stop, run a bypass policy, or resend the last motor command as a motor keepalive.

## Mission phases

`MissionPhase` contains the reusable lifecycle states needed by the Buchlovice project:

- `Init`
- `WaitingForStart`
- `Countdown`
- `Driving`
- `ObstacleWait`
- `Bypass`
- `Arrived`
- `Shutdown`
- `Fault`

This maps the old script-level flow into a small state machine without hiding hardware side effects inside Rozeta.

## Module health inputs

`RuntimeInputs` carries explicit module health flags:

- motors
- GPS
- camera
- depth/Kinect
- map
- communication
- logging

Any unhealthy critical module transitions to `Fault`, requests stop, and requests emergency stop with a reason such as `critical module unhealthy: gps`.

## Tick-based policy hooks

`RuntimeConfig` exposes deterministic counters and timing:

```cpp
rozeta::runtime::RuntimeConfig config;
config.countdown_ticks = 3;
config.obstacle_wait_ticks = 50;
config.bypass_ticks = 10;
config.motor_keepalive_interval = std::chrono::milliseconds(200);
```

`RuntimeOutput` exposes policy hooks instead of touching hardware directly:

- `request_stop` — caller should send a safe stop command.
- `emergency_stop` — caller should latch motor emergency stop and enter safety handling.
- `request_bypass` — caller may run a local obstacle bypass maneuver.
- `resend_last_motor_command` — caller should resend the last safe motor command as the Buchlovice motor keepalive.

The M1 Buchlovice serial backend exposes `buchlovice_repeat_interval`; M2 wires the same concept into runtime-owned scheduling via `motor_keepalive_interval` so the default library remains deterministic, thread-free and hardware-free.

## Minimal usage

```cpp
rozeta::runtime::MissionRuntime runtime;
rozeta::runtime::RuntimeInputs inputs;
inputs.motors_healthy = true;
inputs.gps_healthy = true;
inputs.camera_healthy = true;
inputs.depth_healthy = true;
inputs.map_healthy = true;
inputs.communication_healthy = true;
inputs.logging_healthy = true;
inputs.start_requested = true;

const auto out = runtime.tick(inputs, std::chrono::milliseconds(0));
if (out.emergency_stop) {
    // motor.emergencyStop();
} else if (out.request_stop) {
    // motor.stop();
} else if (out.resend_last_motor_command) {
    // motor.setSpeed(last_safe_command);
    runtime.markMotorCommandSent(std::chrono::milliseconds(0));
}
```

## Verification

The module is covered by `tests/test_runtime.cpp`:

- deterministic waiting/start/countdown/driving/arrival flow.
- fault transition on unhealthy critical module.
- obstacle wait, bypass, and driving resume flow.
- motor keepalive scheduling using explicit tick timestamps.

Default CI uses fake inputs only; no motors, cameras, GPS, Kinect, communication links or timers are opened by the runtime tests.
