# Resilience: clock, health, gps gate, safety state, faults

Five modules that together answer one question a robot must never guess at:
**may the motors turn right now, and how fast?**

They are deliberately separate. Each is a pure value transformer with no clock
of its own and no thread, so the whole failure matrix can be exercised in a unit
test in microseconds rather than in a field session.

| header | what it owns |
|--------|--------------|
| `rozeta/clock.hpp` | injectable time: `SystemClock`, `SimulatedClock` |
| `rozeta/health.hpp` | per-sensor state, confidence and hysteresis |
| `rozeta/gps_gate.hpp` | plausibility of a single GPS fix |
| `rozeta/safety_state.hpp` | the safety state machine, speed governor and motor gate |
| `rozeta/faults.hpp` | deterministic fault injection over the same interfaces |

## clock

`rozeta::Clock` reports monotonic milliseconds. `SystemClock` wraps
`std::chrono::steady_clock` with its zero point at construction; `SimulatedClock`
is advanced by the caller and refuses to move backwards, because a clock that
can run backwards would let a stale reading look fresh. Every module below takes
`now` as an argument rather than reading a clock, which is what makes a
twenty-second GPS dropout a test that runs instantly.

## health

`health::SensorHealth` turns a stream of samples into one of five states:

    Ok -> Degraded -> Stale -> Failed        (by age, or by repeated bad samples)
    Unavailable                              (not fitted / not configured)

Two rules make it usable for safety decisions. **Deterioration is immediate**: a
sensor that ages past a threshold reports it on the same tick. **Recovery is
earned**: leaving a worse state needs `samples_to_recover` consecutive valid
samples, and entering a worse state resets that counter, so samples from before
a failure cannot count towards leaving it.

An invalid sample never refreshes the age. A receiver that keeps emitting
rejected sentences therefore still goes Stale, which is the behaviour that
distinguishes "the data is bad" from "the data is late".

`Unavailable` ranks *below* `Degraded` in `severityOf()`: a sensor that was never
fitted is a configuration fact, not a fault. Leaving `Unavailable` is exempt from
hysteresis, since the first sample is proof the sensor exists.

`health::HealthRegistry` aggregates named sensors and reports a
`SystemHealthSummary`: the worst state overall, the worst among sensors marked
critical, the lowest critical confidence, and a reason string naming the sensor
responsible.

## gps gate

`gps::GpsGate` sits between a receiver and the pose estimate and answers one
question per fix: may this sample move the robot? It rejects

* structurally impossible fixes — invalid, NaN, out of range, exactly `0,0`;
* **impossible jumps** — further than `max_speed_mps * dt` plus a fixed grace
  *plus the receiver's own measured scatter*. Without that last term ordinary
  jitter reads as a teleport, and a gate that rejects good data hides the real
  failures;
* **a frozen receiver** — coordinates unchanged for `freeze_window` while
  independent `MotionEvidence` says the robot is moving. Without motion evidence
  the check stays off, because a stationary robot's receiver is *supposed* to
  repeat itself;
* fixes whose reported accuracy is worse than `max_accuracy_m`.

Accuracy, HDOP and satellite count degrade `confidence` rather than causing a
rejection, and a disagreement with the independent displacement estimate
(`odometry_disagreement`) drops confidence sharply without discarding the sample:
refusing every contradicted fix would stall navigation whenever a wheel slips.

After `max_consecutive_rejects` rejections in a row the gate **re-anchors** on the
newest fix and says so. A receiver that genuinely re-acquired somewhere else must
eventually be believed; the re-anchored fix carries halved confidence so the
speed governor slows down until it proves itself.

`applyToHealth()` wires a verdict into a `health::SensorHealth`.

## safety state

`safety::SafetyStateMachine` replaces scattered booleans with one state and a
reason for every transition:

    READY -> RUNNING -> DEGRADED -> STOPPING -> STOPPED
                 \-------------------> EMERGENCY_STOP
                 \-------------------> FAULT

`EMERGENCY_STOP` is latched: removing the request is not enough, an operator must
acknowledge. A physical E-STOP outranks every other input including a clear
request. A failed preflight refuses an autonomous start visibly, as `FAULT`,
rather than silently staying in `READY`.

`BoundedAutonomyConfig` limits how long and how far the robot may run without a
fresh absolute fix. Whichever of `max_dead_reckoning` and `max_dead_reckoning_m`
trips first ends the fallback with a controlled stop — degraded localization is
never trusted indefinitely.

`safety::SpeedGovernor` maps conditions to a speed cap by taking the **minimum**
over every applicable limit, so two simultaneous problems can never combine into
a higher speed than either alone would allow. Pose confidence scales what is
left. A cap that falls to zero becomes a `STOPPING` transition rather than a
silent zero command.

`safety::MotorCommandLimiter` is the final gate every autonomous command passes
through. It guarantees the invariants the rest of the system may assume: output
finite, within `[-1, 1]`, scaled by the active limit, and exactly zero whenever
motion is not permitted. A non-finite command becomes zero, never a clamped
extreme — NaN means the caller's maths broke, and full speed is the wrong guess
about what it meant.

## faults

`faults::FaultInjector` applies a schedule of failures to sensor samples and
drive commands. The faulty backends implement the same interfaces as the real
ones, so nothing above them can tell a fault is active:

    gps::GpsReceiver     <- FaultyGps
    lidar::LidarScanner  <- FaultyLidar
    motors::MotorController <- FaultyDrive

Scenarios are text:

    # a short dropout under the bridge
    at: 12.0
    fault: gps_dropout
    duration: 5.0
    label: under the bridge

A misspelled fault name is a parse error, not a silent no-op: a typo that parsed
would produce a green test run that proved nothing.

Supported faults: `gps_dropout`, `gps_freeze`, `gps_jump`, `gps_noise`,
`gps_accuracy_loss`, `lidar_dropout`, `lidar_freeze`, `lidar_noise`,
`lidar_partial`, `lidar_zero_storm`, `motor_left_failure`,
`motor_right_failure`, `motor_power_loss`, `motor_no_motion`,
`serial_disconnect`, `imu_freeze`, `camera_dropout`, `obstacle_appears`,
`wheel_slip`.

`FaultyDrive::setSpeed()` returns `IoError` while `serial_disconnect` is active,
exactly as `SerialMotorController` does, so the runtime's retry and watchdog
paths are exercised by the code that runs on the robot. `emergencyStop()` is
never blocked by an injected link fault: on the robot the physical cutout does
not go through the serial link, and the gate that calls it must be able to
assume it always arrives.

Same seed plus same schedule gives the same run on every platform.
