# Arduino MDDS30 bridge — firmware upload and wiring

This is the **default drive path** for a Rozeta robot: the PC runs Rozeta, an
Arduino UNO runs the bridge firmware shipped in this repository, and a Cytron
MDDS30 (SmartDriveDuo-30) delivers power to two brushed DC motors.

```
PC / robot computer (Rozeta, motors::SmoothDrive)
      |  USB cable (USB-serial, 115200 baud, ASCII line protocol)
      v
Arduino UNO (bridge sketch, arduino/mdds30_bridge/)
      |  PWM + DIR logic signals
      v
Cytron MDDS30
      |  power stage
      v
2x brushed DC motor
```

Rozeta never drives the motors directly. It sends high-level speed commands; the
Arduino converts them to PWM/DIR signals and enforces a communication watchdog —
if commands stop arriving (application killed, cable unplugged), the Arduino
stops both motors on its own.

> **Read first — the Arduino firmware must be uploaded before anything works.**
> A brand-new or factory Arduino does **not** understand Rozeta. You must flash
> the included bridge sketch (`arduino/mdds30_bridge/mdds30_bridge.ino`) onto the
> board **once**, before running the robot. Until you do, Rozeta can open the
> serial port but the motor driver stays completely idle.

## What ships in this repository

| Path | Description |
|------|-------------|
| `arduino/mdds30_bridge/mdds30_bridge.ino` | Arduino UNO bridge firmware (complete source). Upload it to the board before first use. |
| `include/rozeta/motors.hpp` | `SerialMotorProtocol::CytronMdds30`, `cytronMdds30Config()`, `SmoothDrive`, `DriveProfile`. |
| `examples/cytron_trip_demo.cpp` | Full trip: accelerate to cruise speed, then brake fluently to standstill. |

The firmware is byte-identical to the sketch used by the
[Cytron Motordriver Tester](https://github.com/michal-kukucka/cytrontester) GUI,
so the same board works with both the tester and Rozeta without reflashing.

## Setup: upload the Arduino firmware

**Do this once, before you use the robot.** Connect the Arduino to the PC with a
**data** USB cable (charge-only cables will not enumerate the board), then use
either method below.

### Option A — Arduino IDE (graphical, easiest)

1. Install the Arduino IDE (<https://www.arduino.cc/en/software>).
2. Open `arduino/mdds30_bridge/mdds30_bridge.ino`.
3. **Tools → Board → Arduino UNO**.
4. **Tools → Port →** your board
   (macOS: `/dev/cu.usbmodem*`, Linux: `/dev/ttyACM*`, Windows: `COM*`).
5. Click **Upload** (right arrow). Wait for "Done uploading."

### Option B — arduino-cli (command line)

```bash
# Install the CLI once
#   macOS:  brew install arduino-cli
#   others: https://arduino.github.io/arduino-cli/latest/installation/

# Install the AVR core (one-time)
arduino-cli core update-index
arduino-cli core install arduino:avr

# Find your board's port
arduino-cli board list

# Compile and upload (replace the port with the one from `board list`)
arduino-cli compile --fqbn arduino:avr:uno arduino/mdds30_bridge
arduino-cli upload  -p /dev/cu.usbmodem14101 --fqbn arduino:avr:uno arduino/mdds30_bridge
```

> **Note:** the upload needs exclusive access to the serial port. If Rozeta (or
> an Arduino Serial Monitor) is connected to the board, disconnect it first, or
> the upload fails with the port busy.

### Verify the upload

Open any serial terminal at 115200 baud (or the Arduino Serial Monitor). Right
after reset the board prints:

```text
OK HERMES_MDDS30_BRIDGE_READY
```

Type `PING` and it answers `OK PONG`. If you see nothing, the wrong port or the
wrong board type was selected during upload.

## Hardware

- Cytron MDDS30 / SmartDriveDuo-30
- Arduino UNO (or compatible 5 V board)
- Two brushed DC motors (MDDS30 is **not** for brushless motors)
- Separate motor battery connected to the MDDS30 Vmotor terminals
  (Arduino USB power is *not* motor power)

## Wiring: Arduino UNO → MDDS30

Signal (logic) side:

| Arduino UNO | Type    | MDDS30 | Function                  |
|-------------|---------|--------|---------------------------|
| D5          | PWM     | AN1    | Left motor speed          |
| D7          | DIGITAL | IN1    | Left motor direction      |
| D6          | PWM     | AN2    | Right motor speed         |
| D8          | DIGITAL | IN2    | Right motor direction     |
| GND         | GND     | GND    | Common logic ground (required) |

Power side (kept separate from logic):

```text
Motor battery +  --->  Vmotor +
Motor battery -  --->  Vmotor -

Left motor       --->  MLA / MLB
Right motor      --->  MRA / MRB
```

Do **not** connect the motor battery to the Arduino 5 V pin, and do not power
motors from the Arduino.

## MDDS30 DIP switch configuration

Mode: *Microcontroller PWM input, Independent Both, Signed Magnitude, Linear
response.*

| Switch | Position | Meaning |
|--------|----------|---------|
| SW1    | ON       | PWM input mode (with SW2) |
| SW2    | OFF      | PWM input mode (with SW1) |
| SW3    | ON       | Independent Both motors (with SW4) |
| SW4    | ON       | Independent Both motors (with SW3) |
| SW5    | OFF      | Linear response |
| SW6    | ON       | Signed Magnitude mode |

SW7/SW8 select the battery-monitor type, not the control mode:

| Battery monitor | SW7 | SW8 |
|-----------------|-----|-----|
| LiPo            | OFF | OFF |
| NiMH            | OFF | ON  |
| SLA (lead-acid) | ON  | OFF |
| Off             | ON  | ON  |

**Important:** change DIP switches only with power off. The MDDS30 reads the
input mode at startup/reset — after changing switches, power-cycle the driver or
press its RESET button. The manual also requires a valid stop signal at
power-up; in this mode, stop means **0 % PWM duty cycle** on AN1 and AN2 (the
bridge sketch outputs this from boot).

## Recommended power-up order

1. Upload the bridge sketch to the Arduino (one-time, see above).
2. Connect Rozeta (or a serial monitor) and confirm the Arduino reports
   `OK HERMES_MDDS30_BRIDGE_READY` with motors stopped.
3. Power the MDDS30 motor supply.
4. If the MDDS30 reports an input error, send `STOP` and press its RESET button.

First tests: **wheels off the ground.**

## Serial protocol (Rozeta → Arduino)

115200 baud, 8N1, ASCII lines terminated by `\n`.

| Command | Effect | Reply |
|---------|--------|-------|
| `M L=<l> R=<r>` | Set both motors. `<l>`, `<r>` are integers −100…100: 0 = stop, positive = forward, negative = reverse, magnitude = % PWM. | `OK L=<l> R=<r>` |
| `STOP` | Immediately stop both motors. | `OK STOP` |
| `PING` | Liveness check. | `OK PONG` |
| `STATUS` | Report last commanded values and timeout. | `OK L=<l> R=<r> TIMEOUT=<ms>` |
| `TIMEOUT=<ms>` | Set watchdog timeout, 50–5000 ms (default 300). | `OK TIMEOUT=<ms>` |

Rejected commands answer `ERR <reason>`.

**Watchdog:** if no valid command arrives within the timeout, the Arduino stops
both motors and prints `OK WATCHDOG_STOP`. Rozeta therefore repeats the active
command every 100 ms while any motor is running — `DriveProfile::command_interval`
in `motors::SmoothDrive` owns that timing (see
[Motor module](motor_module.md#trip-level-drive-smoothdrive)).

## Driving from Rozeta

Build with the serial backend enabled:

```bash
cmake -S . -B build-serial -DROZETA_BUILD_EXAMPLES=ON -DROZETA_WITH_SERIAL_MOTORS=ON
cmake --build build-serial --parallel 2
```

```cpp
#include <rozeta/motors.hpp>

// Bridge defaults: CytronMdds30 protocol, 115200 baud, 100 ms keepalive.
rozeta::motors::SerialMotorController motors(
    rozeta::motors::cytronMdds30Config("/dev/cu.usbmodem14201"));
motors.open();

// Every trip accelerates and brakes within the profile limits.
rozeta::motors::SmoothDrive drive(motors, rozeta::motors::cytronMdds30DriveProfile());
drive.setTarget(0.8, 0.8);                 // cruise request
drive.tick(now_ms);                        // call from the control loop
drive.brake();                             // fluent stop; ramps down, then STOP
```

Run the shipped demo against real hardware:

```bash
./build-serial/examples/cytron_trip_demo --device /dev/cu.usbmodem14201
```

## Direction calibration

"Forward" depends on motor wiring and mounting. If a motor runs the wrong way
for a positive command, either flip `LEFT_FORWARD_LEVEL` / `RIGHT_FORWARD_LEVEL`
in the sketch and re-upload, or power off the driver and swap that motor's two
output wires. Never swap motor wires while the driver is powered.

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| Port opens, motors never move | Firmware not uploaded | Flash `arduino/mdds30_bridge/` (see above) |
| Board does not appear in the port list | Charge-only USB cable | Use a data USB cable |
| Upload fails, port busy | Rozeta or Serial Monitor holds the port | Disconnect, then upload |
| Motors run for ~0.3 s then stop | Commands slower than the watchdog | Keep `drive.command_interval_ms` below 300 ms |
| `ERR bad_move_command` | Malformed line | Command must be `M L=<int> R=<int>` |
| Motors twitch instead of running | Wrong DIP switch mode | Re-check the table above, power-cycle the MDDS30 |
| One wheel turns the wrong way | Motor wiring/mounting | See direction calibration above |

## Safety notes

- MDDS30 does not protect against reversed motor-supply polarity — double-check
  Vmotor + / − before power-up.
- A fuse or breaker in the motor battery line is strongly recommended.
- Arduino GND and MDDS30 GND must be connected together.
- First tests with wheels off the ground.
- The Arduino watchdog is a backstop, not a replacement for the physical E-STOP
  required by `docs/safety_module.md`.

Related: [Motor module](motor_module.md),
[Cytron hardware smoke runbook](cytron_motor_hardware_smoke.md),
[Safety module](safety_module.md).
