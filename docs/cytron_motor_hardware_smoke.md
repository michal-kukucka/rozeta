# Cytron MDDS30 motor hardware smoke runbook

Hardware smoke runbook for the `SerialMotorProtocol::CytronMdds30` serial motor backend: laptop →
Arduino UNO bridge → Cytron MDDS30 (SmartDriveDuo-30) → two brushed DC motors. The bridge firmware
ships in this repository at `arduino/mdds30_bridge/mdds30_bridge.ino`; its upload procedure, wiring
table and DIP-switch reference are in [Arduino MDDS30 bridge](arduino_mdds30_bridge.md). Default CI
stays hardware-free; run these steps only with the robot safely lifted or mechanically disabled.

## Preconditions

- Build with `ROZETA_WITH_SERIAL_MOTORS=ON`.
- Upload `arduino/mdds30_bridge/mdds30_bridge.ino` to the Arduino UNO (Arduino IDE or `arduino-cli`,
  see [Arduino MDDS30 bridge](arduino_mdds30_bridge.md)) and confirm it prints
  `OK HERMES_MDDS30_BRIDGE_READY` on boot with motors stopped. A board that was never flashed
  accepts the serial connection but never moves the driver.
- MDDS30 DIP switches set for *PWM input, Independent Both, Signed Magnitude, Linear response*
  (SW1 ON, SW2 OFF, SW3 ON, SW4 ON, SW5 OFF, SW6 ON) — change switches only with power off, then
  power-cycle or reset the driver.
- Confirm the bridge device path, for example `/dev/cu.usbmodem14201` (macOS), `/dev/ttyACM0`
  (Linux) or `COM3` (Windows). Baud is 115200 (the `SerialMotorConfig` default).
- Motor battery connected to MDDS30 Vmotor only — never to the Arduino. Arduino GND and MDDS30 GND
  common.
- Wheels are off the ground, operator has physical emergency stop access, and a second person can
  cut power.
- Start with the dry-run helper before opening a serial device.

## Build and dry-run

```bash
cmake -S . -B build-serial   -DROZETA_BUILD_TESTS=ON   -DROZETA_BUILD_EXAMPLES=ON   -DROZETA_WITH_SERIAL_MOTORS=ON
cmake --build build-serial --parallel
ctest --test-dir build-serial --output-on-failure
./build-serial/examples/serial_motor_calibrate --dry-run --cytron-mdds30
./build-serial/examples/cytron_trip_demo          # mock backend, no device opened
```

The dry run prints the CytronMdds30 line contract without sending bytes:
`M L=<-100..100> R=<-100..100>`, stop `STOP`, keepalive 100 ms against the bridge's 300 ms watchdog.

## Live smoke checklist

1. Open the port; the UNO auto-resets — wait ~2 s, then confirm `OK HERMES_MDDS30_BRIDGE_READY`.
2. Keep the first live command short and low-speed; the bridge answers `OK L=<l> R=<r>`.
3. Verify `stop()` writes `STOP` and the bridge answers `OK STOP`.
4. Verify the application resends the last safe command at `cytron_repeat_interval` (100 ms) using
   `SmoothDrive::tick()` or `MissionRuntime` motor keepalive, not a serial backend thread. Pause the
   sender and confirm the bridge prints `OK WATCHDOG_STOP` and both motors stop within ~300 ms.
5. Verify forward/reverse per side matches wiring; fix inversions with `LEFT_FORWARD_LEVEL` /
   `RIGHT_FORWARD_LEVEL` in the sketch or by swapping that motor's output wires with power off.
6. Run `cytron_trip_demo --device <port>` and confirm the trip accelerates smoothly to cruise speed,
   brakes fluently and ends in `STOP` with no watchdog trip in between.
7. Exercise a `SpeedRamp` accelerate/decelerate cycle and confirm smooth linear response ending in
   `STOP`.
8. Record bridge device path, observed wheel directions, and any wiring inversion in the field log.

## Abort criteria

- Any unexpected motion, `ERR ...` reply on a well-formed command, missed watchdog stop, stale
  keepalive, wrong direction, or operator discomfort means cut power and file a hardware note before
  retrying.
