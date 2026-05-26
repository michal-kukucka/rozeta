# Buchlovice motor hardware smoke runbook

This is the M1 hardware smoke runbook for the Buchlovice binary serial motor backend. Default CI stays hardware-free; run these steps only with the robot safely lifted or mechanically disabled.

## Preconditions

- Build with `ROZETA_WITH_SERIAL_MOTORS=ON`.
- Confirm the controller device path, for example `/dev/ttyUSB0`.
- Wheels are off the ground, operator has physical emergency stop access, and a second person can cut power.
- Start with the dry-run helper before opening a serial device.

## Build and dry-run

```bash
cmake -S . -B build-serial   -DROZETA_BUILD_TESTS=ON   -DROZETA_BUILD_EXAMPLES=ON   -DROZETA_WITH_SERIAL_MOTORS=ON
cmake --build build-serial --parallel
ctest --test-dir build-serial --output-on-failure
./build-serial/examples/serial_motor_calibrate --dry-run --buchlovice-binary
```

The dry run prints the BuchloviceBinary packet contract without sending bytes:
`[255, pwm_right, pwm_left, reg, lrc, 13, 10]`.

## Live smoke checklist

1. Keep the first live command short and low-speed.
2. Verify stop writes `[255, 0, 0, 0, 0, 13, 10]`.
3. Verify forward/reverse direction bits match Buchlovice wiring: right bit 0, left bit 1.
4. Verify the application resends the last safe command every 200 ms using `MissionRuntime` motor keepalive, not a serial backend thread.
5. Record controller path, baud rate, observed wheel direction, and any wiring inversion in the field log.

## Abort criteria

- Any unexpected motion, checksum mismatch, wrong direction bit, stale keepalive, or operator discomfort means cut power and file a hardware note before retrying.
