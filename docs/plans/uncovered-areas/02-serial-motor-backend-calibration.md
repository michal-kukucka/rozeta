# M2 — Serial Motor Backend and Calibration Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Add a real optional serial motor backend with calibration persistence while preserving the existing mock controller and emergency-stop guarantees.

**Architecture:** Keep `motors::MotorController` as the stable public interface. Add a concrete serial backend behind `ROZETA_WITH_SERIAL_MOTORS` and test command formatting with fake serial transport.

**Tech Stack:** C++17, internal serial utility from M1, CMake option, CTest.

---

## Gap evidence

- `docs/motor_module.md` lists serial motor controller, encoder polling and calibration persistence as future backends.
- Current tests cover mock validation/emergency stop only.

## Tasks

1. Add tests for normalized speed to serial command conversion in `tests/test_serial_motors.cpp`.
2. Add tests for emergency stop writing the configured stop command before refusing further movement.
3. Add `motors::SerialMotorConfig` to `include/rozeta/motors.hpp` if needed.
4. Implement `SerialMotorController` in `src/motors_serial.cpp` or isolated backend file.
5. Add calibration load/save tests using temporary files.
6. Create `examples/serial_motor_calibrate.cpp` with dry-run mode.
7. Update `docs/motor_module.md`, `docs/api-reference.md`, and module diagram data flow.

## Verification

```bash
cmake -S . -B build -DROZETA_BUILD_TESTS=ON -DROZETA_BUILD_EXAMPLES=ON -DROZETA_WITH_SERIAL_MOTORS=ON
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
./build/examples/serial_motor_calibrate --dry-run
python3 scripts/verify_docs.py
```

## Acceptance criteria

- Real backend is optional.
- Mock backend behavior stays unchanged.
- Emergency stop is tested at API and backend level.
- Calibration can be saved/loaded and documented.


## Implementation progress

Status: completed.

Implemented files:

- `src/internal/serial_motor_backend.hpp`
- `src/motors_serial.cpp`
- `tests/test_serial_motors.cpp`
- `tests/test_motor_calibration.cpp`
- `examples/serial_motor_calibrate.cpp`

Modified files:

- `include/rozeta/motors.hpp` — added calibration persistence APIs plus optional serial motor config/controller.
- `src/motors.cpp` — added calibration save/load validation while preserving mock behavior.
- `CMakeLists.txt`, `tests/CMakeLists.txt`, `examples/CMakeLists.txt` — added `ROZETA_WITH_SERIAL_MOTORS` and conditional tests/example.
- `docs/motor_module.md`, `docs/api-reference.md`, `docs/diagrams/module-map.html`, `README.md` — documented M2 behavior and example usage.

Acceptance criteria status:

- [x] Real backend is optional behind `ROZETA_WITH_SERIAL_MOTORS`.
- [x] Mock backend behavior stays unchanged.
- [x] Emergency stop is tested at backend level and writes the stop command before refusing motion.
- [x] Calibration can be saved/loaded and is documented.

Verification:

```bash
ctest --test-dir build-m2 --output-on-failure
./build-m2/examples/serial_motor_calibrate --dry-run
python3 scripts/verify_docs.py
```
