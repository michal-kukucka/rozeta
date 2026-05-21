# M3 — GPS Serial Receiver and Robust NMEA Validation Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Move GPS from parser-only to a usable serial receiver with checksum validation, stream buffering, and documented device setup.

**Architecture:** Keep `gps::NmeaParser` pure and dependency-free. Add a receiver class that reads lines from serial transport and delegates parsing to the existing parser.

**Tech Stack:** C++17, internal serial utility from M1, CTest fixtures with recorded NMEA sentences.

---

## Gap evidence

- `docs/robotour_use_case.md` lists GPS serial receiver and checksum validation as next milestones.
- Current code parses GGA/RMC but does not own serial stream lifecycle.

## Tasks

1. Add RED tests for checksum pass/fail cases in `tests/test_gps.cpp`.
2. Add stream-buffer tests for partial lines and multiple NMEA messages.
3. Add `GpsReceiverConfig` and serial receiver interface/implementation.
4. Add `examples/gps_serial_reader.cpp` with `--device`, `--baud`, and sample-file fallback.
5. Document Linux permissions and sample NMEA fixtures in `docs/gps_module.md`.
6. Update `docs/diagrams/module-map.html` with serial GPS data flow.

## Verification

```bash
ctest --test-dir build -R gps --output-on-failure
./build/examples/gps_serial_reader --sample tests/fixtures/gps/robotour_sample.nmea
python3 scripts/verify_docs.py
doxygen Doxyfile
```

## Acceptance criteria

- Invalid checksums are rejected with clear status.
- Receiver handles fragmented serial reads.
- Example works without GPS hardware via sample input.
