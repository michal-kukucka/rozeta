#!/usr/bin/env python3
"""Contract checks tying the Arduino bridge firmware to the C++ Cytron backend.

The sketch in `arduino/mdds30_bridge/` and `SerialMotorProtocol::CytronMdds30`
must keep speaking the same line protocol, and the customer-facing upload
instructions must stay in the docs surface.
"""
from __future__ import annotations

import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "arduino/mdds30_bridge/mdds30_bridge.ino"


def read(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


class ArduinoBridgeContractTest(unittest.TestCase):
    def test_firmware_ships_with_the_library(self) -> None:
        self.assertTrue(FIRMWARE.is_file(), "Arduino bridge firmware must ship in the repository")
        firmware = FIRMWARE.read_text(encoding="utf-8")
        for token in (
            "Serial.begin(115200)",
            "watchdogTimeoutMs = 300",
            "OK HERMES_MDDS30_BRIDGE_READY",
            "OK WATCHDOG_STOP",
            "constrain(percent, -100, 100)",
        ):
            self.assertIn(token, firmware, f"firmware missing {token}")

    def test_firmware_and_cpp_backend_agree_on_the_protocol(self) -> None:
        firmware = FIRMWARE.read_text(encoding="utf-8")
        backend = read("src/motors_serial.cpp")

        # Move command: the backend emits exactly what the sketch parses.
        self.assertIn('out << "M L=" << left << " R=" << right', backend)
        self.assertIn('line.startsWith("M ")', firmware)
        self.assertIn('line.indexOf("L=")', firmware)
        self.assertIn('line.indexOf("R=")', firmware)

        # Stop command and percent range.
        self.assertIn('writeCommand("STOP\\n")', backend)
        self.assertIn('line == "STOP"', firmware)
        self.assertIn("kCytronMaxCommand = 100", backend)

        # Bridge defaults exposed to callers.
        self.assertIn("config.baud_rate = 115200", backend)
        self.assertIn("SerialMotorProtocol::CytronMdds30", backend)

    def test_keepalive_stays_below_the_firmware_watchdog(self) -> None:
        firmware = FIRMWARE.read_text(encoding="utf-8")
        self.assertIn("watchdogTimeoutMs = 300", firmware)
        # C++ side repeats every 100 ms, and the field runner blocks slower presets.
        self.assertIn("std::chrono::milliseconds(100)", read("src/motors.cpp"))
        self.assertIn("kCytronBridgeWatchdog{300}", read("src/field_runner.cpp"))

    def test_upload_instructions_are_documented(self) -> None:
        guide = read("docs/arduino_mdds30_bridge.md")
        for phrase in (
            "the Arduino firmware must be uploaded before anything works",
            "arduino-cli compile --fqbn arduino:avr:uno arduino/mdds30_bridge",
            "arduino-cli upload",
            "Tools → Board → Arduino UNO",
            "OK HERMES_MDDS30_BRIDGE_READY",
        ):
            self.assertIn(phrase, guide, f"upload guide missing {phrase}")

        # Entry points a customer is likely to open first must point at the guide.
        for source in ("README.md", "docs/motor_module.md", "docs/cytron_motor_hardware_smoke.md"):
            self.assertIn("arduino_mdds30_bridge.md", read(source), f"{source} does not link the upload guide")
        self.assertIn("arduino_mdds30_bridge.md", read("docs/index.html"))


if __name__ == "__main__":
    unittest.main()
