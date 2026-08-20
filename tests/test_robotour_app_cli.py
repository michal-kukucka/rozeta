#!/usr/bin/env python3
"""Contract tests for the robotour_app configuration layer.

The application's promise is that a configuration is inspectable and
reproducible: `--print-config` must emit the whole configuration, and reading
that output back must produce the same configuration again. If it does not,
a preset stops being a faithful record of a run.
"""
from __future__ import annotations

import subprocess
import sys
import tempfile
from pathlib import Path


def run(exe: Path, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(exe), *args], text=True, capture_output=True, check=False
    )


def assert_code(result: subprocess.CompletedProcess[str], expected: int, label: str) -> None:
    if result.returncode != expected:
        raise AssertionError(
            f"{label}: expected exit {expected}, got {result.returncode}\n"
            f"stdout={result.stdout!r}\nstderr={result.stderr!r}"
        )


def keys_of(document: str) -> set[str]:
    keys = set()
    for line in document.splitlines():
        line = line.split("#", 1)[0].strip()
        if "=" in line:
            keys.add(line.split("=", 1)[0].strip())
    return keys


def main() -> int:
    if len(sys.argv) != 4:
        print(
            "usage: test_robotour_app_cli.py /path/to/robotour_app maps.json presets_dir",
            file=sys.stderr,
        )
        return 2

    exe = Path(sys.argv[1])
    catalog = Path(sys.argv[2])
    presets = Path(sys.argv[3])
    catalog_arg = f"map.catalog={catalog}"

    listed = run(exe, "--list-keys")
    assert_code(listed, 0, "--list-keys")
    listed_keys = {line.strip() for line in listed.stdout.splitlines()
                   if line.strip() and not line.startswith("#")}
    if len(listed_keys) < 100:
        raise AssertionError(f"--list-keys advertises only {len(listed_keys)} keys")

    first = run(exe, "--set", catalog_arg, "--print-config")
    assert_code(first, 0, "--print-config")
    printed_keys = keys_of(first.stdout)

    # Everything --list-keys advertises has to appear in --print-config, or a
    # setting exists that a preset file could never record.
    missing = listed_keys - printed_keys - {
        key for key in listed_keys if f"# {key} =" in first.stdout
    }
    if missing:
        raise AssertionError(f"keys advertised but not printed: {sorted(missing)}")

    with tempfile.TemporaryDirectory() as tmp:
        preset_path = Path(tmp) / "resolved.preset"
        preset_path.write_text(first.stdout, encoding="utf-8")

        second = run(exe, "--preset", str(preset_path), "--print-config")
        assert_code(second, 0, "--print-config after reload")
        if second.stdout != first.stdout:
            raise AssertionError("--print-config is not a fixed point under reload")

        # An override must beat the file, whichever layer owns the key.
        overridden = run(
            exe,
            "--preset", str(preset_path),
            "--set", "follower.cruise_speed=0.31",
            "--set", "app.log_every=7",
            "--print-config",
        )
        assert_code(overridden, 0, "override")
        if "follower.cruise_speed = 0.31" not in overridden.stdout:
            raise AssertionError("library override did not take effect")
        if "app.log_every = 7" not in overridden.stdout:
            raise AssertionError("application override did not take effect")

    # The shipped presets must load, and a hardware one must refuse to run
    # without an E-STOP rather than opening a device.
    for preset in sorted(presets.glob("*.preset")):
        loaded = run(exe, "--preset", str(preset), "--set", catalog_arg, "--print-config")
        assert_code(loaded, 0, f"load {preset.name}")

    hardware = run(exe, "--base", "buchlovice", "--set", catalog_arg,
                   "--dry-run", "--set", "app.quiet=true")
    if hardware.returncode == 0:
        raise AssertionError("hardware dry run passed without an E-STOP configured")
    if "E-STOP" not in hardware.stderr:
        raise AssertionError(f"missing E-STOP diagnostic: {hardware.stderr!r}")

    # A dry run must be read-only: it is meant to be scripted against a robot
    # that is not powered up, and a check that leaves files behind would put an
    # SVG of a route nobody drove next to the record of the last real run.
    with tempfile.TemporaryDirectory() as tmp:
        outputs = [
            "app.svg=run.svg",
            "app.telemetry_csv=ticks.csv",
            "app.event_log=events.csv",
            "app.log_csv=log.csv",
            "app.preset_out=resolved.preset",
        ]
        args = ["--set", catalog_arg, "--set", "map.id=castle_park", "--dry-run"]
        for output in outputs:
            args += ["--set", output]
        dry = subprocess.run([str(exe), *args], text=True, capture_output=True,
                             check=False, cwd=tmp)
        assert_code(dry, 0, "read-only dry run")
        left_behind = sorted(p.name for p in Path(tmp).iterdir())
        if left_behind:
            raise AssertionError(f"dry run wrote files: {left_behind}")

        # ...unless the plan picture is asked for explicitly.
        planned = subprocess.run(
            [str(exe), *args, "--plan-svg", "plan.svg"],
            text=True, capture_output=True, check=False, cwd=tmp,
        )
        assert_code(planned, 0, "dry run with --plan-svg")
        if sorted(p.name for p in Path(tmp).iterdir()) != ["plan.svg"]:
            raise AssertionError("--plan-svg did not write exactly the plan picture")

    # Headless, an operator gate nobody can release must not hang the run:
    # there is no window to press S in, so the run starts anyway.
    gated = run(exe, "--set", catalog_arg, "--set", "map.id=castle_park",
                "--set", "app.auto_start=false", "--set", "app.log_every=0",
                "--set", "app.quiet=true")
    assert_code(gated, 0, "headless run with app.auto_start=false")

    assert_code(run(exe, "--set", "no.such.key=1"), 3, "unknown key")
    assert_code(run(exe, "--set", "malformed"), 2, "malformed --set")
    assert_code(run(exe, "--base", "nonsense"), 2, "unknown base preset")
    assert_code(run(exe, "--help"), 0, "--help")

    print("robotour_app CLI contract OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
