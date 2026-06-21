#!/usr/bin/env python3
"""Smoke-test the field_operator_wizard example CLI contract."""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path


def run(exe: Path, script: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [str(exe), "--script", script],
        text=True,
        capture_output=True,
        check=False,
    )


def assert_code(result: subprocess.CompletedProcess[str], expected: int, label: str) -> None:
    if result.returncode != expected:
        raise AssertionError(
            f"{label}: expected exit {expected}, got {result.returncode}\n"
            f"stdout={result.stdout!r}\nstderr={result.stderr!r}"
        )


def assert_no_controls(text: str, label: str) -> None:
    bad = [ch for ch in text if (ord(ch) < 0x20 and ch not in "\t\n") or ord(ch) == 0x7F]
    bad += [ch for ch in text if 0x80 <= ord(ch) <= 0x9F]
    if bad:
        encoded = text.encode("unicode_escape").decode("ascii")
        raise AssertionError(f"{label}: control bytes leaked: {encoded}")


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: test_field_operator_wizard_cli.py /path/to/field_operator_wizard", file=sys.stderr)
        return 2

    exe = Path(sys.argv[1])
    assert_code(run(exe, "continue,continue,continue,continue"), 0, "complete script")
    assert_code(run(exe, "continue"), 1, "incomplete script")
    assert_code(run(exe, "quit"), 1, "abort script")

    unknown = run(exe, "nope")
    assert_code(unknown, 1, "unknown script token")
    assert_no_controls(unknown.stderr, "unknown-token stderr")

    escape = run(exe, "bad\x1b[2J")
    assert_code(escape, 1, "ESC script token")
    assert_no_controls(escape.stderr, "ESC-token stderr")

    c1 = run(exe, "bad\u009b[2J")
    assert_code(c1, 1, "C1 script token")
    assert_no_controls(c1.stderr, "C1-token stderr")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
