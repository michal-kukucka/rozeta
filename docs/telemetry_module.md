# Telemetry module

`include/rozeta/telemetry.hpp` defines Rozeta's replayable telemetry contracts for Robotour/Buchlovice runs. The module stays dependency-free so field laptops can normalize logs, compare decisions in CI and replay UI snapshots without opening hardware.

## Replay CSV schema

The stable replay schema is `rozeta.telemetry.v1`. `replayCsvHeader()`, `parseReplayLog()`, `loadReplayLog()`, `replayNavigation()` and `replayUiSnapshots()` parse strict comma-delimited replay fixtures and rebuild navigation/UI outputs deterministically.

The replay parser is intentionally fail-closed: quoted fields, embedded commas, missing columns, unsupported schema names, partial numeric parses and non-finite values return parse errors instead of silently producing unsafe replay data.

## Mission tick CSV

`MissionTickSample` stores the per-cycle mission facts used by the M13 telemetry logger and M27 converter:

- timestamp, phase and leg
- current GPS and target coordinates
- dark/diff obstacle coverage
- obstacle flag and source
- route cue text
- left/right motor command
- bypass direction

`missionTickCsvHeader()` and `formatMissionTickCsv()` produce the normalized mission-tick CSV row used by field replay and CI comparisons.

## M27 — Buchlovice telemetry converter

M27 adds `BuchloviceTelemetryConvertResult` and `convertBuchloviceTelemetry()` so legacy Buchlovice/Robotour text logs can be converted into Rozeta telemetry without Python or hardware dependencies.

Accepted line grammar is whitespace-separated `key=value` records:

```text
# comments are ignored
tick ts=100 phase=to_loading leg=1 gps=48.800100,17.390200 target=48.800500,17.390900 dark=0.25 diff=0.50 obstacle=1 obstacle_source=rgb_dark route_cue=Turn_left_in_7_m motor=0.40,0.35 bypass=-1
event ts=120 type=qr_scanned detail=geo:48.8005;17.3909
```

The `tick ts=100` line becomes one `MissionTickSample` with `timestamp_ms=100`; the `event` line becomes one `MissionEventRecord`. Operator text tokens such as `phase`, `obstacle_source`, `route_cue=Turn_left_in_7_m` and event `detail` convert underscores to spaces for readable HUD/CSV text while rejecting commas, control characters and spreadsheet formula prefixes (`=`, `+`, `-`, `@`) so `formatMissionTickCsv()` remains line-oriented.

Strict validation rules:

- unknown record kinds fail with `ParseError`
- malformed tokens and duplicate keys fail with `ParseError`
- missing required keys fail with `ParseError`
- non-finite numbers such as `nan` or `inf` fail with `ParseError`
- GPS, target and motor pairs must contain exactly one comma
- GPS and target coordinates must stay inside latitude/longitude bounds
- text fields reject commas, control characters and spreadsheet formula prefixes
- empty/comment-only input fails with `ParseError`

## Executable converter

`buchlovice_telemetry_converter` reads a legacy log from a file path or stdin, calls `convertBuchloviceTelemetry()` and writes normalized mission-tick CSV to stdout. Events are counted on stderr so shell pipelines can keep CSV output clean.

Example:

```bash
printf 'tick ts=100 phase=returning leg=3 gps=48.8,17.3 target=48.7,17.2 dark=0 diff=0 obstacle=0 obstacle_source=clear route_cue=Continue_straight motor=0.10,0.10 bypass=0\n' \
  | ./build-final/examples/buchlovice_telemetry_converter
```
