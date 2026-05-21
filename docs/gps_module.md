# GPS module

Public header: `include/rozeta/gps.hpp`

The GPS module now covers the full lightweight Robotour path from serial/file NMEA input to validated `GpsFix` data. It stays dependency-free and uses the M1 POSIX serial transport for hardware reads.

## Supported sentences

- `$GPGGA` / compatible GGA: latitude, longitude, altitude, fix quality, satellite count
- `$GPRMC` / compatible RMC: latitude, longitude, speed in knots converted to m/s, course over ground

`NmeaParser::parseLineDetailed()` returns structured parse status so applications can distinguish unsupported sentences, invalid fixes, malformed numbers, missing checksums and checksum mismatches. `parseLine()` remains as the compatibility helper that returns only a `GpsFix`.

## Checksum validation

Rozeta validates standard NMEA checksums by XORing the bytes between `$` and `*` and comparing them with the two hex characters after `*`.

```cpp
auto validation = rozeta::gps::validateNmeaSentence(
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47");
if (!validation.ok()) {
    // inspect validation.code and validation.message
}
```

Receiver-mode parsing requires checksummed sentences. Invalid checksums are rejected and counted by `GpsReceiverStats::checksum_failures`.

## Stream buffering

Serial reads are arbitrary byte chunks, not guaranteed NMEA lines. `NmeaStreamBuffer` handles the common field reality:

- fragmented messages across multiple reads,
- multiple NMEA messages in one read,
- CRLF or LF line endings,
- garbage before the next `$`,
- bounded pending buffers to avoid unbounded growth.

## Serial receiver

`SerialGpsReceiver` combines `internal::SerialPort`, `NmeaStreamBuffer` and `NmeaParser`.

```cpp
rozeta::gps::GpsReceiverConfig config;
config.device = "/dev/serial/by-id/usb-Your_GPS";
config.baud_rate = 9600;

rozeta::gps::SerialGpsReceiver receiver(config);
auto status = receiver.open();
if (status.ok()) {
    auto fix = receiver.readFix();
}
```

Default serial settings target common GPS modules: 9600 baud, 8N1 raw POSIX mode, no flow control and finite read timeouts.

`readFix()` returns `std::nullopt` on timeout, parse errors or hardware errors. Inspect `lastStatus()` and `stats()` for diagnostics.

## Example usage

Sample-file mode works without GPS hardware:

```bash
./build/examples/gps_serial_reader --sample tests/fixtures/gps/robotour_sample.nmea
```

Hardware mode:

```bash
./build/examples/gps_serial_reader --device /dev/serial/by-id/usb-Your_GPS --baud 9600
```

## Linux device setup

Prefer stable device paths:

```bash
ls -l /dev/serial/by-id/
```

Common dynamic device paths are `/dev/ttyUSB0` and `/dev/ttyACM0`.

If opening the device fails with permission denied, add your user to the serial device group and log out/in:

```bash
sudo usermod -aG dialout "$USER"
```

Troubleshooting checklist:

- `HardwareUnavailable`: wrong path, missing device, or permissions.
- `Timeout`: port opened but no complete NMEA sentence arrived before the timeout.
- checksum failures: wrong baud rate, noisy wiring, wrong device, or non-NMEA/proprietary output.
- no valid fix: GPS may need outdoor sky view or time to acquire satellites.

## Local coordinates

Use `rozeta::geoToLocal(origin, point)` or `gps::toLocal(origin, fix)` to convert GPS fixes into approximate local ENU-like meters suitable for small outdoor courses.

## Future work

- HDOP/VDOP fields.
- UBX/proprietary binary GPS protocols behind separate optional adapters.
- Projection strategy for larger maps.
