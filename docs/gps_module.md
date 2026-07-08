# GPS module

Public header: `include/rozeta/gps.hpp`

The GPS module now covers the full lightweight Robotour path from serial/file/network GPS input to validated `GpsFix` data. It stays dependency-free and uses POSIX serial and loopback-testable TCP/UDP sockets for hardware-adjacent reads.

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

Receiver-mode NMEA parsing requires checksummed sentences. Invalid checksums are rejected and counted by `GpsReceiverStats::checksum_failures`.

## Network payload parsing

M4 adds `gps::parseGpsPayload` for iPhone-style GPS feeds that do not always send NMEA. The parser normalizes all successful inputs into `GpsFix`:

- NMEA `$GPGGA` / `$GPRMC` lines with checksums,
- JSON packets like `{ "lat": 48.333, "lon": 17.444 }`,
- plain decimal coordinate lines like `48.333,17.444`.

All non-NMEA formats still validate latitude in `[-90, 90]` and longitude in `[-180, 180]`.

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
On Windows, use normal COM names such as `COM3` for ports below 10 and the Win32 device prefix such as
`\\.\COM10` for two-digit ports when a driver exposes that spelling. The same `SerialGpsReceiver` API is used;
only the device string changes.

`readFix()` returns `std::nullopt` on timeout, parse errors or hardware errors. Inspect `lastStatus()` and `stats()` for diagnostics.

## Network receiver

`NetworkGpsReceiver` covers the Buchlovice iPhone TCP/UDP paths without pulling networking into the parser. Configure the protocol, IPv4 host and port, then call `readFix()` with finite timeouts:

```cpp
rozeta::gps::NetworkGpsReceiverConfig config;
config.protocol = rozeta::gps::NetworkGpsProtocol::Udp;
config.host = "127.0.0.1";
config.port = 5005;

rozeta::gps::NetworkGpsReceiver receiver(config);
if (receiver.open().ok()) {
    auto fix = receiver.readFix();
}
```

TCP feeds are line-buffered so fragmented newline-delimited messages can arrive across multiple packets. UDP treats each datagram as one GPS payload. TCP sockets close and can reconnect after `reconnect_backoff`; UDP reads return `Timeout` when no packet arrives before `read_timeout`.

## Example usage

Payload mode works without GPS hardware or sockets:

```bash
./build/examples/gps_network_reader --payload '{"lat": 48.1486, "lon": 17.1077}'
```

TCP/UDP modes listen/read one fix from an iPhone-style feed:

```bash
./build/examples/gps_network_reader --udp 127.0.0.1:5005
./build/examples/gps_network_reader --tcp 127.0.0.1:5005
```

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
