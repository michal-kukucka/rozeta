# GPS module

Public header: `include/rozeta/gps.hpp`

Milestone 1 includes a dependency-free NMEA parser for common Robotour-compatible GPS sources.

## Supported sentences

- `$GPGGA` / compatible GGA: latitude, longitude, altitude, fix quality, satellite count
- `$GPRMC` / compatible RMC: latitude, longitude, speed in knots converted to m/s, course over ground

## Local coordinates

Use `rozeta::geoToLocal(origin, point)` or `gps::toLocal(origin, fix)` to convert GPS fixes into approximate local ENU-like meters suitable for small outdoor courses.

## Future work

- serial GPS receiver implementation
- checksum validation
- HDOP/VDOP fields
- projection strategy for larger maps
