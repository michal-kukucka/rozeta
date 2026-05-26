# Mission module

Public header: `include/rozeta/mission.hpp`

The mission module closes M3 — QR mission target intake for Buchlovice/Robotour flows. It keeps QR decoding and text parsing separate so tests and default CI do not need camera hardware or OpenCV.

## Target parser

`parseMissionTarget` converts QR/string payloads into `mission::MissionTarget` with a validated `GeoCoordinate`.

Supported payload forms:

- `geo:lat,lon`
- `gps lat,lon`
- `lat: 48.111; lon: 17.222`
- `N 48.333 E 17.444`
- `S 12.5 W 45.75`

Invalid text returns `ParseError`. Coordinates outside latitude `[-90, 90]` or longitude `[-180, 180]` return `InvalidArgument`.

```cpp
rozeta::mission::MissionTarget target;
auto status = rozeta::mission::parseMissionTarget("geo:48.1234,17.5678", target);
```

## QR source seam

`QrDecoder` is a small dependency-injected backend interface. `parseMissionTargetFromQr` validates a grayscale image, asks the decoder for text, then reuses `parseMissionTarget`.

```cpp
class MyQrDecoder final : public rozeta::mission::QrDecoder {
public:
    rozeta::Status decode(const rozeta::mission::QrImage& image, std::string& payload) override;
};
```

An OpenCV QR hook is declared behind `ROZETA_WITH_OPENCV` as `OpenCvQrDecoder`. The default build remains dependency-free; the M3 public seam and fake-decoder tests are already in place for the later optional OpenCV QR adapter.

## Verification

Covered by `tests/test_mission.cpp`:

- `geo:lat,lon` and `gps lat,lon` parsing.
- labeled `lat`/`lon` payloads with whitespace and semicolons.
- `N ... E ...` and `S ... W ...` hemisphere signs.
- SPayD-like/non-GPS rejection and malformed/out-of-range coordinates.
- QR source seam with a fake `QrDecoder` backend.
