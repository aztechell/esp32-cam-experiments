# ESP32-CAM AI Thinker

Russian version: [README.md](README.md)

![Web UI](docs/web-ui.png)

A set of firmware experiments for the AI Thinker ESP32-CAM with an OV2640
camera. The project started as a quick board/camera check and grew into a few
separate modes: web preview, manual mosaic reading, and an automatic
single-shot reader for robot use.

## What's Included

- `diagnostic`: minimal serial diagnostics for the board, camera, and PSRAM. No
  Wi-Fi.
- `web_photo`: Wi-Fi web UI with JPEG-polling live view, camera controls, saved
  NVS settings, and reset-to-defaults.
- `mosaic_reader`: manual 4x3 mosaic setup. The ESP32 captures raw RGB565,
  lets AWB/AEC/AGC settle with warm-up frames, samples 12 configured points, and
  classifies `yellow`, `green`, `blue`, or `white` on-device.
- `mosaic_reader_v2`: single-shot robot detector. One HTTP request captures one
  frame, finds the mosaic, and returns 12 colors.

All firmware variants live in one PlatformIO project, but each one is built as
its own environment. They are isolated by `build_src_filter` and do not compile
together.

## Quick Start

Python 3 and Git are required:

```powershell
python --version
git --version
```

Install PlatformIO and esptool locally:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install platformio esptool
```

Check the install:

```powershell
.\.venv\Scripts\pio.exe --version
```

The project uses the local `.venv` and `.platformio` directories. Both are
ignored by Git, so downloaded tools and packages do not end up in the repo.

## Commands

List available serial ports:

```powershell
.\esp32cam.cmd ports
```

Check that the ESP32 responds on a port:

```powershell
.\esp32cam.cmd chip -Ports COM7
```

Build firmware:

```powershell
.\esp32cam.cmd build -Environment diagnostic
.\esp32cam.cmd build -Environment web_photo
.\esp32cam.cmd build -Environment mosaic_reader
.\esp32cam.cmd build -Environment mosaic_reader_v2
```

Upload firmware:

```powershell
.\esp32cam.cmd upload -Port COM7 -Environment diagnostic
.\esp32cam.cmd upload -Port COM7 -Environment web_photo
.\esp32cam.cmd upload -Port COM7 -Environment mosaic_reader
.\esp32cam.cmd upload -Port COM7 -Environment mosaic_reader_v2
```

Open the serial monitor:

```powershell
.\esp32cam.cmd monitor -Port COM7
```

If upload does not start, put the board into bootloader mode: hold `BOOT` or
connect `IO0` to `GND`, press `RST`, run upload, then release `BOOT` and press
`RST` again.

## Wi-Fi Secrets

Wi-Fi firmware does not store real SSIDs or passwords in Git. Before building a
Wi-Fi environment, copy the matching example:

```powershell
Copy-Item src\web_photo\wifi_secrets.example.h src\web_photo\wifi_secrets.h
Copy-Item src\mosaic_reader\wifi_secrets.example.h src\mosaic_reader\wifi_secrets.h
Copy-Item src\mosaic_reader_v2\wifi_secrets.example.h src\mosaic_reader_v2\wifi_secrets.h
```

Fill `WIFI_SSID` and `WIFI_PASSWORD` in the needed `wifi_secrets.h`. These files
are ignored by Git.

## Diagnostic

`diagnostic` is the fastest way to check whether the board is alive and the
camera is visible. It does not use Wi-Fi; all output goes to the serial monitor.

Successful output should include:

```text
camera init ok
capture ok: <bytes> bytes
jpeg markers: ok
probe done
```

The firmware also prints a heartbeat every 2 seconds:

```text
heartbeat: <ms> ms, camera: ready, count: <n>
```

Diagnostics also report PSRAM state, free heap, the AI Thinker pin map, and the
camera SCCB/I2C scan result.

## Web Photo

`web_photo` connects to Wi-Fi, prints its IP address to the serial monitor, and
starts an HTTP server on port `80`.

The UI has one live display and a camera settings panel: JPEG quality,
brightness, contrast, saturation, sharpness, white balance, exposure, gain,
mirror, flip, lens correction, and optional warm-up frame discard. Live view
uses JPEG polling. The default is `2 fps`; `1`, `5`, `8`, and `10 fps` are also
available.

Settings are stored in ESP32 NVS/Preferences and restored after reboot. The UI
reads `/status` first, applies saved values to the controls, and writes NVS only
when a value actually changes.

HTTP API:

- `GET /`: web UI.
- `GET /frame?res=qqvga|qvga|vga&fps=1|2|5|8|10`: one JPEG frame for live-view.
- `GET` or `POST /capture?res=qqvga|qvga|vga`: compatibility alias for one JPEG
  frame.
- `GET /status`: IP, PSRAM, camera state, active and saved resolution, counters,
  sensor settings, and the last error.
- `GET` or `POST /settings/reset`: reset saved settings.

If PSRAM does not work, the firmware uses one frame buffer in DRAM. `QQVGA` and
`QVGA` are the practical modes for that setup. `VGA` is exposed for testing, but
may return HTTP 503 without PSRAM.

## Mosaic Reader

![Mosaic Reader UI](docs/mosaic-reader.png)

`mosaic_reader` is useful when the camera is mounted consistently and the grid
can be configured once by hand. The UI moves corner handles `1`, `4`, `9`, and
`12`; the other points are computed as a straight 4x3 grid. The browser only
shows the raw frame and sends settings. Recognition runs on the ESP32.

Firmware behavior:

- captures raw `RGB565`;
- discards warm-up/stale frames so auto exposure and white balance can settle;
- samples a small patch around each of the 12 points;
- ignores near-black frame pixels when a point lands close to a cell border;
- classifies using normalized ratios `R/(R+G+B)`, `G/(R+G+B)`, `B/(R+G+B)`.

HTTP API:

- `GET /`: setup UI with raw RGB565 preview, grid, corner handles, calibration,
  and a 3x4 result table.
- `GET /frame?res=qqvga|qvga&radius=0..10&warmup=0..8`: one RGB565 frame plus
  result headers.
- `GET /result?res=qqvga|qvga&radius=0..10&warmup=0..8`: JSON result only.
- `GET /status`: IP, camera, PSRAM, resolution, radius, warm-up, counters,
  points, calibration status, and last result.
- `POST /points`: save 12 normalized point coordinates in NVS.
- `POST /calibrate?point=0..11&color=yellow|green|blue|white`: sample the
  selected point and save calibration for that color.
- `POST /settings/reset`: reset points, radius, warm-up, resolution, and
  calibration.

## Mosaic Reader v2

![Mosaic Reader v2 UI](docs/mosaic-reader-v2.png)

`mosaic_reader_v2` is built for the robot flow: the robot arrives at whatever
pose it gets, the ESP32 captures one frame, finds the mosaic, and returns 12
values. There is no tracking, no physical marker, and no manual 4-corner model.

Current detector flow:

- captures `QQVGA RGB565` in DRAM;
- extracts colored and white cell blob candidates;
- fits a 4x3 lattice from blobs when possible: `source: "blob_lattice"`;
- falls back to a full-frame grid search using dark separator lines and
  colored/white cell centers: `source: "grid_search"`;
- uses a projective grid for rotated or perspective-distorted boards;
- classifies colors from NVS calibration samples for `yellow`, `green`, `blue`,
  and `white`.

If the detector is uncertain, it still returns 12 colors with
`status: "best_effort"` and low `confidence`. HTTP 503 is reserved for actual
camera or capture failures.

HTTP API:

- `GET /`: debug UI with preview, detected grid, matched blob overlay, and a
  result table.
- `GET /preview`: fast raw RGB565 frame without recognition, used for aiming.
- `GET /frame`: raw RGB565 frame with full recognition. The UI reads the full
  result from `/status` to avoid oversized HTTP headers on the ESP32.
- `GET /result`: runtime JSON for the robot: `status`, `found`, `confidence`,
  `source`, `pattern`, `corners`, `grid`, `points`.
- `GET /status`: camera state, counters, calibration, and `last_result`.
- `POST /model`: compatibility no-op; manual models are disabled in v2.
- `POST /calibrate?cell=0..11&color=yellow|green|blue|white`: update
  calibration from the selected cell.
- `POST /settings/reset`: reset calibration, warm-up, and detector state.

Useful fields while tuning:

- `source`: `blob_lattice` or `grid_search`.
- `found`: geometry is considered reliable.
- `complete`: all 12 sample points are inside the frame.
- `confidence`: overall confidence.
- `pattern`: 12 colors in row-major order, `r1c1..r3c4`.
- `points[n].confidence`, `coverage`, `blob_match`, `rgb`: per-cell diagnostics.

## Useful Links

- [WRO 2026 Senior Randomizer](https://legorobot.com.tw/WRO2026-SeniorRandomizer/)
  helps test recognition against realistic randomized Senior patterns.

## Hardware and Limitations

Tested with an AI Thinker ESP32-CAM with OV2640 and a CH340 USB-UART adapter.
The working port for this setup was `COM7`.

Things to keep in mind:

- many ESP32-CAM clone boards look like AI Thinker modules, but PSRAM may be
  missing or broken;
- without working PSRAM, prefer `QQVGA` or `QVGA`;
- `VGA` can be tested, but may fail with HTTP 503 without PSRAM;
- live view uses polling so camera settings do not get blocked by a long MJPEG
  loop.

## If The Monitor Is Empty

First check the real port:

```powershell
.\esp32cam.cmd ports
```

If upload prints `Failed to connect to ESP32: No serial data received`:

1. Hold `BOOT` or connect `IO0` to `GND`.
2. Press and release `RST` while `BOOT/IO0` is active.
3. Run upload, for example:

   ```powershell
   .\esp32cam.cmd upload -Port COM7 -Environment web_photo
   ```

4. Release `BOOT` or disconnect `IO0-GND`.
5. Open the monitor:

   ```powershell
   .\esp32cam.cmd monitor -Port COM7
   ```

6. Press and release `RST`.
