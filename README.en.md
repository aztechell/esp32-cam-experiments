# ESP32-CAM AI Thinker

Russian version: [README.md](README.md)

![Web UI](docs/web-ui.png)

This project contains four separate PlatformIO firmware environments for an
AI Thinker ESP32-CAM with an OV2640 camera:

- `diagnostic`: serial-only board, camera, and PSRAM diagnostics, no Wi-Fi.
- `web_photo`: Wi-Fi web UI with JPEG-polling live view, camera controls,
  saved settings, and reset-to-defaults.
- `mosaic_reader`: Wi-Fi setup UI for a 4x3 mosaic reader. The ESP32 captures
  raw RGB565 frames, stabilizes camera auto modes with warm-up frames, samples
  12 points on an adjustable grid, and classifies yellow, green, blue, or white
  on-device using normalized color.
- `mosaic_reader_v2`: single-shot robot detector. The ESP32 searches for the
  mosaic in one RGB565 frame and returns 12 best-effort colors.

## Local Setup

Python 3 and Git are required. Check that both are available:

```powershell
python --version
git --version
```

Create a local virtual environment and install the tools:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install platformio esptool
```

Check PlatformIO:

```powershell
.\.venv\Scripts\pio.exe --version
```

PlatformIO and esptool are used from the local `.venv`. PlatformIO stores its
packages in the local `.platformio` directory, so the project does not need to
write dependencies into the user's home directory. `.venv`, `.platformio`, and
`.pio` are not committed.

Build the diagnostic firmware:

```powershell
.\esp32cam.cmd build -Environment diagnostic
```

List currently openable serial ports:

```powershell
.\esp32cam.cmd ports
```

Check the ESP32 port without flashing:

```powershell
.\esp32cam.cmd chip -Ports COM7
```

Upload the diagnostic firmware:

```powershell
.\esp32cam.cmd upload -Port COM7 -Environment diagnostic
```

Build the web firmware:

```powershell
.\esp32cam.cmd build -Environment web_photo
```

Upload the web firmware:

```powershell
.\esp32cam.cmd upload -Port COM7 -Environment web_photo
```

Build the mosaic reader firmware:

```powershell
.\esp32cam.cmd build -Environment mosaic_reader
```

Upload the mosaic reader firmware:

```powershell
.\esp32cam.cmd upload -Port COM7 -Environment mosaic_reader
```

Build the v2 single-shot detector:

```powershell
.\esp32cam.cmd build -Environment mosaic_reader_v2
```

Upload the v2 single-shot detector:

```powershell
.\esp32cam.cmd upload -Port COM7 -Environment mosaic_reader_v2
```

Open the serial monitor:

```powershell
.\esp32cam.cmd monitor -Port COM7
```

## Web Firmware

`web_photo` connects to the configured Wi-Fi network, prints its IP address to
the serial monitor, and starts an HTTP server on port `80`.

Before building for your own network, copy the secrets example:

```powershell
Copy-Item src\web_photo\wifi_secrets.example.h src\web_photo\wifi_secrets.h
```

Then fill `WIFI_SSID` and `WIFI_PASSWORD` in
`src\web_photo\wifi_secrets.h`. This file is listed in `.gitignore` and must not
be committed.

Endpoints:

- `GET /`: web UI.
- `GET /frame?res=qqvga|qvga|vga&fps=1|2|5|8|10`: one JPEG frame for the
  live-view polling loop.
- `GET` or `POST /capture?res=qqvga|qvga|vga`: compatibility alias for one
  JPEG frame.
- `GET /status`: JSON with IP, PSRAM, camera state, active/saved resolution,
  frame count, saved sensor settings, and the last camera error.
- `GET` or `POST /settings/reset`: reset saved web-firmware settings.

The UI shows one live display and a camera settings panel: JPEG quality,
brightness, contrast, saturation, sharpness, white balance, exposure, gain,
mirror, flip, lens correction, and optional warm-up frame discard. Live view
starts at `2 fps`; `1`, `5`, `8`, and `10 fps` can be selected without
reflashing.

`web_photo` settings are stored in ESP32 NVS/Preferences and restored after
reboot. The UI reads `/status` before the first frame, applies saved values to
the controls, and writes NVS only when a setting actually changes.

If PSRAM does not work, the web firmware uses one frame buffer in DRAM. The
practical modes for this board are usually `QQVGA` and `QVGA`; `VGA` is exposed
in the UI, but may return HTTP 503 if there is not enough memory.

## Mosaic Reader Firmware

![Mosaic Reader UI](docs/mosaic-reader.png)

The screenshot shows the setup UI for the main mosaic reader firmware: raw
RGB565 frame with the configured grid on the left, 3x4 result, confidence,
warm-up, and color calibration on the right. The UI moves the corner handles
`1`, `4`, `9`, and `12`; the other 8 points are computed automatically as a
straight 4x3 grid.

`mosaic_reader` connects to Wi-Fi and starts a setup UI on port `80`. Copy
`src\mosaic_reader\wifi_secrets.example.h` to
`src\mosaic_reader\wifi_secrets.h` and fill in `WIFI_SSID` and
`WIFI_PASSWORD` before building for your network.

Endpoints:

- `GET /`: setup UI with raw RGB565 preview, red overlay grid, corner handles,
  calibration controls, warm-up setting, and a 3x4 result table.
- `GET /frame?res=qqvga|qvga&radius=0..10&warmup=0..8`: captures one RGB565
  frame, discards warm-up/stale frames, recognizes the 12 points on the ESP32,
  returns raw frame bytes, and includes width, height, and result headers.
- `GET /result?res=qqvga|qvga&radius=0..10&warmup=0..8`: captures and returns
  only JSON recognition output.
- `GET /status`: IP, camera, PSRAM, resolution, radius, warm-up, counters,
  points, calibration status, and last result.
- `POST /points`: saves 12 normalized point coordinates in ESP32 NVS. The UI
  usually sends the grid computed from the 4 corner handles.
- `POST /calibrate?point=0..11&color=yellow|green|blue|white`: samples the
  selected point and stores that color calibration in NVS.
- `POST /settings/reset`: restores default points, sample radius, warm-up,
  resolution, and default calibration samples.

Recognition behavior:

- each sample is averaged over a small area around the point; near-black frame
  pixels are ignored so a slight hit on the cell border does not poison the
  average;
- classification uses normalized ratios `R/(R+G+B)`, `G/(R+G+B)`,
  `B/(R+G+B)`, which makes the result less sensitive to overall brightness;
- `Warm-up frames` defaults to `4`, giving AWB/AEC/AGC time to settle before
  the working frame is analyzed.

The browser does not classify colors. It only displays the raw frame and sends
calibration/point changes; recognition happens on the ESP32 so the result path
can later be reused for I2C output.

## Mosaic Reader v2

`mosaic_reader_v2` is for the robot flow: one request captures one frame,
searches for the mosaic from scratch, and returns 12 colors. It does not use
tracking or physical markers.

Before building for your network:

```powershell
Copy-Item src\mosaic_reader_v2\wifi_secrets.example.h src\mosaic_reader_v2\wifi_secrets.h
```

Endpoints:

- `GET /`: debug/setup UI with raw RGB565 frame, detected grid, 4 initial model
  corners, and result table.
- `GET /frame`: one RGB565 frame plus an `X-Mosaic-Result` header for the UI.
- `GET /result`: runtime JSON for the robot: `status`, `found`, `confidence`,
  `pattern`, `corners`, `points`.
- `POST /model`: save the 4 initial search-area corners.
- `POST /calibrate?cell=0..11&color=yellow|green|blue|white`: update
  calibration from the selected cell.
- `POST /settings/reset`: reset model and calibration.

When the detector is uncertain, it still returns 12 colors with
`status: "best_effort"` and low `confidence`. HTTP 503 is reserved for real
camera/capture failures.

## Hardware and Limitations

Tested with an AI Thinker ESP32-CAM with OV2640 and a CH340 USB-UART adapter.
The working port for this setup was `COM7`.

Important limitations:

- many ESP32-CAM clone boards look like AI Thinker modules, but PSRAM may be
  missing or broken;
- without working PSRAM, prefer `QQVGA` or `QVGA`;
- `VGA` is exposed for testing, but may fail with HTTP 503 without PSRAM;
- live view uses JPEG polling so camera settings remain responsive even without
  PSRAM.

## Bootloader Mode

If upload cannot connect, put the board into bootloader mode:

1. Connect `IO0` to `GND` or hold `BOOT`.
2. Press and release `RST`.
3. Run the upload command.
4. Disconnect `IO0` from `GND` or release `BOOT`.
5. Press `RST` again to start the firmware.

Successful diagnostic serial output should include:

```text
camera init ok
capture ok: <bytes> bytes
jpeg markers: ok
probe done
```

The diagnostic firmware also prints a heartbeat every 2 seconds:

```text
heartbeat: <ms> ms, camera: ready, count: <n>
```

## If The Monitor Is Empty

First check the real openable port:

```powershell
.\esp32cam.cmd ports
```

For this board, the working port was `COM7`.

If upload prints `Failed to connect to ESP32: No serial data received`:

1. Hold `BOOT` or connect `IO0` to `GND`.
2. Press and release `RST` while `BOOT/IO0` is active.
3. Run `.\esp32cam.cmd upload -Port COM7 -Environment web_photo`.
4. Release `BOOT` or disconnect `IO0-GND`.
5. Open `.\esp32cam.cmd monitor -Port COM7`.
6. Press and release `RST`.
