#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <math.h>
#include "esp_camera.h"
#include "esp_err.h"
#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif

namespace {

#ifndef WIFI_SSID
#define WIFI_SSID "CHANGE_ME"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "CHANGE_ME"
#endif

constexpr char WIFI_SSID_VALUE[] = WIFI_SSID;
constexpr char WIFI_PASSWORD_VALUE[] = WIFI_PASSWORD;
constexpr char SETTINGS_NAMESPACE[] = "mosaic_v1";
constexpr uint32_t SETTINGS_VERSION = 1;

constexpr int POINT_COUNT = 12;
constexpr int GRID_ROWS = 3;
constexpr int GRID_COLS = 4;
constexpr int COLOR_COUNT = 4;
constexpr int DEFAULT_RESOLUTION_INDEX = 0;
constexpr int SAFE_RESOLUTION_INDEX = 0;
constexpr int DEFAULT_SAMPLE_RADIUS = 3;
constexpr int DEFAULT_WARMUP_FRAMES = 4;
constexpr int STALE_FRAME_DISCARDS = 1;

constexpr int PWDN_GPIO_NUM = 32;
constexpr int RESET_GPIO_NUM = -1;
constexpr int XCLK_GPIO_NUM = 0;
constexpr int SIOD_GPIO_NUM = 26;
constexpr int SIOC_GPIO_NUM = 27;

constexpr int Y9_GPIO_NUM = 35;
constexpr int Y8_GPIO_NUM = 34;
constexpr int Y7_GPIO_NUM = 39;
constexpr int Y6_GPIO_NUM = 36;
constexpr int Y5_GPIO_NUM = 21;
constexpr int Y4_GPIO_NUM = 19;
constexpr int Y3_GPIO_NUM = 18;
constexpr int Y2_GPIO_NUM = 5;
constexpr int VSYNC_GPIO_NUM = 25;
constexpr int HREF_GPIO_NUM = 23;
constexpr int PCLK_GPIO_NUM = 22;

struct ResolutionOption {
  const char *key;
  const char *label;
  framesize_t frameSize;
  int width;
  int height;
};

struct PointCoord {
  uint16_t x = 0;
  uint16_t y = 0;
};

struct RgbColor {
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;

  RgbColor() = default;
  RgbColor(uint8_t red, uint8_t green, uint8_t blue)
      : r(red), g(green), b(blue) {}
};

struct NormalizedColor {
  float r = 0.0f;
  float g = 0.0f;
  float b = 0.0f;
  bool valid = false;
};

struct CalibrationSample {
  RgbColor rgb;
  bool valid = false;
  bool user = false;

  CalibrationSample() = default;
  CalibrationSample(RgbColor color, bool isValid, bool isUser)
      : rgb(color), valid(isValid), user(isUser) {}
};

enum MosaicColor : uint8_t {
  COLOR_YELLOW = 0,
  COLOR_GREEN = 1,
  COLOR_BLUE = 2,
  COLOR_WHITE = 3,
  COLOR_UNKNOWN = 255,
};

struct PointResult {
  MosaicColor color = COLOR_UNKNOWN;
  uint8_t confidence = 0;
  RgbColor rgb;
  uint16_t x = 0;
  uint16_t y = 0;
};

struct MosaicResult {
  uint32_t sequence = 0;
  int width = 0;
  int height = 0;
  PointResult points[POINT_COUNT];
};

const ResolutionOption RESOLUTIONS[] = {
    {"qqvga", "QQVGA 160x120", FRAMESIZE_QQVGA, 160, 120},
    {"qvga", "QVGA 320x240", FRAMESIZE_QVGA, 320, 240},
};

const char *COLOR_KEYS[] = {"yellow", "green", "blue", "white"};
const char *COLOR_LABELS[] = {"Y", "G", "B", "W"};

WebServer server(80);
Preferences preferences;

PointCoord points[POINT_COUNT];
CalibrationSample calibration[COLOR_COUNT];
MosaicResult lastResult;
bool hasLastResult = false;
bool cameraReady = false;
bool captureInProgress = false;
bool settingsStorageReady = false;
esp_err_t lastCameraError = ESP_OK;
int currentResolutionIndex = DEFAULT_RESOLUTION_INDEX;
int activeResolutionIndex = -1;
int sampleRadius = DEFAULT_SAMPLE_RADIUS;
int warmupFrames = DEFAULT_WARMUP_FRAMES;
uint32_t captureCount = 0;
uint32_t staleFrameDiscardCount = 0;
uint32_t warmupFrameDiscardCount = 0;
uint32_t settingsSaveCount = 0;

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32-CAM Mosaic Reader</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, -apple-system, Segoe UI, sans-serif; }
    * { box-sizing: border-box; }
    body { margin: 0; background: #0f172a; color: #f8fafc; }
    button, input, select { font: inherit; }
    .app { min-height: 100vh; display: grid; grid-template-rows: auto 1fr; gap: 12px; padding: 14px; }
    header { display: flex; justify-content: space-between; gap: 12px; align-items: center; }
    h1 { margin: 0; font-size: 24px; line-height: 1.1; }
    .top { display: flex; gap: 8px; align-items: center; flex-wrap: wrap; }
    .layout { min-height: 0; display: grid; grid-template-columns: minmax(420px, 760px) 420px; justify-content: center; align-items: start; gap: 14px; }
    .viewer { min-width: 0; display: grid; grid-template-rows: auto auto auto; gap: 10px; }
    .canvas-shell { border: 1px solid #334155; border-radius: 8px; background: #020617; padding: 10px; overflow: hidden; }
    .stage { position: relative; width: 100%; aspect-ratio: 4 / 3; background: #020617; }
    canvas, .overlay { position: absolute; inset: 0; display: block; width: 100%; height: 100%; }
    canvas { image-rendering: auto; }
    .overlay { touch-action: none; }
    .grid-line { stroke: rgba(248, 113, 113, .92); stroke-width: 3; vector-effect: non-scaling-stroke; stroke-linecap: round; }
    .handle { cursor: grab; }
    .handle-dot { stroke: #f8fafc; stroke-width: 3; vector-effect: non-scaling-stroke; }
    .handle.selected .handle-dot { stroke: #38bdf8; stroke-width: 5; }
    .handle-label { fill: #020617; font-size: 360px; font-weight: 800; text-anchor: middle; dominant-baseline: central; pointer-events: none; user-select: none; }
    .status { border: 1px solid #1d4ed8; background: #172554; color: #dbeafe; border-radius: 6px; padding: 9px 10px; font-size: 13px; line-height: 1.35; min-height: 38px; }
    .error { border-color: #991b1b; background: #450a0a; color: #fecaca; }
    .panel { min-height: 0; overflow: auto; border: 1px solid #334155; border-radius: 8px; background: #111827; padding: 12px; }
    .group { display: grid; gap: 10px; margin-bottom: 14px; }
    .row { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 8px; }
    label { display: grid; gap: 5px; color: #cbd5e1; font-size: 12px; }
    .label { display: flex; justify-content: space-between; gap: 6px; align-items: baseline; }
    .label strong { color: #f8fafc; font-size: 13px; }
    select, input { min-width: 0; width: 100%; height: 38px; border-radius: 6px; border: 1px solid #475569; background: #1f2937; color: #f8fafc; padding: 0 10px; }
    button { height: 40px; border-radius: 6px; border: 1px solid #2563eb; padding: 0 13px; background: #2563eb; color: white; cursor: pointer; }
    button.secondary { background: #0f766e; border-color: #0f766e; }
    button.danger { background: #991b1b; border-color: #b91c1c; }
    button:disabled { opacity: .55; cursor: default; }
    .table { display: grid; grid-template-columns: repeat(4, minmax(0, 1fr)); gap: 7px; }
    .cell { height: auto; text-align: left; border: 1px solid #334155; border-radius: 6px; background: #0f172a; padding: 8px; min-height: 78px; cursor: pointer; }
    .cell.selected { outline: 2px solid #38bdf8; }
    .symbol { display: flex; align-items: center; justify-content: space-between; gap: 6px; font-size: 22px; font-weight: 800; }
    .swatch { width: 22px; height: 22px; border-radius: 50%; border: 1px solid #94a3b8; background: #64748b; }
    .meta { margin-top: 5px; color: #94a3b8; font-size: 11px; line-height: 1.35; word-break: break-word; }
    .hint { color: #94a3b8; font-size: 12px; line-height: 1.35; }
    @media (max-width: 1020px) {
      .layout { grid-template-columns: 1fr; }
      .panel { overflow: visible; }
    }
    @media (max-width: 620px) {
      .app { padding: 10px; }
      header { align-items: flex-start; flex-direction: column; }
      .top, .row { width: 100%; display: grid; grid-template-columns: 1fr; }
      button { width: 100%; }
    }
  </style>
</head>
<body>
  <main class="app">
    <header>
      <h1>ESP32-CAM Mosaic Reader</h1>
      <div class="top">
        <button id="analyze" type="button">Analyze frame</button>
        <button id="live" class="secondary" type="button">Live setup 1 fps</button>
        <button id="reset" class="danger" type="button">Reset settings</button>
      </div>
    </header>

    <div class="layout">
      <section class="viewer">
        <div class="canvas-shell">
          <div class="stage">
            <canvas id="frame" width="160" height="120"></canvas>
            <svg id="overlay" class="overlay" viewBox="0 0 10000 7500" preserveAspectRatio="none" aria-hidden="true"></svg>
          </div>
        </div>
        <div id="status" class="status">Loading status...</div>
        <div id="error" class="status error" hidden></div>
      </section>

      <aside class="panel">
        <div class="group">
          <div class="row">
            <label>
              <span class="label"><strong>Resolution</strong><span>raw RGB565</span></span>
              <select id="resolution">
                <option value="qqvga">QQVGA 160x120</option>
                <option value="qvga">QVGA 320x240 (may fail without PSRAM)</option>
              </select>
            </label>
            <label>
              <span class="label"><strong>Sample radius</strong><span>pixels</span></span>
              <input id="radius" type="number" min="0" max="10" value="3">
            </label>
            <label>
              <span class="label"><strong>Warm-up frames</strong><span>AWB/AEC settle</span></span>
              <input id="warmup" type="number" min="0" max="8" value="4">
            </label>
          </div>
          <div class="row">
            <label>
              <span class="label"><strong>Selected point</strong><span>1..12</span></span>
              <select id="point"></select>
            </label>
            <label>
              <span class="label"><strong>Calibration color</strong><span>sample target</span></span>
              <select id="color">
                <option value="yellow">Yellow</option>
                <option value="green">Green</option>
                <option value="blue">Blue</option>
                <option value="white">White</option>
              </select>
            </label>
          </div>
          <button id="calibrate" type="button">Calibrate selected color</button>
          <div class="hint">Drag corner points 1, 4, 9, and 12 onto the mosaic corners. Inner recognition points are computed on straight grid lines. ESP32 does the color recognition; this page only displays the raw frame and saves calibration.</div>
        </div>

        <div class="group">
          <strong>Result 3 x 4</strong>
          <div id="resultTable" class="table"></div>
        </div>

        <div id="calibration" class="status">Calibration loading...</div>
      </aside>
    </div>
  </main>

  <script>
    const canvas = document.getElementById('frame');
    const ctx = canvas.getContext('2d');
    const overlay = document.getElementById('overlay');
    const statusBox = document.getElementById('status');
    const errorBox = document.getElementById('error');
    const resultTable = document.getElementById('resultTable');
    const calibrationBox = document.getElementById('calibration');
    const resolution = document.getElementById('resolution');
    const radius = document.getElementById('radius');
    const warmup = document.getElementById('warmup');
    const pointSelect = document.getElementById('point');
    const colorSelect = document.getElementById('color');
    const analyzeButton = document.getElementById('analyze');
    const liveButton = document.getElementById('live');
    const resetButton = document.getElementById('reset');
    const calibrateButton = document.getElementById('calibrate');

    const colorStyles = {
      yellow: '#facc15',
      green: '#22c55e',
      blue: '#3b82f6',
      white: '#f8fafc',
      unknown: '#64748b'
    };
    const NORM_MAX = 10000;
    const OVERLAY_WIDTH = 10000;
    const OVERLAY_HEIGHT = 7500;
    const CORNER_INDICES = [0, 3, 8, 11];
    const cornerSet = new Set(CORNER_INDICES);
    const labels = { yellow: 'Y', green: 'G', blue: 'B', white: 'W', unknown: '?' };
    let points = [];
    let lastResult = null;
    let calibration = [];
    let selectedPoint = 0;
    let dragPoint = -1;
    let busy = false;
    let live = false;
    let liveTimer = 0;
    let lastImageData = null;
    let pointsDirty = false;

    for (let i = 0; i < 12; i++) {
      const option = document.createElement('option');
      option.value = String(i);
      option.textContent = `${i + 1} / r${Math.floor(i / 4) + 1}c${i % 4 + 1}`;
      pointSelect.appendChild(option);
    }

    function showError(message) {
      errorBox.hidden = !message;
      errorBox.textContent = message || '';
    }

    function svgElement(name, attributes) {
      const element = document.createElementNS('http://www.w3.org/2000/svg', name);
      Object.entries(attributes).forEach(([key, value]) => element.setAttribute(key, String(value)));
      return element;
    }

    function overlayPoint(point) {
      return {
        x: point.x * OVERLAY_WIDTH / NORM_MAX,
        y: point.y * OVERLAY_HEIGHT / NORM_MAX
      };
    }

    function lerp(a, b, t) {
      return a + (b - a) * t;
    }

    function gridPoint(row, col) {
      const tl = points[0];
      const tr = points[3];
      const bl = points[8];
      const br = points[11];
      const s = col / 3;
      const t = row / 2;
      const top = { x: lerp(tl.x, tr.x, s), y: lerp(tl.y, tr.y, s) };
      const bottom = { x: lerp(bl.x, br.x, s), y: lerp(bl.y, br.y, s) };
      return {
        x: Math.round(lerp(top.x, bottom.x, t)),
        y: Math.round(lerp(top.y, bottom.y, t))
      };
    }

    function constrainGridPoints() {
      if (points.length !== 12) return false;
      let changed = false;
      for (let row = 0; row < 3; row++) {
        for (let col = 0; col < 4; col++) {
          const index = row * 4 + col;
          if (cornerSet.has(index)) continue;
          const next = gridPoint(row, col);
          if (points[index].x !== next.x || points[index].y !== next.y) {
            points[index] = next;
            changed = true;
          }
        }
      }
      return changed;
    }

    function drawLine(aIndex, bIndex, className = 'grid-line') {
      const a = overlayPoint(points[aIndex]);
      const b = overlayPoint(points[bIndex]);
      overlay.appendChild(svgElement('line', {
        class: className,
        x1: a.x, y1: a.y, x2: b.x, y2: b.y
      }));
    }

    function drawOverlay() {
      overlay.replaceChildren();
      if (!points.length) return;
      drawLine(0, 3);
      drawLine(4, 7);
      drawLine(8, 11);
      drawLine(0, 8);
      drawLine(1, 9);
      drawLine(2, 10);
      drawLine(3, 11);
      points.forEach((point, index) => {
        const p = overlayPoint(point);
        const result = lastResult && lastResult.points ? lastResult.points[index] : null;
        const color = result ? result.color : 'unknown';
        const group = svgElement('g', { class: 'handle' + (index === selectedPoint ? ' selected' : '') + (cornerSet.has(index) ? ' corner' : '') });
        group.dataset.index = String(index);
        group.appendChild(svgElement('circle', {
          class: 'handle-dot',
          cx: p.x,
          cy: p.y,
          r: index === selectedPoint ? 420 : 320,
          fill: colorStyles[color] || colorStyles.unknown
        }));
        const text = svgElement('text', {
          class: 'handle-label',
          x: p.x,
          y: p.y + 12
        });
        text.textContent = String(index + 1);
        group.appendChild(text);
        overlay.appendChild(group);
      });
    }

    function renderRgb565(buffer, width, height) {
      canvas.width = width;
      canvas.height = height;
      const src = new Uint8Array(buffer);
      const image = ctx.createImageData(width, height);
      for (let i = 0, j = 0; i + 1 < src.length && j < image.data.length; i += 2, j += 4) {
        const raw = (src[i] << 8) | src[i + 1];
        image.data[j] = ((raw >> 11) & 0x1f) * 255 / 31;
        image.data[j + 1] = ((raw >> 5) & 0x3f) * 255 / 63;
        image.data[j + 2] = (raw & 0x1f) * 255 / 31;
        image.data[j + 3] = 255;
      }
      lastImageData = image;
      ctx.putImageData(lastImageData, 0, 0);
      drawOverlay();
    }

    function renderTable() {
      resultTable.innerHTML = '';
      for (let i = 0; i < 12; i++) {
        const item = lastResult && lastResult.points ? lastResult.points[i] : null;
        const color = item ? item.color : 'unknown';
        const rgb = item ? `${item.rgb.r},${item.rgb.g},${item.rgb.b}` : '-';
        const confidence = item ? `${item.confidence}%` : '-';
        const cell = document.createElement('button');
        cell.type = 'button';
        cell.className = 'cell' + (i === selectedPoint ? ' selected' : '');
        cell.innerHTML = `<div class="symbol"><span>${labels[color] || '?'}</span><span class="swatch" style="background:${colorStyles[color] || colorStyles.unknown}"></span></div><div class="meta">#${i + 1} r${Math.floor(i / 4) + 1}c${i % 4 + 1}<br>conf ${confidence}<br>rgb ${rgb}</div>`;
        cell.addEventListener('click', () => setSelectedPoint(i));
        resultTable.appendChild(cell);
      }
    }

    function renderCalibration() {
      if (!calibration.length) {
        calibrationBox.textContent = 'Calibration unavailable';
        return;
      }
      calibrationBox.textContent = calibration.map(sample => {
        const mark = sample.user ? 'user' : 'default';
        return `${sample.color}: ${sample.r},${sample.g},${sample.b} (${mark})`;
      }).join(' | ');
    }

    function setSelectedPoint(index) {
      selectedPoint = Math.max(0, Math.min(11, index));
      pointSelect.value = String(selectedPoint);
      renderTable();
      redrawCurrentOverlay();
    }

    function redrawCurrentOverlay() {
      analyzeButton.disabled = busy;
      drawOverlay();
    }

    async function loadStatus() {
      const response = await fetch('/status', { cache: 'no-store' });
      const data = await response.json();
      points = data.points || points;
      pointsDirty = constrainGridPoints() || pointsDirty;
      calibration = data.calibration || calibration;
      resolution.value = data.resolution || 'qqvga';
      radius.value = data.sample_radius || 3;
      warmup.value = data.warmup_frames ?? 4;
      lastResult = data.last_result || lastResult;
      statusBox.textContent = `IP ${data.ip} | camera ${data.camera} | PSRAM ${data.psram} | res ${data.resolution} | captures ${data.captures} | warm-up ${data.warmup_frames}`;
      renderTable();
      renderCalibration();
      redrawCurrentOverlay();
    }

    async function analyzeFrame() {
      if (busy) return;
      busy = true;
      analyzeButton.disabled = true;
      showError('');
      try {
        if (pointsDirty) await savePoints(false);
        const params = new URLSearchParams({ res: resolution.value, radius: radius.value, warmup: warmup.value, t: String(Date.now()) });
        const response = await fetch(`/frame?${params}`, { cache: 'no-store' });
        if (!response.ok) throw new Error(await response.text());
        const width = Number(response.headers.get('X-Frame-Width') || 160);
        const height = Number(response.headers.get('X-Frame-Height') || 120);
        const resultHeader = response.headers.get('X-Mosaic-Result');
        const buffer = await response.arrayBuffer();
        if (resultHeader) lastResult = JSON.parse(resultHeader);
        renderRgb565(buffer, width, height);
        await loadStatus();
      } catch (error) {
        showError(error.message || String(error));
      } finally {
        busy = false;
        analyzeButton.disabled = false;
        if (live) liveTimer = window.setTimeout(analyzeFrame, 1000);
      }
    }

    async function savePoints(reload = true) {
      constrainGridPoints();
      const params = new URLSearchParams({ res: resolution.value, radius: radius.value, warmup: warmup.value });
      points.forEach((point, index) => {
        params.set(`x${index}`, String(Math.round(point.x)));
        params.set(`y${index}`, String(Math.round(point.y)));
      });
      const response = await fetch(`/points?${params}`, { method: 'POST', cache: 'no-store' });
      if (!response.ok) throw new Error(await response.text());
      pointsDirty = false;
      if (reload === true) await loadStatus();
    }

    async function calibrateSelected() {
      showError('');
      try {
        const params = new URLSearchParams({
          point: String(selectedPoint),
          color: colorSelect.value,
          res: resolution.value,
          radius: radius.value,
          warmup: warmup.value
        });
        const response = await fetch(`/calibrate?${params}`, { method: 'POST', cache: 'no-store' });
        if (!response.ok) throw new Error(await response.text());
        await response.json();
        await analyzeFrame();
      } catch (error) {
        showError(error.message || String(error));
      }
    }

    function eventToNorm(event) {
      const rect = overlay.getBoundingClientRect();
      return {
        x: Math.max(0, Math.min(NORM_MAX, Math.round((event.clientX - rect.left) * NORM_MAX / rect.width))),
        y: Math.max(0, Math.min(NORM_MAX, Math.round((event.clientY - rect.top) * NORM_MAX / rect.height)))
      };
    }

    function eventToOverlay(event) {
      const rect = overlay.getBoundingClientRect();
      return {
        x: Math.max(0, Math.min(OVERLAY_WIDTH, (event.clientX - rect.left) * OVERLAY_WIDTH / rect.width)),
        y: Math.max(0, Math.min(OVERLAY_HEIGHT, (event.clientY - rect.top) * OVERLAY_HEIGHT / rect.height))
      };
    }

    overlay.addEventListener('pointerdown', event => {
      if (!points.length) return;
      const p = eventToOverlay(event);
      let best = -1;
      let bestDist = Infinity;
      points.forEach((point, index) => {
        const q = overlayPoint(point);
        const dx = p.x - q.x;
        const dy = p.y - q.y;
        const dist = dx * dx + dy * dy;
        if (dist < bestDist) { bestDist = dist; best = index; }
      });
      if (best >= 0 && bestDist < 900 * 900) {
        setSelectedPoint(best);
        if (cornerSet.has(best)) {
          dragPoint = best;
          overlay.setPointerCapture(event.pointerId);
        }
      }
    });

    overlay.addEventListener('pointermove', event => {
      if (dragPoint < 0) return;
      points[dragPoint] = eventToNorm(event);
      pointsDirty = constrainGridPoints() || pointsDirty;
      redrawCurrentOverlay();
    });

    overlay.addEventListener('pointerup', async event => {
      if (dragPoint < 0) return;
      dragPoint = -1;
      overlay.releasePointerCapture(event.pointerId);
      try { await savePoints(); } catch (error) { showError(error.message || String(error)); }
    });

    pointSelect.addEventListener('change', () => setSelectedPoint(Number(pointSelect.value)));
    resolution.addEventListener('change', analyzeFrame);
    radius.addEventListener('change', () => savePoints());
    warmup.addEventListener('change', () => savePoints());
    analyzeButton.addEventListener('click', analyzeFrame);
    calibrateButton.addEventListener('click', calibrateSelected);
    liveButton.addEventListener('click', () => {
      live = !live;
      liveButton.textContent = live ? 'Pause live setup' : 'Live setup 1 fps';
      if (live) analyzeFrame(); else window.clearTimeout(liveTimer);
    });
    resetButton.addEventListener('click', async () => {
      if (!confirm('Reset saved points and calibration?')) return;
      const response = await fetch('/settings/reset', { method: 'POST', cache: 'no-store' });
      if (!response.ok) { showError(await response.text()); return; }
      lastResult = null;
      await loadStatus();
      await analyzeFrame();
    });

    loadStatus().then(analyzeFrame).catch(error => showError(error.message || String(error)));
  </script>
</body>
</html>
)HTML";

int clampValue(int value, int minValue, int maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

const ResolutionOption &configuredResolution() {
  return RESOLUTIONS[currentResolutionIndex];
}

void applyDefaultPoints() {
  const uint16_t defaultX[GRID_COLS] = {2500, 4167, 5833, 7500};
  const uint16_t defaultY[GRID_ROWS] = {3500, 5000, 6500};
  for (int row = 0; row < GRID_ROWS; row++) {
    for (int col = 0; col < GRID_COLS; col++) {
      const int index = row * GRID_COLS + col;
      points[index].x = defaultX[col];
      points[index].y = defaultY[row];
    }
  }
}

void applyDefaultCalibration() {
  calibration[COLOR_YELLOW] = CalibrationSample(RgbColor(235, 215, 20), true, false);
  calibration[COLOR_GREEN] = CalibrationSample(RgbColor(45, 155, 105), true, false);
  calibration[COLOR_BLUE] = CalibrationSample(RgbColor(35, 105, 220), true, false);
  calibration[COLOR_WHITE] = CalibrationSample(RgbColor(225, 225, 220), true, false);
}

void applyDefaultSettings() {
  currentResolutionIndex = DEFAULT_RESOLUTION_INDEX;
  activeResolutionIndex = -1;
  sampleRadius = DEFAULT_SAMPLE_RADIUS;
  warmupFrames = DEFAULT_WARMUP_FRAMES;
  applyDefaultPoints();
  applyDefaultCalibration();
}

int resolutionIndexForKey(const String &key) {
  for (size_t i = 0; i < sizeof(RESOLUTIONS) / sizeof(RESOLUTIONS[0]); i++) {
    if (key.equals(RESOLUTIONS[i].key)) {
      return static_cast<int>(i);
    }
  }
  return currentResolutionIndex;
}

int requestInt(const char *name, int fallback, int minValue, int maxValue) {
  if (!server.hasArg(name)) return fallback;
  return clampValue(server.arg(name).toInt(), minValue, maxValue);
}

int colorIndexForKey(String key) {
  key.toLowerCase();
  for (int i = 0; i < COLOR_COUNT; i++) {
    if (key == COLOR_KEYS[i]) return i;
  }
  return -1;
}

const char *colorKey(MosaicColor color) {
  if (color <= COLOR_WHITE) return COLOR_KEYS[color];
  return "unknown";
}

const char *colorLabel(MosaicColor color) {
  if (color <= COLOR_WHITE) return COLOR_LABELS[color];
  return "?";
}

void savePersistentSettings() {
  if (!settingsStorageReady) return;
  preferences.putUInt("version", SETTINGS_VERSION);
  preferences.putInt("res", currentResolutionIndex);
  preferences.putInt("radius", sampleRadius);
  preferences.putInt("warmup", warmupFrames);
  for (int i = 0; i < POINT_COUNT; i++) {
    preferences.putInt(("x" + String(i)).c_str(), points[i].x);
    preferences.putInt(("y" + String(i)).c_str(), points[i].y);
  }
  for (int i = 0; i < COLOR_COUNT; i++) {
    preferences.putInt(("cr" + String(i)).c_str(), calibration[i].rgb.r);
    preferences.putInt(("cg" + String(i)).c_str(), calibration[i].rgb.g);
    preferences.putInt(("cb" + String(i)).c_str(), calibration[i].rgb.b);
    preferences.putBool(("cv" + String(i)).c_str(), calibration[i].valid);
    preferences.putBool(("cu" + String(i)).c_str(), calibration[i].user);
  }
  settingsSaveCount++;
}

void loadPersistentSettings() {
  applyDefaultSettings();
  settingsStorageReady = preferences.begin(SETTINGS_NAMESPACE, false);
  if (!settingsStorageReady) {
    Serial.println("settings storage unavailable");
    return;
  }

  if (preferences.getUInt("version", 0) != SETTINGS_VERSION) {
    savePersistentSettings();
    Serial.println("settings initialized to defaults");
    return;
  }

  currentResolutionIndex = clampValue(preferences.getInt("res", currentResolutionIndex), 0,
                                      static_cast<int>(sizeof(RESOLUTIONS) / sizeof(RESOLUTIONS[0])) - 1);
  sampleRadius = clampValue(preferences.getInt("radius", sampleRadius), 0, 10);
  warmupFrames = clampValue(preferences.getInt("warmup", warmupFrames), 0, 8);
  for (int i = 0; i < POINT_COUNT; i++) {
    points[i].x = static_cast<uint16_t>(clampValue(preferences.getInt(("x" + String(i)).c_str(), points[i].x), 0, 10000));
    points[i].y = static_cast<uint16_t>(clampValue(preferences.getInt(("y" + String(i)).c_str(), points[i].y), 0, 10000));
  }
  for (int i = 0; i < COLOR_COUNT; i++) {
    calibration[i].rgb.r = static_cast<uint8_t>(clampValue(preferences.getInt(("cr" + String(i)).c_str(), calibration[i].rgb.r), 0, 255));
    calibration[i].rgb.g = static_cast<uint8_t>(clampValue(preferences.getInt(("cg" + String(i)).c_str(), calibration[i].rgb.g), 0, 255));
    calibration[i].rgb.b = static_cast<uint8_t>(clampValue(preferences.getInt(("cb" + String(i)).c_str(), calibration[i].rgb.b), 0, 255));
    calibration[i].valid = preferences.getBool(("cv" + String(i)).c_str(), calibration[i].valid);
    calibration[i].user = preferences.getBool(("cu" + String(i)).c_str(), calibration[i].user);
  }
  Serial.printf("settings loaded: res=%s radius=%d warmup=%d\n",
                configuredResolution().key, sampleRadius, warmupFrames);
}

void resetPersistentSettings() {
  if (settingsStorageReady) {
    preferences.clear();
  }
  applyDefaultSettings();
  savePersistentSettings();
  cameraReady = false;
  activeResolutionIndex = -1;
  hasLastResult = false;
  Serial.println("mosaic settings reset to defaults");
}

void updateSettingsFromRequest() {
  const int previousResolutionIndex = currentResolutionIndex;
  const int previousRadius = sampleRadius;
  const int previousWarmupFrames = warmupFrames;
  if (server.hasArg("res")) {
    currentResolutionIndex = resolutionIndexForKey(server.arg("res"));
  }
  sampleRadius = requestInt("radius", sampleRadius, 0, 10);
  warmupFrames = requestInt("warmup", warmupFrames, 0, 8);
  if (previousResolutionIndex != currentResolutionIndex ||
      previousRadius != sampleRadius || previousWarmupFrames != warmupFrames) {
    savePersistentSettings();
  }
}

RgbColor decodeRgb565(const uint8_t *pixel) {
  const uint16_t raw = (static_cast<uint16_t>(pixel[0]) << 8) | pixel[1];
  RgbColor rgb;
  rgb.r = static_cast<uint8_t>(((raw >> 11) & 0x1F) * 255 / 31);
  rgb.g = static_cast<uint8_t>(((raw >> 5) & 0x3F) * 255 / 63);
  rgb.b = static_cast<uint8_t>((raw & 0x1F) * 255 / 31);
  return rgb;
}

int colorLuma(const RgbColor &rgb) {
  return (77 * rgb.r + 150 * rgb.g + 29 * rgb.b) >> 8;
}

int colorChroma(const RgbColor &rgb) {
  const int maxChannel = max(static_cast<int>(rgb.r), max(static_cast<int>(rgb.g), static_cast<int>(rgb.b)));
  const int minChannel = min(static_cast<int>(rgb.r), min(static_cast<int>(rgb.g), static_cast<int>(rgb.b)));
  return maxChannel - minChannel;
}

bool isBlackFramePixel(const RgbColor &rgb) {
  const int maxChannel = max(static_cast<int>(rgb.r), max(static_cast<int>(rgb.g), static_cast<int>(rgb.b)));
  const int luma = colorLuma(rgb);
  const int chroma = colorChroma(rgb);
  return maxChannel < 45 || (luma < 35 && maxChannel < 70 && chroma < 30);
}

NormalizedColor normalizeColor(const RgbColor &rgb) {
  const int sum = static_cast<int>(rgb.r) + rgb.g + rgb.b;
  if (sum <= 0) {
    return {};
  }

  NormalizedColor normalized;
  normalized.r = static_cast<float>(rgb.r) / static_cast<float>(sum);
  normalized.g = static_cast<float>(rgb.g) / static_cast<float>(sum);
  normalized.b = static_cast<float>(rgb.b) / static_cast<float>(sum);
  normalized.valid = true;
  return normalized;
}

RgbColor samplePoint(const camera_fb_t *fb, const PointCoord &point, int radius) {
  const int width = fb->width > 0 ? fb->width : configuredResolution().width;
  const int height = fb->height > 0 ? fb->height : configuredResolution().height;
  const int centerX = (static_cast<int>(point.x) * (width - 1) + 5000) / 10000;
  const int centerY = (static_cast<int>(point.y) * (height - 1) + 5000) / 10000;
  uint32_t sumR = 0;
  uint32_t sumG = 0;
  uint32_t sumB = 0;
  uint32_t count = 0;
  uint32_t fallbackSumR = 0;
  uint32_t fallbackSumG = 0;
  uint32_t fallbackSumB = 0;
  uint32_t fallbackCount = 0;

  for (int y = centerY - radius; y <= centerY + radius; y++) {
    if (y < 0 || y >= height) continue;
    for (int x = centerX - radius; x <= centerX + radius; x++) {
      if (x < 0 || x >= width) continue;
      const size_t offset = (static_cast<size_t>(y) * width + x) * 2;
      if (offset + 1 >= fb->len) continue;
      const RgbColor rgb = decodeRgb565(fb->buf + offset);
      fallbackSumR += rgb.r;
      fallbackSumG += rgb.g;
      fallbackSumB += rgb.b;
      fallbackCount++;
      if (isBlackFramePixel(rgb)) continue;
      sumR += rgb.r;
      sumG += rgb.g;
      sumB += rgb.b;
      count++;
    }
  }

  if (count == 0) {
    if (fallbackCount == 0) return {};
    return RgbColor(static_cast<uint8_t>(fallbackSumR / fallbackCount),
                    static_cast<uint8_t>(fallbackSumG / fallbackCount),
                    static_cast<uint8_t>(fallbackSumB / fallbackCount));
  }

  return RgbColor(static_cast<uint8_t>(sumR / count),
                  static_cast<uint8_t>(sumG / count),
                  static_cast<uint8_t>(sumB / count));
}

float colorDistance(const NormalizedColor &a, const NormalizedColor &b) {
  const float dr = a.r - b.r;
  const float dg = a.g - b.g;
  const float db = a.b - b.b;
  return sqrtf(dr * dr + dg * dg + db * db);
}

MosaicColor classifyColor(const RgbColor &rgb, uint8_t &confidence) {
  if (colorLuma(rgb) < 20) {
    confidence = 0;
    return COLOR_UNKNOWN;
  }

  const NormalizedColor target = normalizeColor(rgb);
  if (!target.valid) {
    confidence = 0;
    return COLOR_UNKNOWN;
  }

  int bestIndex = -1;
  float best = 100000.0f;
  float second = 100000.0f;

  for (int i = 0; i < COLOR_COUNT; i++) {
    if (!calibration[i].valid) continue;
    const NormalizedColor sample = normalizeColor(calibration[i].rgb);
    if (!sample.valid) continue;
    const float distance = colorDistance(target, sample);
    if (distance < best) {
      second = best;
      best = distance;
      bestIndex = i;
    } else if (distance < second) {
      second = distance;
    }
  }

  if (bestIndex < 0) {
    confidence = 0;
    return COLOR_UNKNOWN;
  }

  const int absolute = clampValue(100 - static_cast<int>(best * 160.0f), 0, 100);
  const int margin = second < 99999.0f ? clampValue(static_cast<int>((second - best) * 100.0f / second), 0, 100) : absolute;
  confidence = static_cast<uint8_t>((absolute + margin) / 2);
  return static_cast<MosaicColor>(bestIndex);
}

void recognizeFrame(const camera_fb_t *fb, MosaicResult &result) {
  result.sequence = captureCount;
  result.width = fb->width > 0 ? fb->width : configuredResolution().width;
  result.height = fb->height > 0 ? fb->height : configuredResolution().height;
  for (int i = 0; i < POINT_COUNT; i++) {
    result.points[i].x = points[i].x;
    result.points[i].y = points[i].y;
    result.points[i].rgb = samplePoint(fb, points[i], sampleRadius);
    result.points[i].color = classifyColor(result.points[i].rgb, result.points[i].confidence);
  }
}

String resultPattern(const MosaicResult &result) {
  String pattern;
  for (int i = 0; i < POINT_COUNT; i++) {
    if (i > 0 && i % GRID_COLS == 0) pattern += "/";
    pattern += colorLabel(result.points[i].color);
  }
  return pattern;
}

void appendRgbJson(String &json, const RgbColor &rgb) {
  json += "\"r\":" + String(rgb.r) + ",";
  json += "\"g\":" + String(rgb.g) + ",";
  json += "\"b\":" + String(rgb.b);
}

String resultToJson(const MosaicResult &result) {
  String json = "{";
  json += "\"sequence\":" + String(result.sequence) + ",";
  json += "\"width\":" + String(result.width) + ",";
  json += "\"height\":" + String(result.height) + ",";
  json += "\"pattern\":\"" + resultPattern(result) + "\",";
  json += "\"points\":[";
  for (int i = 0; i < POINT_COUNT; i++) {
    if (i > 0) json += ",";
    const PointResult &point = result.points[i];
    json += "{";
    json += "\"index\":" + String(i) + ",";
    json += "\"row\":" + String(i / GRID_COLS) + ",";
    json += "\"col\":" + String(i % GRID_COLS) + ",";
    json += "\"color\":\"" + String(colorKey(point.color)) + "\",";
    json += "\"label\":\"" + String(colorLabel(point.color)) + "\",";
    json += "\"confidence\":" + String(point.confidence) + ",";
    json += "\"x\":" + String(point.x) + ",";
    json += "\"y\":" + String(point.y) + ",";
    json += "\"rgb\":{";
    appendRgbJson(json, point.rgb);
    json += "}";
    json += "}";
  }
  json += "]";
  json += "}";
  return json;
}

void appendPointsJson(String &json) {
  json += "\"points\":[";
  for (int i = 0; i < POINT_COUNT; i++) {
    if (i > 0) json += ",";
    json += "{\"x\":" + String(points[i].x) + ",\"y\":" + String(points[i].y) + "}";
  }
  json += "]";
}

void appendCalibrationJson(String &json) {
  json += "\"calibration\":[";
  for (int i = 0; i < COLOR_COUNT; i++) {
    if (i > 0) json += ",";
    json += "{";
    json += "\"color\":\"" + String(COLOR_KEYS[i]) + "\",";
    appendRgbJson(json, calibration[i].rgb);
    json += ",\"valid\":" + String(calibration[i].valid ? "true" : "false");
    json += ",\"user\":" + String(calibration[i].user ? "true" : "false");
    json += "}";
  }
  json += "]";
}

String statusJson() {
  String json = "{";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"camera\":\"" + String(cameraReady ? "ready" : "not_ready") + "\",";
  json += "\"psram\":\"" + String(psramFound() ? "yes" : "no") + "\",";
  json += "\"free_heap\":" + String(ESP.getFreeHeap()) + ",";
  json += "\"resolution\":\"" + String(configuredResolution().key) + "\",";
  json += "\"width\":" + String(configuredResolution().width) + ",";
  json += "\"height\":" + String(configuredResolution().height) + ",";
  json += "\"sample_radius\":" + String(sampleRadius) + ",";
  json += "\"warmup_frames\":" + String(warmupFrames) + ",";
  json += "\"captures\":" + String(captureCount) + ",";
  json += "\"stale_discards\":" + String(staleFrameDiscardCount) + ",";
  json += "\"warmup_discards\":" + String(warmupFrameDiscardCount) + ",";
  json += "\"settings_storage\":\"" + String(settingsStorageReady ? "ready" : "unavailable") + "\",";
  json += "\"settings_saves\":" + String(settingsSaveCount) + ",";
  json += "\"last_error\":\"" + String(esp_err_to_name(lastCameraError)) + "\",";
  appendPointsJson(json);
  json += ",";
  appendCalibrationJson(json);
  if (hasLastResult) {
    json += ",\"last_result\":";
    json += resultToJson(lastResult);
  } else {
    json += ",\"last_result\":null";
  }
  json += "}";
  return json;
}

camera_config_t makeCameraConfig(const ResolutionOption &resolution) {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size = resolution.frameSize;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_DRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;
  return config;
}

void applySensorDefaults() {
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr) return;
  if (sensor->set_whitebal) sensor->set_whitebal(sensor, 1);
  if (sensor->set_awb_gain) sensor->set_awb_gain(sensor, 1);
  if (sensor->set_exposure_ctrl) sensor->set_exposure_ctrl(sensor, 1);
  if (sensor->set_gain_ctrl) sensor->set_gain_ctrl(sensor, 1);
  if (sensor->set_dcw) sensor->set_dcw(sensor, 1);
}

bool ensureCamera(const ResolutionOption &resolution) {
  const int desiredIndex = resolutionIndexForKey(resolution.key);
  if (cameraReady && activeResolutionIndex == desiredIndex) {
    return true;
  }

  cameraReady = false;
  Serial.printf("camera init: %s RGB565 DRAM\n", resolution.key);
  esp_camera_deinit();
  delay(50);

  pinMode(PWDN_GPIO_NUM, OUTPUT);
  digitalWrite(PWDN_GPIO_NUM, LOW);
  delay(50);

  camera_config_t config = makeCameraConfig(resolution);
  lastCameraError = esp_camera_init(&config);
  if (lastCameraError != ESP_OK) {
    Serial.printf("camera init failed: 0x%08X (%s)\n", lastCameraError,
                  esp_err_to_name(lastCameraError));
    activeResolutionIndex = -1;
    return false;
  }

  activeResolutionIndex = desiredIndex;
  cameraReady = true;
  applySensorDefaults();
  Serial.printf("camera ready: resolution=%s size=%dx%d format=RGB565 fb=DRAM\n",
                resolution.key, resolution.width, resolution.height);
  return true;
}

bool ensureConfiguredCamera() {
  if (ensureCamera(configuredResolution())) {
    return true;
  }

  if (currentResolutionIndex == SAFE_RESOLUTION_INDEX) {
    return false;
  }

  const char *failedResolution = configuredResolution().key;
  Serial.printf("camera fallback: %s failed with %s, retrying %s\n",
                failedResolution, esp_err_to_name(lastCameraError),
                RESOLUTIONS[SAFE_RESOLUTION_INDEX].key);
  currentResolutionIndex = SAFE_RESOLUTION_INDEX;
  cameraReady = false;
  activeResolutionIndex = -1;
  savePersistentSettings();
  return ensureCamera(configuredResolution());
}

struct CaptureLock {
  explicit CaptureLock(bool &locked) : lockedRef(locked) {
    lockedRef = true;
  }
  ~CaptureLock() {
    lockedRef = false;
  }
  bool &lockedRef;
};

void sendPlainError(int status, const String &message) {
  Serial.printf("http error %d: %s\n", status, message.c_str());
  server.send(status, "text/plain", message);
}

bool discardFrames(String &error, int count, const char *reason, uint32_t &counter) {
  for (int i = 0; i < count; i++) {
    camera_fb_t *stale = esp_camera_fb_get();
    if (stale == nullptr) {
      error = "Frame failed: ";
      error += reason;
      error += " frame discard returned null.";
      return false;
    }

    counter++;
    Serial.printf("discard %s frame: index=%d/%d bytes=%u total=%lu\n",
                  reason, i + 1, count, static_cast<unsigned>(stale->len),
                  static_cast<unsigned long>(counter));
    esp_camera_fb_return(stale);
    delay(10);
  }
  return true;
}

camera_fb_t *captureFrame(String &error) {
  if (!ensureConfiguredCamera()) {
    error = "Camera init failed: ";
    error += esp_err_to_name(lastCameraError);
    return nullptr;
  }

  if (!discardFrames(error, warmupFrames, "warm-up", warmupFrameDiscardCount)) {
    return nullptr;
  }

  if (!discardFrames(error, STALE_FRAME_DISCARDS, "stale", staleFrameDiscardCount)) {
    return nullptr;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (fb == nullptr) {
    error = "Frame failed: esp_camera_fb_get returned null.";
    return nullptr;
  }

  if (fb->format != PIXFORMAT_RGB565) {
    esp_camera_fb_return(fb);
    error = "Frame failed: camera did not return RGB565.";
    return nullptr;
  }

  captureCount++;
  return fb;
}

void printResultToSerial(const MosaicResult &result) {
  Serial.printf("mosaic result: seq=%lu res=%dx%d pattern=%s\n",
                static_cast<unsigned long>(result.sequence), result.width,
                result.height, resultPattern(result).c_str());
  for (int i = 0; i < POINT_COUNT; i++) {
    const PointResult &point = result.points[i];
    Serial.printf("  p%02d r%d c%d %s conf=%u rgb=%u,%u,%u norm=%u,%u\n",
                  i + 1, i / GRID_COLS + 1, i % GRID_COLS + 1,
                  colorKey(point.color), point.confidence, point.rgb.r,
                  point.rgb.g, point.rgb.b, point.x, point.y);
  }
}

void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  server.send(200, "application/json", statusJson());
}

void handleSettingsReset() {
  resetPersistentSettings();
  server.send(200, "application/json", statusJson());
}

void handleFrame() {
  if (captureInProgress) {
    sendPlainError(503, "Capture already in progress.");
    return;
  }
  CaptureLock lock(captureInProgress);
  updateSettingsFromRequest();

  String error;
  camera_fb_t *fb = captureFrame(error);
  if (fb == nullptr) {
    sendPlainError(503, error);
    return;
  }

  recognizeFrame(fb, lastResult);
  hasLastResult = true;
  const String resultJson = resultToJson(lastResult);
  printResultToSerial(lastResult);

  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("X-Frame-Width", String(lastResult.width));
  server.sendHeader("X-Frame-Height", String(lastResult.height));
  server.sendHeader("X-Mosaic-Result", resultJson);
  server.setContentLength(fb->len);
  server.send(200, "application/octet-stream", "");
  WiFiClient client = server.client();
  client.write(fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void handleResult() {
  if (captureInProgress) {
    sendPlainError(503, "Capture already in progress.");
    return;
  }
  CaptureLock lock(captureInProgress);
  updateSettingsFromRequest();

  String error;
  camera_fb_t *fb = captureFrame(error);
  if (fb == nullptr) {
    sendPlainError(503, error);
    return;
  }

  recognizeFrame(fb, lastResult);
  hasLastResult = true;
  esp_camera_fb_return(fb);
  printResultToSerial(lastResult);
  server.send(200, "application/json", resultToJson(lastResult));
}

void handlePoints() {
  updateSettingsFromRequest();
  bool changed = false;
  for (int i = 0; i < POINT_COUNT; i++) {
    const String xKey = "x" + String(i);
    const String yKey = "y" + String(i);
    if (server.hasArg(xKey) && server.hasArg(yKey)) {
      const uint16_t x = static_cast<uint16_t>(requestInt(xKey.c_str(), points[i].x, 0, 10000));
      const uint16_t y = static_cast<uint16_t>(requestInt(yKey.c_str(), points[i].y, 0, 10000));
      if (points[i].x != x || points[i].y != y) {
        points[i].x = x;
        points[i].y = y;
        changed = true;
      }
    }
  }
  if (changed) {
    savePersistentSettings();
    Serial.println("mosaic points saved");
  }
  server.send(200, "application/json", statusJson());
}

void handleCalibrate() {
  if (captureInProgress) {
    sendPlainError(503, "Capture already in progress.");
    return;
  }
  updateSettingsFromRequest();
  const int pointIndex = requestInt("point", -1, -1, POINT_COUNT - 1);
  const int colorIndex = colorIndexForKey(server.arg("color"));
  if (pointIndex < 0) {
    sendPlainError(400, "Missing or invalid point index.");
    return;
  }
  if (colorIndex < 0) {
    sendPlainError(400, "Missing or invalid color. Use yellow, green, blue, or white.");
    return;
  }

  CaptureLock lock(captureInProgress);
  String error;
  camera_fb_t *fb = captureFrame(error);
  if (fb == nullptr) {
    sendPlainError(503, error);
    return;
  }

  calibration[colorIndex].rgb = samplePoint(fb, points[pointIndex], sampleRadius);
  calibration[colorIndex].valid = true;
  calibration[colorIndex].user = true;
  savePersistentSettings();

  recognizeFrame(fb, lastResult);
  hasLastResult = true;
  esp_camera_fb_return(fb);
  printResultToSerial(lastResult);

  Serial.printf("calibrated %s from point %d rgb=%u,%u,%u\n", COLOR_KEYS[colorIndex],
                pointIndex + 1, calibration[colorIndex].rgb.r,
                calibration[colorIndex].rgb.g, calibration[colorIndex].rgb.b);
  server.send(200, "application/json", statusJson());
}

void handleNotFound() {
  sendPlainError(404, "Not found");
}

void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID_VALUE, WIFI_PASSWORD_VALUE);

  Serial.printf("connecting to Wi-Fi SSID: %s\n", WIFI_SSID_VALUE);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.printf("Wi-Fi connected: http://%s/\n", WiFi.localIP().toString().c_str());
}

void startHttpServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/frame", HTTP_GET, handleFrame);
  server.on("/result", HTTP_GET, handleResult);
  server.on("/points", HTTP_POST, handlePoints);
  server.on("/calibrate", HTTP_POST, handleCalibrate);
  server.on("/settings/reset", HTTP_POST, handleSettingsReset);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");
}

void printBootStatus() {
  Serial.println();
  Serial.println("ESP32-CAM mosaic reader firmware");
  Serial.printf("psramFound: %s\n", psramFound() ? "yes" : "no");
  Serial.printf("free heap: %u bytes\n", static_cast<unsigned>(ESP.getFreeHeap()));
  Serial.printf("resolution: %s RGB565\n", configuredResolution().key);
  Serial.printf("sample radius: %d px\n", sampleRadius);
  Serial.printf("warm-up frames: %d\n", warmupFrames);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1200);
  loadPersistentSettings();
  printBootStatus();
  connectWifi();
  startHttpServer();
  ensureConfiguredCamera();
}

void loop() {
  server.handleClient();
  delay(2);
}
