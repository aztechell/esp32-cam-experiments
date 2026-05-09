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
constexpr char SETTINGS_NAMESPACE[] = "mosaic_v2";
constexpr uint32_t SETTINGS_VERSION = 1;

constexpr int FRAME_W = 160;
constexpr int FRAME_H = 120;
constexpr int POINT_COUNT = 12;
constexpr int ROWS = 3;
constexpr int COLS = 4;
constexpr int COLOR_COUNT = 4;
constexpr int SAMPLE_RADIUS = 4;
constexpr int STALE_FRAME_DISCARDS = 1;
constexpr int DEFAULT_WARMUP_FRAMES = 4;
constexpr int TOP_CANDIDATE_COUNT = 4;
constexpr int FRAME_PIXELS = FRAME_W * FRAME_H;
constexpr int MAX_COMPONENTS = 28;
constexpr int MAX_DEBUG_COMPONENTS = 20;
constexpr int MAX_STEP_CANDIDATES = 18;
constexpr uint32_t DETECTOR_TIME_BUDGET_MS = 8000;
constexpr uint32_t WIFI_RECONNECT_INTERVAL_MS = 5000;

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

struct PointF {
  float x = 0;
  float y = 0;

  PointF() = default;
  PointF(float xValue, float yValue) : x(xValue), y(yValue) {}
};

struct PointNorm {
  uint16_t x = 0;
  uint16_t y = 0;

  PointNorm() = default;
  PointNorm(uint16_t xValue, uint16_t yValue) : x(xValue), y(yValue) {}
};

struct Quad {
  PointF tl;
  PointF tr;
  PointF bl;
  PointF br;

  Quad() = default;
  Quad(PointF topLeft, PointF topRight, PointF bottomLeft, PointF bottomRight)
      : tl(topLeft), tr(topRight), bl(bottomLeft), br(bottomRight) {}
};

struct RgbColor {
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;

  RgbColor() = default;
  RgbColor(uint8_t red, uint8_t green, uint8_t blue) : r(red), g(green), b(blue) {}
};

struct CalibrationSample {
  RgbColor rgb;
  bool user = false;

  CalibrationSample() = default;
  CalibrationSample(RgbColor color, bool isUser) : rgb(color), user(isUser) {}
};

enum MosaicColor : uint8_t {
  COLOR_YELLOW = 0,
  COLOR_GREEN = 1,
  COLOR_BLUE = 2,
  COLOR_WHITE = 3,
};

struct CellResult {
  MosaicColor color = COLOR_GREEN;
  uint8_t confidence = 0;
  uint8_t coverage = 0;
  bool inFrame = false;
  bool blobMatched = false;
  RgbColor rgb;
  PointF center;
};

struct DebugComponent {
  uint8_t x = 0;
  uint8_t y = 0;
  uint8_t w = 0;
  uint8_t h = 0;
  uint8_t score = 0;
};

struct DetectionResult {
  bool found = false;
  bool complete = false;
  float score = 0;
  float outsideRatio = 1.0f;
  float residualPx = 99.0f;
  float gridScore = 0;
  float blobScore = 0;
  uint16_t elapsedMs = 0;
  uint8_t confidence = 0;
  uint8_t visibleCells = 0;
  uint8_t blobCount = 0;
  uint8_t matchedBlobs = 0;
  uint16_t matchMask = 0;
  bool timedOut = false;
  bool projective = false;
  bool gridSearch = false;
  Quad quad;
  float homography[8] = {};
  PointF grid[(COLS + 1) * (ROWS + 1)];
  int8_t componentForCell[POINT_COUNT] = {};
  CellResult cells[POINT_COUNT];
  uint8_t debugComponentCount = 0;
  DebugComponent debugComponents[MAX_DEBUG_COMPONENTS];
};

struct CandidateScore {
  float score = -100000.0f;
  float outsideRatio = 1.0f;
  float extraGridScore = 0.0f;
  uint8_t visibleCells = 0;
};

struct Candidate {
  float cx = 0;
  float cy = 0;
  float width = 0;
  float ratio = 0.75f;
  float angle = 0;
  CandidateScore metrics;
  Quad quad;
};

struct BlobComponent {
  uint16_t area = 0;
  uint16_t minX = FRAME_W;
  uint16_t minY = FRAME_H;
  uint16_t maxX = 0;
  uint16_t maxY = 0;
  uint32_t sumX = 0;
  uint32_t sumY = 0;
  uint32_t sumR = 0;
  uint32_t sumG = 0;
  uint32_t sumB = 0;
  uint32_t sumLuma = 0;
  uint32_t sumChroma = 0;
  float cx = 0;
  float cy = 0;
  uint8_t meanR = 0;
  uint8_t meanG = 0;
  uint8_t meanB = 0;
  uint8_t meanLuma = 0;
  uint8_t meanChroma = 0;
  uint8_t fillRatio = 0;
  uint8_t borderDark = 0;
  uint8_t borderDarkSides = 0;
  uint8_t quality = 0;
};

struct StepCandidate {
  PointF step;
  float score = -100000.0f;
};

struct LatticeFit {
  bool valid = false;
  PointF origin;
  PointF xStep;
  PointF yStep;
  Quad quad;
  float score = -100000.0f;
  float residualPx = 99.0f;
  float gridScore = 0;
  float blobScore = 0;
  float outsideRatio = 1.0f;
  uint8_t matchedBlobs = 0;
  uint16_t matchMask = 0;
  uint32_t componentMask = 0;
  int8_t componentForCell[POINT_COUNT] = {};
};

struct GridSearchFit {
  bool valid = false;
  Quad quad;
  float score = -100000.0f;
  float gridScore = 0;
  float outsideRatio = 1.0f;
  uint8_t visibleCells = 0;
};

WebServer server(80);
Preferences preferences;

CalibrationSample calibration[COLOR_COUNT];
DetectionResult lastResult;
bool hasLastResult = false;
bool cameraReady = false;
bool captureInProgress = false;
bool settingsStorageReady = false;
esp_err_t lastCameraError = ESP_OK;
uint32_t captureCount = 0;
uint32_t staleFrameDiscardCount = 0;
uint32_t warmupFrameDiscardCount = 0;
uint32_t settingsSaveCount = 0;
int warmupFrames = DEFAULT_WARMUP_FRAMES;
uint8_t luma[FRAME_PIXELS];
uint8_t chroma[FRAME_PIXELS];
uint8_t cellMask[FRAME_PIXELS];
uint16_t *floodQueue = nullptr;
BlobComponent components[MAX_COMPONENTS];
StepCandidate xStepCandidates[MAX_STEP_CANDIDATES];
StepCandidate yStepCandidates[MAX_STEP_CANDIDATES];
uint8_t darkThreshold = 70;
uint32_t detectorDeadlineMs = 0;
uint32_t longWorkCounter = 0;
uint32_t lastWifiReconnectAttempt = 0;
bool detectorTimedOut = false;
bool wifiWasConnected = false;

const char *COLOR_KEYS[] = {"yellow", "green", "blue", "white"};
const char *COLOR_LABELS[] = {"Y", "G", "B", "W"};

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32-CAM Mosaic Reader v2</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, -apple-system, Segoe UI, sans-serif; }
    * { box-sizing: border-box; }
    body { margin: 0; background: #0f172a; color: #f8fafc; }
    button, input, select { font: inherit; }
    .app { min-height: 100vh; padding: 14px; display: grid; gap: 12px; grid-template-rows: auto 1fr; }
    header { display: flex; align-items: center; justify-content: space-between; gap: 12px; }
    h1 { margin: 0; font-size: 24px; }
    .top { display: flex; gap: 8px; flex-wrap: wrap; }
    .layout { display: grid; grid-template-columns: minmax(420px, 760px) 430px; gap: 14px; justify-content: center; align-items: start; }
    .viewer { display: grid; gap: 10px; }
    .shell { border: 1px solid #334155; border-radius: 8px; padding: 10px; background: #020617; }
    .stage { position: relative; width: 100%; aspect-ratio: 4 / 3; }
    canvas, svg { position: absolute; inset: 0; width: 100%; height: 100%; display: block; }
    svg { touch-action: none; }
    .model-line { stroke: rgba(250, 204, 21, .9); stroke-width: 2; stroke-dasharray: 4 4; vector-effect: non-scaling-stroke; fill: none; }
    .det-line { stroke: rgba(56, 189, 248, .96); stroke-width: 3; vector-effect: non-scaling-stroke; stroke-linecap: round; }
    .corner { cursor: grab; }
    .corner-dot { fill: #facc15; stroke: #0f172a; stroke-width: 2; vector-effect: non-scaling-stroke; }
    .cell-dot { stroke: #f8fafc; stroke-width: 2; vector-effect: non-scaling-stroke; }
    .blob-box { fill: rgba(251, 146, 60, .12); stroke: rgba(251, 146, 60, .86); stroke-width: 1.5; vector-effect: non-scaling-stroke; }
    .blob-quad { fill: rgba(251, 146, 60, .16); stroke: rgba(251, 146, 60, .98); stroke-width: 1.8; vector-effect: non-scaling-stroke; }
    .blob-dot { fill: #fb923c; stroke: #020617; stroke-width: 1; vector-effect: non-scaling-stroke; }
    .label { fill: #020617; font-size: 7px; font-weight: 800; text-anchor: middle; dominant-baseline: central; pointer-events: none; }
    .panel { border: 1px solid #334155; border-radius: 8px; background: #111827; padding: 12px; display: grid; gap: 12px; }
    .row { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
    label { display: grid; gap: 5px; color: #cbd5e1; font-size: 12px; }
    select, input { min-width: 0; width: 100%; height: 38px; border: 1px solid #475569; border-radius: 6px; background: #1f2937; color: #f8fafc; padding: 0 10px; }
    button { height: 40px; border: 1px solid #2563eb; border-radius: 6px; background: #2563eb; color: #fff; padding: 0 13px; cursor: pointer; }
    button.secondary { background: #0f766e; border-color: #0f766e; }
    button.danger { background: #991b1b; border-color: #b91c1c; }
    .top select { width: auto; min-width: 96px; height: 40px; }
    .status { border: 1px solid #1d4ed8; background: #172554; color: #dbeafe; border-radius: 6px; padding: 9px 10px; font-size: 13px; line-height: 1.35; }
    .error { border-color: #991b1b; background: #450a0a; color: #fecaca; }
    .table { display: grid; grid-template-columns: repeat(4, 1fr); gap: 7px; }
    .cell { min-height: 76px; border: 1px solid #334155; border-radius: 6px; background: #0f172a; padding: 8px; text-align: left; }
    .symbol { display: flex; align-items: center; justify-content: space-between; font-size: 22px; font-weight: 800; }
    .swatch { width: 22px; height: 22px; border-radius: 50%; border: 1px solid #94a3b8; }
    .meta { margin-top: 5px; color: #94a3b8; font-size: 11px; line-height: 1.35; }
    .hint { color: #94a3b8; font-size: 12px; line-height: 1.35; }
    @media (max-width: 1040px) { .layout { grid-template-columns: 1fr; } .panel { max-width: none; } }
    @media (max-width: 620px) { header { align-items: flex-start; flex-direction: column; } .top, .row { width: 100%; grid-template-columns: 1fr; } button { width: 100%; } }
  </style>
</head>
<body>
  <main class="app">
    <header>
      <h1>ESP32-CAM Mosaic Reader v2</h1>
      <div class="top">
        <button id="analyze" type="button">Analyze</button>
        <button id="previewToggle" class="secondary" type="button">Pause preview</button>
        <select id="previewFps" aria-label="Preview FPS">
          <option value="1">1 fps</option>
          <option value="2" selected>2 fps</option>
          <option value="5">5 fps</option>
          <option value="8">8 fps</option>
        </select>
        <button id="reset" class="danger" type="button">Reset</button>
      </div>
    </header>
    <div class="layout">
      <section class="viewer">
        <div class="shell">
          <div class="stage">
            <canvas id="frame" width="160" height="120"></canvas>
            <svg id="overlay" viewBox="0 0 160 120" preserveAspectRatio="none" aria-hidden="true"></svg>
          </div>
        </div>
        <div id="status" class="status">Loading...</div>
        <div id="error" class="status error" hidden></div>
      </section>
      <aside class="panel">
        <div class="hint">Preview is raw camera only for aiming. Analyze pauses preview and runs the heavy auto-detector once.</div>
        <div class="row">
          <label>Warm-up frames<input id="warmup" type="number" min="0" max="8" value="4"></label>
          <label>Calibrate cell<select id="cell"></select></label>
        </div>
        <label>Color<select id="color"><option value="yellow">Yellow</option><option value="green">Green</option><option value="blue">Blue</option><option value="white">White</option></select></label>
        <button id="calibrate" type="button">Calibrate selected cell</button>
        <div id="diagnostics" class="status">No result yet.</div>
        <div class="table" id="resultTable"></div>
      </aside>
    </div>
  </main>
  <script>
    const canvas = document.getElementById('frame');
    const ctx = canvas.getContext('2d');
    const overlay = document.getElementById('overlay');
    const statusBox = document.getElementById('status');
    const errorBox = document.getElementById('error');
    const diagnostics = document.getElementById('diagnostics');
    const resultTable = document.getElementById('resultTable');
    const cellSelect = document.getElementById('cell');
    const colorSelect = document.getElementById('color');
    const warmupInput = document.getElementById('warmup');
    const previewToggle = document.getElementById('previewToggle');
    const previewFps = document.getElementById('previewFps');
    const colors = { yellow: '#facc15', green: '#22c55e', blue: '#3b82f6', white: '#f8fafc' };
    const labels = { yellow: 'Y', green: 'G', blue: 'B', white: 'W' };
    let result = null;
    let previewRunning = true;
    let previewBusy = false;
    let previewTimer = 0;
    for (let i = 0; i < 12; i++) {
      const option = document.createElement('option');
      option.value = String(i);
      option.textContent = `${i + 1} / r${Math.floor(i / 4) + 1}c${i % 4 + 1}`;
      cellSelect.appendChild(option);
    }
    function showError(text) { errorBox.hidden = !text; errorBox.textContent = text || ''; }
    async function fetchWithTimeout(url, options = {}, timeoutMs = 12000) {
      return fetch(url, options);
    }
    function svg(name, attrs) {
      const element = document.createElementNS('http://www.w3.org/2000/svg', name);
      Object.entries(attrs).forEach(([k, v]) => element.setAttribute(k, String(v)));
      return element;
    }
    function addLine(a, b, cls) {
      overlay.appendChild(svg('line', { class: cls, x1: a.x, y1: a.y, x2: b.x, y2: b.y }));
    }
    function addPolygon(points, cls) {
      overlay.appendChild(svg('polygon', { class: cls, points: points.map(p => `${p.x},${p.y}`).join(' ') }));
    }
    function lerpPoint(a, b, t) {
      return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t };
    }
    function insetQuad(points, amount) {
      const center = points.reduce((acc, p) => ({ x: acc.x + p.x / 4, y: acc.y + p.y / 4 }), { x: 0, y: 0 });
      return points.map(p => lerpPoint(p, center, amount));
    }
    function drawOverlay() {
      overlay.replaceChildren();
      if (!result) return;
      if (!result.found && result.components_debug) {
        result.components_debug.forEach(c => {
          overlay.appendChild(svg('rect', { class: 'blob-box', x: c.x, y: c.y, width: c.w, height: c.h, rx: 1 }));
          overlay.appendChild(svg('circle', { class: 'blob-dot', cx: c.x + c.w / 2, cy: c.y + c.h / 2, r: 1.8 }));
        });
      }
      if (!result.found || result.confidence < 45) return;
      function gridPoint(col, row) {
        if (result.grid && result.grid.length === 20) return result.grid[row * 5 + col];
        const q = result.corners;
        const u = col / 4;
        const v = row / 3;
        const top = { x: q.tl.x + (q.tr.x - q.tl.x) * u, y: q.tl.y + (q.tr.y - q.tl.y) * u };
        const bot = { x: q.bl.x + (q.br.x - q.bl.x) * u, y: q.bl.y + (q.br.y - q.bl.y) * u };
        return { x: top.x + (bot.x - top.x) * v, y: top.y + (bot.y - top.y) * v };
      }
      result.points.forEach((p, i) => {
        if (!p.blob_match) return;
        const col = i % 4;
        const row = Math.floor(i / 4);
        const quad = [
          gridPoint(col, row),
          gridPoint(col + 1, row),
          gridPoint(col + 1, row + 1),
          gridPoint(col, row + 1)
        ];
        addPolygon(insetQuad(quad, 0.18), 'blob-quad');
      });
      for (let col = 0; col <= 4; col++) addLine(gridPoint(col, 0), gridPoint(col, 3), 'det-line');
      for (let row = 0; row <= 3; row++) addLine(gridPoint(0, row), gridPoint(4, row), 'det-line');
      result.points.forEach((p, i) => {
        const g = svg('g', {});
        g.appendChild(svg('circle', { class: 'cell-dot', cx: p.center.x, cy: p.center.y, r: 4, fill: colors[p.color] || '#64748b' }));
        const t = svg('text', { class: 'label', x: p.center.x, y: p.center.y + .2 });
        t.textContent = String(i + 1);
        g.appendChild(t);
        overlay.appendChild(g);
      });
    }
    function renderRgb565(buffer) {
      const src = new Uint8Array(buffer);
      const image = ctx.createImageData(160, 120);
      for (let i = 0, j = 0; i + 1 < src.length && j < image.data.length; i += 2, j += 4) {
        const raw = (src[i] << 8) | src[i + 1];
        image.data[j] = ((raw >> 11) & 31) * 255 / 31;
        image.data[j + 1] = ((raw >> 5) & 63) * 255 / 63;
        image.data[j + 2] = (raw & 31) * 255 / 31;
        image.data[j + 3] = 255;
      }
      ctx.putImageData(image, 0, 0);
    }
    function warmupValue() {
      const parsed = Number.parseInt(warmupInput.value, 10);
      const value = Number.isFinite(parsed) ? Math.max(0, Math.min(8, parsed)) : 4;
      warmupInput.value = String(value);
      return String(value);
    }
    function renderTable() {
      resultTable.innerHTML = '';
      for (let i = 0; i < 12; i++) {
        const p = result && result.points ? result.points[i] : null;
        const color = p ? p.color : 'green';
        const cell = document.createElement('div');
        cell.className = 'cell';
        cell.innerHTML = `<div class="symbol"><span>${labels[color] || '?'}</span><span class="swatch" style="background:${colors[color] || '#64748b'}"></span></div><div class="meta">#${i + 1}<br>conf ${p ? p.confidence : 0}% cov ${p ? p.coverage : 0}% ${p && p.blob_match ? 'blob' : 'no blob'}<br>rgb ${p ? `${p.rgb.r},${p.rgb.g},${p.rgb.b}` : '-'}</div>`;
        resultTable.appendChild(cell);
      }
    }
    async function loadStatus() {
      const response = await fetchWithTimeout('/status', { cache: 'no-store' }, 5000);
      const data = await response.json();
      result = data.last_result || result;
      if (typeof data.warmup_frames === 'number') warmupInput.value = String(data.warmup_frames);
      statusBox.textContent = `IP ${data.ip} | camera ${data.camera} | captures ${data.captures} | warm-up ${data.warmup_frames} | warm discards ${data.warmup_discards} | stale discards ${data.stale_discards}`;
      if (result) diagnostics.textContent = `status ${result.status} | source ${result.source || '-'} | complete ${result.complete ? 'yes' : 'no'} | timeout ${result.timed_out ? 'yes' : 'no'} | score ${result.score} | conf ${result.confidence}% | blobs ${result.matched_blobs}/${result.blob_count} | residual ${result.residual_px}px | grid ${result.grid_score} | blob ${result.blob_score} | ${result.elapsed_ms} ms | ${result.pattern}`;
      renderTable(); drawOverlay();
    }
    function stopPreview() {
      previewRunning = false;
      previewToggle.textContent = 'Start preview';
      if (previewTimer) {
        clearTimeout(previewTimer);
        previewTimer = 0;
      }
    }
    function schedulePreview() {
      if (!previewRunning) return;
      const fps = Math.max(1, Number(previewFps.value) || 2);
      previewTimer = setTimeout(fetchPreview, Math.round(1000 / fps));
    }
    async function fetchPreview() {
      if (!previewRunning || previewBusy) return;
      previewBusy = true;
      result = null;
      drawOverlay();
      showError('');
      try {
        const response = await fetchWithTimeout(`/preview?t=${Date.now()}`, { cache: 'no-store' }, 4000);
        if (!response.ok) throw new Error(await response.text());
        const buffer = await response.arrayBuffer();
        renderRgb565(buffer);
        const count = response.headers.get('X-Capture-Count');
        if (count) statusBox.textContent = `preview running | captures ${count}`;
        result = null;
        diagnostics.textContent = 'preview only | no detection';
        drawOverlay();
      } catch (error) {
        result = null;
        drawOverlay();
        showError(error.message || String(error));
      } finally {
        previewBusy = false;
        schedulePreview();
      }
    }
    function startPreview() {
      if (previewRunning) return;
      result = null;
      drawOverlay();
      previewRunning = true;
      previewToggle.textContent = 'Pause preview';
      fetchPreview();
    }
    async function analyze() {
      stopPreview();
      showError('');
      const waitStarted = Date.now();
      while (previewBusy && Date.now() - waitStarted < 5000) {
        diagnostics.textContent = 'waiting for preview...';
        await new Promise(resolve => setTimeout(resolve, 50));
      }
      if (previewBusy) {
        showError('Preview request is still busy; wait a second or reload the page.');
        return;
      }
      try {
        diagnostics.textContent = 'analyzing...';
        const params = new URLSearchParams({ warmup: warmupValue(), t: Date.now() });
        const response = await fetchWithTimeout(`/frame?${params}`, { cache: 'no-store' }, 15000);
        if (!response.ok) throw new Error(await response.text());
        const buffer = await response.arrayBuffer();
        renderRgb565(buffer);
        await loadStatus();
      } catch (error) { showError(error.message || String(error)); }
    }
    previewToggle.addEventListener('click', () => {
      if (previewRunning) stopPreview(); else startPreview();
    });
    previewFps.addEventListener('change', () => {
      if (!previewRunning) return;
      if (previewTimer) clearTimeout(previewTimer);
      previewTimer = 0;
      schedulePreview();
    });
    document.getElementById('analyze').addEventListener('click', analyze);
    document.getElementById('reset').addEventListener('click', async () => {
      if (!confirm('Reset v2 calibration and detector state?')) return;
      const response = await fetchWithTimeout('/settings/reset', { method: 'POST', cache: 'no-store' }, 10000);
      if (!response.ok) { showError(await response.text()); return; }
      await loadStatus(); stopPreview(); await analyze();
    });
    document.getElementById('calibrate').addEventListener('click', async () => {
      showError('');
      const params = new URLSearchParams({ cell: cellSelect.value, color: colorSelect.value, warmup: warmupValue() });
      const response = await fetchWithTimeout(`/calibrate?${params}`, { method: 'POST', cache: 'no-store' }, 15000);
      if (!response.ok) { showError(await response.text()); return; }
      await loadStatus(); stopPreview(); await analyze();
    });
    loadStatus().then(fetchPreview).catch(e => showError(e.message || String(e)));
  </script>
</body>
</html>
)HTML";

int clampValue(int value, int minValue, int maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

bool serviceLongWork() {
  longWorkCounter++;
  if ((longWorkCounter & 0x3F) == 0) {
    delay(0);
  }
  if (detectorDeadlineMs == 0) return false;
  if (static_cast<int32_t>(millis() - detectorDeadlineMs) >= 0) {
    detectorTimedOut = true;
    delay(0);
    return true;
  }
  return false;
}

int requestInt(const char *name, int fallback, int minValue, int maxValue) {
  if (!server.hasArg(name)) return fallback;
  return clampValue(server.arg(name).toInt(), minValue, maxValue);
}

void saveSettings();

int requestWarmupFrames() {
  if (!server.hasArg("warmup")) return warmupFrames;
  const int requested = requestInt("warmup", warmupFrames, 0, 8);
  if (requested != warmupFrames) {
    warmupFrames = requested;
    saveSettings();
  }
  return warmupFrames;
}

const char *colorKey(MosaicColor color) {
  return COLOR_KEYS[static_cast<int>(color)];
}

const char *colorLabel(MosaicColor color) {
  return COLOR_LABELS[static_cast<int>(color)];
}

void applyDefaultCalibration() {
  calibration[COLOR_YELLOW] = CalibrationSample(RgbColor(235, 220, 30), false);
  calibration[COLOR_GREEN] = CalibrationSample(RgbColor(55, 155, 95), false);
  calibration[COLOR_BLUE] = CalibrationSample(RgbColor(40, 105, 220), false);
  calibration[COLOR_WHITE] = CalibrationSample(RgbColor(225, 225, 220), false);
}

void saveSettings() {
  if (!settingsStorageReady) return;
  preferences.putUInt("version", SETTINGS_VERSION);
  preferences.putInt("warmup", warmupFrames);
  for (int i = 0; i < COLOR_COUNT; i++) {
    preferences.putInt(("cr" + String(i)).c_str(), calibration[i].rgb.r);
    preferences.putInt(("cg" + String(i)).c_str(), calibration[i].rgb.g);
    preferences.putInt(("cb" + String(i)).c_str(), calibration[i].rgb.b);
    preferences.putBool(("cu" + String(i)).c_str(), calibration[i].user);
  }
  settingsSaveCount++;
}

void loadSettings() {
  applyDefaultCalibration();
  settingsStorageReady = preferences.begin(SETTINGS_NAMESPACE, false);
  if (!settingsStorageReady) {
    Serial.println("settings storage unavailable");
    return;
  }
  if (preferences.getUInt("version", 0) != SETTINGS_VERSION) {
    saveSettings();
    Serial.println("v2 settings initialized to defaults");
    return;
  }
  for (int i = 0; i < COLOR_COUNT; i++) {
    calibration[i].rgb.r = static_cast<uint8_t>(clampValue(preferences.getInt(("cr" + String(i)).c_str(), calibration[i].rgb.r), 0, 255));
    calibration[i].rgb.g = static_cast<uint8_t>(clampValue(preferences.getInt(("cg" + String(i)).c_str(), calibration[i].rgb.g), 0, 255));
    calibration[i].rgb.b = static_cast<uint8_t>(clampValue(preferences.getInt(("cb" + String(i)).c_str(), calibration[i].rgb.b), 0, 255));
    calibration[i].user = preferences.getBool(("cu" + String(i)).c_str(), calibration[i].user);
  }
  warmupFrames = clampValue(preferences.getInt("warmup", warmupFrames), 0, 8);
}

void resetSettings() {
  if (settingsStorageReady) preferences.clear();
  applyDefaultCalibration();
  warmupFrames = DEFAULT_WARMUP_FRAMES;
  hasLastResult = false;
  saveSettings();
}

RgbColor decodeRgb565(const uint8_t *pixel) {
  const uint16_t raw = (static_cast<uint16_t>(pixel[0]) << 8) | pixel[1];
  return RgbColor(static_cast<uint8_t>(((raw >> 11) & 0x1F) * 255 / 31),
                  static_cast<uint8_t>(((raw >> 5) & 0x3F) * 255 / 63),
                  static_cast<uint8_t>((raw & 0x1F) * 255 / 31));
}

uint8_t pixelLuma(const RgbColor &rgb) {
  return static_cast<uint8_t>((30 * rgb.r + 59 * rgb.g + 11 * rgb.b) / 100);
}

uint8_t pixelChroma(const RgbColor &rgb) {
  const uint8_t maxValue = max(rgb.r, max(rgb.g, rgb.b));
  const uint8_t minValue = min(rgb.r, min(rgb.g, rgb.b));
  return maxValue - minValue;
}

void buildFeatureImages(const camera_fb_t *fb) {
  uint16_t histogram[256] = {};
  for (int i = 0; i < FRAME_W * FRAME_H; i++) {
    const RgbColor rgb = decodeRgb565(fb->buf + i * 2);
    luma[i] = pixelLuma(rgb);
    chroma[i] = pixelChroma(rgb);
    histogram[luma[i]]++;
    if ((i & 0x3FF) == 0) serviceLongWork();
  }

  const int target = FRAME_W * FRAME_H / 5;
  int cumulative = 0;
  uint8_t percentile20 = 55;
  for (int i = 0; i < 256; i++) {
    cumulative += histogram[i];
    if (cumulative >= target) {
      percentile20 = static_cast<uint8_t>(i);
      break;
    }
  }
  darkThreshold = static_cast<uint8_t>(clampValue(percentile20 + 16, 38, 105));
}

bool inFrame(const PointF &p) {
  return p.x >= 0 && p.y >= 0 && p.x < FRAME_W && p.y < FRAME_H;
}

PointF mapQuad(const Quad &quad, float u, float v) {
  const PointF top = {quad.tl.x + (quad.tr.x - quad.tl.x) * u,
                      quad.tl.y + (quad.tr.y - quad.tl.y) * u};
  const PointF bottom = {quad.bl.x + (quad.br.x - quad.bl.x) * u,
                         quad.bl.y + (quad.br.y - quad.bl.y) * u};
  return {top.x + (bottom.x - top.x) * v, top.y + (bottom.y - top.y) * v};
}

PointF addPoint(const PointF &a, const PointF &b) {
  return PointF(a.x + b.x, a.y + b.y);
}

PointF subPoint(const PointF &a, const PointF &b) {
  return PointF(a.x - b.x, a.y - b.y);
}

PointF mulPoint(const PointF &p, float value) {
  return PointF(p.x * value, p.y * value);
}

float pointLength(const PointF &p) {
  return sqrtf(p.x * p.x + p.y * p.y);
}

PointF normalizePoint(const PointF &p, const PointF &fallback) {
  const float length = pointLength(p);
  if (length < 0.001f) return fallback;
  return PointF(p.x / length, p.y / length);
}

Quad makeCandidateQuad(float cx, float cy, float width, float ratio, float angleDeg) {
  const float angle = angleDeg * 3.1415926f / 180.0f;
  const float ca = cosf(angle);
  const float sa = sinf(angle);
  const float halfW = width * 0.5f;
  const float halfH = width * ratio * 0.5f;
  const PointF ux(ca, sa);
  const PointF uy(-sa, ca);
  const PointF center(cx, cy);
  const PointF wx = mulPoint(ux, halfW);
  const PointF hy = mulPoint(uy, halfH);
  return Quad(subPoint(subPoint(center, wx), hy),
              subPoint(addPoint(center, wx), hy),
              addPoint(subPoint(center, wx), hy),
              addPoint(addPoint(center, wx), hy));
}

float quadArea(const Quad &quad) {
  const PointF points[4] = {quad.tl, quad.tr, quad.br, quad.bl};
  float area = 0;
  for (int i = 0; i < 4; i++) {
    const PointF &a = points[i];
    const PointF &b = points[(i + 1) % 4];
    area += a.x * b.y - b.x * a.y;
  }
  return fabsf(area) * 0.5f;
}

Quad remapQuad(const Quad &quad, float u0, float v0, float u1, float v1) {
  return Quad(mapQuad(quad, u0, v0),
              mapQuad(quad, u1, v0),
              mapQuad(quad, u0, v1),
              mapQuad(quad, u1, v1));
}

PointF mapProjective(const float h[8], float u, float v) {
  const float denominator = h[6] * u + h[7] * v + 1.0f;
  if (fabsf(denominator) < 0.001f) return PointF(-1000.0f, -1000.0f);
  return PointF((h[0] * u + h[1] * v + h[2]) / denominator,
                (h[3] * u + h[4] * v + h[5]) / denominator);
}

bool solveLinear8(float matrix[8][9], float out[8]) {
  for (int col = 0; col < 8; col++) {
    int pivot = col;
    float pivotAbs = fabsf(matrix[col][col]);
    for (int row = col + 1; row < 8; row++) {
      const float valueAbs = fabsf(matrix[row][col]);
      if (valueAbs > pivotAbs) {
        pivotAbs = valueAbs;
        pivot = row;
      }
    }
    if (pivotAbs < 0.000001f) return false;
    if (pivot != col) {
      for (int k = col; k < 9; k++) {
        const float temp = matrix[col][k];
        matrix[col][k] = matrix[pivot][k];
        matrix[pivot][k] = temp;
      }
    }

    const float divisor = matrix[col][col];
    for (int k = col; k < 9; k++) matrix[col][k] /= divisor;

    for (int row = 0; row < 8; row++) {
      if (row == col) continue;
      const float factor = matrix[row][col];
      if (fabsf(factor) < 0.000001f) continue;
      for (int k = col; k < 9; k++) matrix[row][k] -= factor * matrix[col][k];
    }
  }

  for (int i = 0; i < 8; i++) out[i] = matrix[i][8];
  return true;
}

float darkScoreAt(const PointF &p) {
  if (!inFrame(p)) return -0.35f;
  const int x = clampValue(static_cast<int>(roundf(p.x)), 0, FRAME_W - 1);
  const int y = clampValue(static_cast<int>(roundf(p.y)), 0, FRAME_H - 1);
  const uint8_t minY = luma[y * FRAME_W + x];
  if (minY <= darkThreshold) return 1.0f;
  const int softHigh = min(150, darkThreshold + 45);
  if (minY < softHigh) return (softHigh - minY) / static_cast<float>(softHigh - darkThreshold);
  return -0.25f;
}

float centerQualityFromLumaChroma(uint8_t yy, uint8_t cc) {
  if (yy < 32) return -0.65f;
  const float colored = yy >= 46 && cc >= 34 ? min(1.0f, (cc - 28) / 86.0f) :
                        (yy >= 34 && cc >= 48 ? min(0.75f, (cc - 34) / 95.0f) : -0.35f);
  const float white = yy >= 150 && cc <= 60 ? min(1.0f, (yy - 130) / 75.0f) : -0.35f;
  const float quality = max(colored, white);
  return quality < 0.0f ? -0.55f : quality;
}

float centerScoreAt(const PointF &p) {
  if (!inFrame(p)) return -0.35f;
  const int x = clampValue(static_cast<int>(roundf(p.x)), 0, FRAME_W - 1);
  const int y = clampValue(static_cast<int>(roundf(p.y)), 0, FRAME_H - 1);
  uint16_t sumY = 0;
  uint16_t sumC = 0;
  uint8_t count = 0;
  const int offsets[5][2] = {{0, 0}, {-1, 0}, {1, 0}, {0, -1}, {0, 1}};
  for (const auto &offset : offsets) {
    const int sx = x + offset[0];
    const int sy = y + offset[1];
    if (sx < 0 || sx >= FRAME_W || sy < 0 || sy >= FRAME_H) continue;
    sumY += luma[sy * FRAME_W + sx];
    sumC += chroma[sy * FRAME_W + sx];
    count++;
  }
  if (count == 0) return -0.35f;
  return centerQualityFromLumaChroma(static_cast<uint8_t>(sumY / count),
                                    static_cast<uint8_t>(sumC / count));
}

float pointDistanceSq(const PointF &a, const PointF &b) {
  const float dx = a.x - b.x;
  const float dy = a.y - b.y;
  return dx * dx + dy * dy;
}

float dotPoint(const PointF &a, const PointF &b) {
  return a.x * b.x + a.y * b.y;
}

float crossPoint(const PointF &a, const PointF &b) {
  return a.x * b.y - a.y * b.x;
}

uint8_t countBits(uint8_t value) {
  uint8_t count = 0;
  while (value != 0) {
    count += value & 1;
    value >>= 1;
  }
  return count;
}

bool isCellInteriorPixel(int index) {
  const uint8_t yy = luma[index];
  const uint8_t cc = chroma[index];
  return (yy >= 46 && cc >= 28) || (yy >= 34 && cc >= 48) || (yy >= 138 && cc <= 92);
}

void buildCellMask() {
  for (int i = 0; i < FRAME_PIXELS; i++) {
    cellMask[i] = isCellInteriorPixel(i) ? 1 : 0;
    if ((i & 0x3FF) == 0) serviceLongWork();
  }
}

bool ensureFloodQueue() {
  if (floodQueue != nullptr) return true;
  floodQueue = static_cast<uint16_t *>(malloc(FRAME_PIXELS * sizeof(uint16_t)));
  if (floodQueue == nullptr) {
    Serial.println("blob detector: flood queue allocation failed");
    return false;
  }
  return true;
}

uint8_t borderDarkPercent(const BlobComponent &component) {
  uint16_t dark = 0;
  uint16_t total = 0;
  const int minX = static_cast<int>(component.minX) - 2;
  const int minY = static_cast<int>(component.minY) - 2;
  const int maxX = static_cast<int>(component.maxX) + 2;
  const int maxY = static_cast<int>(component.maxY) + 2;

  for (int y = minY; y <= maxY; y++) {
    for (int x = minX; x <= maxX; x++) {
      if (x < 0 || x >= FRAME_W || y < 0 || y >= FRAME_H) continue;
      const bool nearHorizontal = y < static_cast<int>(component.minY) || y > static_cast<int>(component.maxY);
      const bool nearVertical = x < static_cast<int>(component.minX) || x > static_cast<int>(component.maxX);
      if (!nearHorizontal && !nearVertical) continue;
      total++;
      if (luma[y * FRAME_W + x] <= darkThreshold + 10) dark++;
      if ((total & 0x3F) == 0) serviceLongWork();
    }
  }
  if (total == 0) return 0;
  return static_cast<uint8_t>(clampValue(dark * 100 / total, 0, 100));
}

uint8_t borderDarkSideCount(const BlobComponent &component) {
  uint16_t dark[4] = {};
  uint16_t total[4] = {};
  const int minX = static_cast<int>(component.minX) - 3;
  const int minY = static_cast<int>(component.minY) - 3;
  const int maxX = static_cast<int>(component.maxX) + 3;
  const int maxY = static_cast<int>(component.maxY) + 3;

  for (int y = minY; y <= maxY; y++) {
    for (int x = minX; x <= maxX; x++) {
      if (x < 0 || x >= FRAME_W || y < 0 || y >= FRAME_H) continue;
      int side = -1;
      if (y < static_cast<int>(component.minY) && x >= static_cast<int>(component.minX) && x <= static_cast<int>(component.maxX)) {
        side = 0;
      } else if (y > static_cast<int>(component.maxY) && x >= static_cast<int>(component.minX) && x <= static_cast<int>(component.maxX)) {
        side = 1;
      } else if (x < static_cast<int>(component.minX) && y >= static_cast<int>(component.minY) && y <= static_cast<int>(component.maxY)) {
        side = 2;
      } else if (x > static_cast<int>(component.maxX) && y >= static_cast<int>(component.minY) && y <= static_cast<int>(component.maxY)) {
        side = 3;
      }
      if (side < 0) continue;
      total[side]++;
      if (luma[y * FRAME_W + x] <= darkThreshold + 12) dark[side]++;
      if (((total[0] + total[1] + total[2] + total[3]) & 0x3F) == 0) serviceLongWork();
    }
  }

  uint8_t sides = 0;
  for (int i = 0; i < 4; i++) {
    if (total[i] > 0 && dark[i] * 100 / total[i] >= 16) sides++;
  }
  return sides;
}

bool finalizeComponent(BlobComponent &component) {
  if (component.area == 0) return false;
  const int width = static_cast<int>(component.maxX - component.minX + 1);
  const int height = static_cast<int>(component.maxY - component.minY + 1);
  const int boxArea = max(1, width * height);
  component.cx = component.sumX / static_cast<float>(component.area);
  component.cy = component.sumY / static_cast<float>(component.area);
  component.meanR = static_cast<uint8_t>(component.sumR / component.area);
  component.meanG = static_cast<uint8_t>(component.sumG / component.area);
  component.meanB = static_cast<uint8_t>(component.sumB / component.area);
  component.meanLuma = static_cast<uint8_t>(component.sumLuma / component.area);
  component.meanChroma = static_cast<uint8_t>(component.sumChroma / component.area);
  component.fillRatio = static_cast<uint8_t>(clampValue(component.area * 100 / boxArea, 0, 100));
  component.borderDark = borderDarkPercent(component);
  component.borderDarkSides = borderDarkSideCount(component);

  if (component.area < 16 || component.area > 820) return false;
  if (width < 4 || height < 4 || width > 42 || height > 42) return false;
  const float ratio = width / static_cast<float>(height);
  if (ratio < 0.38f || ratio > 2.65f) return false;
  if (component.fillRatio < 22) return false;
  if (component.meanLuma > 225 && component.meanChroma < 16 && component.area > 260) return false;
  if (component.meanLuma > 235 && component.meanChroma < 18 && component.area > 220) return false;

  const float interior = centerQualityFromLumaChroma(component.meanLuma, component.meanChroma);
  if (interior < -0.25f) return false;
  const int shapePenalty = static_cast<int>(fabsf(ratio - 1.0f) * 20.0f);
  const int borderBonus = min(35, component.borderDark / 2 + component.borderDarkSides * 5);
  const int quality = 30 + static_cast<int>(interior * 45.0f) +
                      borderBonus + component.fillRatio / 5 - shapePenalty;
  component.quality = static_cast<uint8_t>(clampValue(quality, 0, 100));
  return component.quality >= 24;
}

void considerComponent(BlobComponent filtered[], uint8_t &count, const BlobComponent &component) {
  if (count < MAX_COMPONENTS) {
    filtered[count++] = component;
    return;
  }

  int weakest = 0;
  for (int i = 1; i < MAX_COMPONENTS; i++) {
    if (filtered[i].quality < filtered[weakest].quality) weakest = i;
  }
  if (component.quality > filtered[weakest].quality) filtered[weakest] = component;
}

uint8_t detectCellComponents(const camera_fb_t *fb, BlobComponent filtered[]) {
  if (!ensureFloodQueue()) return 0;
  buildCellMask();
  uint8_t count = 0;
  for (int start = 0; start < FRAME_PIXELS; start++) {
    if ((start & 0x1FF) == 0 && serviceLongWork()) return count;
    if (cellMask[start] == 0) continue;

    BlobComponent component;
    uint16_t head = 0;
    uint16_t tail = 0;
    floodQueue[tail++] = start;
    cellMask[start] = 0;

    while (head < tail) {
      if ((component.area & 0x7F) == 0 && serviceLongWork()) return count;
      const uint16_t index = floodQueue[head++];
      const int x = index % FRAME_W;
      const int y = index / FRAME_W;
      const RgbColor rgb = decodeRgb565(fb->buf + index * 2);
      component.area++;
      component.sumX += x;
      component.sumY += y;
      component.sumR += rgb.r;
      component.sumG += rgb.g;
      component.sumB += rgb.b;
      component.sumLuma += luma[index];
      component.sumChroma += chroma[index];
      if (x < component.minX) component.minX = x;
      if (y < component.minY) component.minY = y;
      if (x > component.maxX) component.maxX = x;
      if (y > component.maxY) component.maxY = y;

      const int neighbors[4] = {index - 1, index + 1, index - FRAME_W, index + FRAME_W};
      if (x > 0 && cellMask[neighbors[0]] != 0) {
        cellMask[neighbors[0]] = 0;
        floodQueue[tail++] = static_cast<uint16_t>(neighbors[0]);
      }
      if (x < FRAME_W - 1 && cellMask[neighbors[1]] != 0) {
        cellMask[neighbors[1]] = 0;
        floodQueue[tail++] = static_cast<uint16_t>(neighbors[1]);
      }
      if (y > 0 && cellMask[neighbors[2]] != 0) {
        cellMask[neighbors[2]] = 0;
        floodQueue[tail++] = static_cast<uint16_t>(neighbors[2]);
      }
      if (y < FRAME_H - 1 && cellMask[neighbors[3]] != 0) {
        cellMask[neighbors[3]] = 0;
        floodQueue[tail++] = static_cast<uint16_t>(neighbors[3]);
      }
    }
    if (finalizeComponent(component)) {
      considerComponent(filtered, count, component);
    }
  }
  return count;
}

float scoreLine(const PointF &a, const PointF &b, int samples) {
  float total = 0;
  for (int i = 0; i <= samples; i++) {
    const float t = i / static_cast<float>(samples);
    total += darkScoreAt(PointF(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t));
  }
  serviceLongWork();
  return total / (samples + 1);
}

void considerStep(StepCandidate steps[], uint8_t &count, const PointF &step, float score) {
  if (count < MAX_STEP_CANDIDATES) {
    steps[count].step = step;
    steps[count].score = score;
    count++;
    return;
  }
  int weakest = 0;
  for (int i = 1; i < MAX_STEP_CANDIDATES; i++) {
    if (steps[i].score < steps[weakest].score) weakest = i;
  }
  if (score > steps[weakest].score) {
    steps[weakest].step = step;
    steps[weakest].score = score;
  }
}

void buildStepCandidates(const BlobComponent comps[], uint8_t count,
                         StepCandidate xSteps[], uint8_t &xCount,
                         StepCandidate ySteps[], uint8_t &yCount) {
  xCount = 0;
  yCount = 0;
  for (int i = 0; i < count; i++) {
    for (int j = i + 1; j < count; j++) {
      if (serviceLongWork()) return;
      PointF delta(comps[j].cx - comps[i].cx, comps[j].cy - comps[i].cy);
      const float baseScore = comps[i].quality + comps[j].quality;
      for (int span = 1; span <= 3; span++) {
        PointF step(delta.x / span, delta.y / span);
        if (step.x < 0) step = mulPoint(step, -1.0f);
        const float length = pointLength(step);
        if (length < 7.0f || length > 42.0f) continue;
        if (step.x < 3.0f || fabsf(step.y) > fabsf(step.x) * 1.45f) continue;
        considerStep(xSteps, xCount, step, baseScore - span * 6.0f - fabsf(step.y) * 0.3f);
      }

      for (int span = 1; span <= 2; span++) {
        PointF step(delta.x / span, delta.y / span);
        if (step.y < 0) step = mulPoint(step, -1.0f);
        const float length = pointLength(step);
        if (length < 7.0f || length > 42.0f) continue;
        if (step.y < 3.0f || fabsf(step.x) > fabsf(step.y) * 1.70f) continue;
        considerStep(ySteps, yCount, step, baseScore - span * 6.0f - fabsf(step.x) * 0.2f);
      }
    }
  }
}

PointF latticePoint(const PointF &origin, const PointF &xStep, const PointF &yStep, float col, float row) {
  return addPoint(origin, addPoint(mulPoint(xStep, col), mulPoint(yStep, row)));
}

Quad latticeOuterQuad(const PointF &origin, const PointF &xStep, const PointF &yStep) {
  return Quad(latticePoint(origin, xStep, yStep, -0.5f, -0.5f),
              latticePoint(origin, xStep, yStep, COLS - 0.5f, -0.5f),
              latticePoint(origin, xStep, yStep, -0.5f, ROWS - 0.5f),
              latticePoint(origin, xStep, yStep, COLS - 0.5f, ROWS - 0.5f));
}

float latticeGridScore(const PointF &origin, const PointF &xStep, const PointF &yStep) {
  float total = 0;
  int count = 0;
  for (int col = 0; col <= COLS; col++) {
    const PointF a = latticePoint(origin, xStep, yStep, col - 0.5f, -0.5f);
    const PointF b = latticePoint(origin, xStep, yStep, col - 0.5f, ROWS - 0.5f);
    total += scoreLine(a, b, 8);
    count++;
  }
  for (int row = 0; row <= ROWS; row++) {
    const PointF a = latticePoint(origin, xStep, yStep, -0.5f, row - 0.5f);
    const PointF b = latticePoint(origin, xStep, yStep, COLS - 0.5f, row - 0.5f);
    total += scoreLine(a, b, 10);
    count++;
  }
  return 100.0f * total / max(1, count);
}

LatticeFit evaluateLattice(const PointF &origin, const PointF &xStep, const PointF &yStep,
                           const BlobComponent comps[], uint8_t count) {
  LatticeFit fit;
  for (int i = 0; i < POINT_COUNT; i++) fit.componentForCell[i] = -1;
  const float xLen = pointLength(xStep);
  const float yLen = pointLength(yStep);
  if (xLen < 7.0f || xLen > 42.0f || yLen < 7.0f || yLen > 42.0f) return fit;
  if (xStep.x <= 2.0f || yStep.y <= 2.0f) return fit;
  const float cross = crossPoint(xStep, yStep);
  if (cross <= 32.0f) return fit;
  const float dotNorm = fabsf(dotPoint(xStep, yStep)) / max(1.0f, xLen * yLen);
  if (dotNorm > 0.78f) return fit;
  if (xLen < yLen * 0.55f) return fit;

  uint32_t used = 0;
  uint8_t rowMask = 0;
  uint8_t colMask = 0;
  float residualSum = 0;
  uint16_t qualitySum = 0;
  uint8_t outside = 0;
  const float radius = min(11.0f, max(4.0f, min(xLen, yLen) * 0.46f));
  const float radiusSq = radius * radius;

  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < COLS; col++) {
      const int cellIndex = row * COLS + col;
      const PointF predicted = latticePoint(origin, xStep, yStep, col, row);
      if (!inFrame(predicted)) outside++;
      int bestComponent = -1;
      float bestDistanceSq = radiusSq;
      for (int i = 0; i < count; i++) {
        if ((used & (1UL << i)) != 0) continue;
        const PointF center(comps[i].cx, comps[i].cy);
        const float distanceSq = pointDistanceSq(predicted, center);
        if (distanceSq < bestDistanceSq) {
          bestDistanceSq = distanceSq;
          bestComponent = i;
        }
      }
      if (bestComponent >= 0) {
        used |= 1UL << bestComponent;
        fit.componentMask |= 1UL << bestComponent;
        fit.matchMask |= 1U << cellIndex;
        fit.componentForCell[cellIndex] = static_cast<int8_t>(bestComponent);
        fit.matchedBlobs++;
        rowMask |= 1U << row;
        colMask |= 1U << col;
        residualSum += sqrtf(bestDistanceSq);
        qualitySum += comps[bestComponent].quality;
      }
    }
  }

  fit.valid = true;
  fit.origin = origin;
  fit.xStep = xStep;
  fit.yStep = yStep;
  fit.quad = latticeOuterQuad(origin, xStep, yStep);
  fit.residualPx = fit.matchedBlobs == 0 ? 99.0f : residualSum / fit.matchedBlobs;
  fit.gridScore = latticeGridScore(origin, xStep, yStep);
  const uint8_t rowCoverage = countBits(rowMask);
  const uint8_t colCoverage = countBits(colMask);
  const float qualityAvg = fit.matchedBlobs == 0 ? 0.0f : qualitySum / static_cast<float>(fit.matchedBlobs);
  fit.blobScore = fit.matchedBlobs * 100.0f / POINT_COUNT;
  fit.outsideRatio = outside / static_cast<float>(POINT_COUNT);
  fit.score = fit.matchedBlobs * 13.0f + rowCoverage * 7.0f + colCoverage * 6.0f +
              fit.gridScore * 0.48f + qualityAvg * 0.10f -
              fit.residualPx * 2.3f - fit.outsideRatio * 24.0f;
  if (rowCoverage == ROWS && colCoverage == COLS) fit.score += 24.0f;
  if (rowCoverage < ROWS) fit.score -= 18.0f;
  if (colCoverage < COLS) fit.score -= 24.0f;
  return fit;
}

LatticeFit fitBestLattice(const BlobComponent comps[], uint8_t count) {
  LatticeFit best;
  uint8_t xCount = 0;
  uint8_t yCount = 0;
  buildStepCandidates(comps, count, xStepCandidates, xCount, yStepCandidates, yCount);
  if (xCount == 0 || yCount == 0 || count == 0) return best;

  for (int xi = 0; xi < xCount; xi++) {
    for (int yi = 0; yi < yCount; yi++) {
      const PointF &xStep = xStepCandidates[xi].step;
      const PointF &yStep = yStepCandidates[yi].step;
      const float xLen = pointLength(xStep);
      const float yLen = pointLength(yStep);
      if (crossPoint(xStep, yStep) <= 32.0f) continue;
      if (fabsf(dotPoint(xStep, yStep)) / max(1.0f, xLen * yLen) > 0.78f) continue;

      for (int anchor = 0; anchor < count; anchor++) {
        const PointF anchorPoint(comps[anchor].cx, comps[anchor].cy);
        for (int row = 0; row < ROWS; row++) {
          for (int col = 0; col < COLS; col++) {
            const PointF origin = subPoint(anchorPoint, addPoint(mulPoint(xStep, col), mulPoint(yStep, row)));
            const LatticeFit fit = evaluateLattice(origin, xStep, yStep, comps, count);
            if (fit.valid && fit.score > best.score) best = fit;
            if (serviceLongWork()) return best;
          }
        }
      }
    }
  }
  return best;
}

float extraGridScoreAt(const Quad &quad, float coord, bool vertical) {
  float total = 0;
  int count = 0;
  for (int i = 1; i <= 7; i++) {
    const float t = i / 8.0f;
    const PointF p = vertical ? mapQuad(quad, coord, t) : mapQuad(quad, t, coord);
    if (!inFrame(p)) continue;
    const float dark = darkScoreAt(p);
    total += max(0.0f, dark);
    count++;
  }
  if (count == 0) return 0.0f;
  return total / count;
}

CandidateScore scoreQuadDetailed(const Quad &quad) {
  float lineTotal = 0;
  int lineCount = 0;
  int outsideCount = 0;
  int sampleCount = 0;
  const float cols[5] = {0, 0.25f, 0.5f, 0.75f, 1};
  const float rows[4] = {0, 1.0f / 3.0f, 2.0f / 3.0f, 1};
  for (float u : cols) {
    for (int i = 0; i <= 8; i++) {
      const PointF p = mapQuad(quad, u, i / 8.0f);
      if (!inFrame(p)) outsideCount++;
      lineTotal += darkScoreAt(p);
      lineCount++;
      sampleCount++;
    }
  }
  for (float v : rows) {
    for (int i = 0; i <= 10; i++) {
      const PointF p = mapQuad(quad, i / 10.0f, v);
      if (!inFrame(p)) outsideCount++;
      lineTotal += darkScoreAt(p);
      lineCount++;
      sampleCount++;
    }
  }

  float centerTotal = 0;
  int centerCount = 0;
  int visibleCells = 0;
  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < COLS; col++) {
      const PointF p = mapQuad(quad, (col + 0.5f) / COLS, (row + 0.5f) / ROWS);
      if (!inFrame(p)) outsideCount++;
      const float centerScore = centerScoreAt(p);
      centerTotal += centerScore;
      if (inFrame(p) && centerScore > 0.22f) visibleCells++;
      centerCount++;
      sampleCount++;
    }
  }

  const float lineScore = lineTotal / max(1, lineCount);
  const float centerScore = centerTotal / max(1, centerCount);
  CandidateScore metrics;
  metrics.outsideRatio = outsideCount / static_cast<float>(max(1, sampleCount));
  metrics.visibleCells = static_cast<uint8_t>(visibleCells);
  metrics.extraGridScore = 0.25f * (
      extraGridScoreAt(quad, -0.25f, true) +
      extraGridScoreAt(quad, 1.25f, true) +
      extraGridScoreAt(quad, -1.0f / 3.0f, false) +
      extraGridScoreAt(quad, 1.0f + 1.0f / 3.0f, false));
  const float visibleScore = visibleCells / static_cast<float>(POINT_COUNT);
  metrics.score = 100.0f * (0.38f * lineScore + 0.46f * centerScore +
                            0.20f * visibleScore - 0.36f * metrics.outsideRatio -
                            0.28f * metrics.extraGridScore);
  const float areaRatio = quadArea(quad) / static_cast<float>(FRAME_W * FRAME_H);
  const float sizeBonus = min(1.0f, max(0.0f, (areaRatio - 0.16f) / 0.20f));
  const float smallPenalty = areaRatio < 0.17f ? (0.17f - areaRatio) * 95.0f : 0.0f;
  metrics.score += 12.0f * sizeBonus - smallPenalty;
  return metrics;
}

Candidate makeCandidate(float cx, float cy, float width, float ratio, float angle) {
  Candidate candidate;
  candidate.cx = cx;
  candidate.cy = cy;
  candidate.width = width;
  candidate.ratio = ratio;
  candidate.angle = angle;
  candidate.quad = makeCandidateQuad(cx, cy, width, ratio, angle);
  candidate.metrics = scoreQuadDetailed(candidate.quad);
  return candidate;
}

void considerCandidate(Candidate top[], const Candidate &candidate) {
  for (int i = 0; i < TOP_CANDIDATE_COUNT; i++) {
    if (candidate.metrics.score > top[i].metrics.score) {
      for (int j = TOP_CANDIDATE_COUNT - 1; j > i; j--) {
        top[j] = top[j - 1];
      }
      top[i] = candidate;
      return;
    }
  }
}

Quad refineOuterBorders(const Quad &quad);

GridSearchFit gridFitFromCandidate(const Candidate &candidate) {
  GridSearchFit fit;
  const float areaRatio = quadArea(candidate.quad) / static_cast<float>(FRAME_PIXELS);
  if (areaRatio < 0.13f || areaRatio > 0.68f) return fit;
  if (candidate.metrics.visibleCells < 8) return fit;
  if (candidate.metrics.outsideRatio > 0.26f) return fit;
  if (candidate.metrics.score < 18.0f) return fit;

  fit.valid = true;
  fit.quad = candidate.quad;
  fit.score = candidate.metrics.score;
  fit.gridScore = clampValue(static_cast<int>(candidate.metrics.score), 0, 100);
  fit.outsideRatio = candidate.metrics.outsideRatio;
  fit.visibleCells = candidate.metrics.visibleCells;
  return fit;
}

void considerGridFit(GridSearchFit &best, const Candidate &candidate) {
  GridSearchFit fit = gridFitFromCandidate(candidate);
  if (fit.valid && fit.score > best.score) best = fit;
}

GridSearchFit fitGridByFullFrameSearch() {
  Candidate top[TOP_CANDIDATE_COUNT];
  GridSearchFit best;
  const float ratios[] = {0.62f, 0.74f, 0.86f};
  for (float width = 64.0f; width <= 128.0f; width += 12.0f) {
    for (float cy = 28.0f; cy <= 94.0f; cy += 10.0f) {
      for (float cx = 34.0f; cx <= 126.0f; cx += 10.0f) {
        for (float angle = -58.0f; angle <= 58.0f; angle += 10.0f) {
          for (float ratio : ratios) {
            considerCandidate(top, makeCandidate(cx, cy, width, ratio, angle));
          }
          if (serviceLongWork()) goto refine;
        }
      }
    }
  }

refine:
  for (int i = 0; i < TOP_CANDIDATE_COUNT; i++) {
    if (top[i].metrics.score < -99999.0f) continue;
    considerGridFit(best, top[i]);

    for (float dx = -8.0f; dx <= 8.0f; dx += 4.0f) {
      for (float dy = -8.0f; dy <= 8.0f; dy += 4.0f) {
        for (float dw = -8.0f; dw <= 8.0f; dw += 4.0f) {
          for (float da = -6.0f; da <= 6.0f; da += 3.0f) {
            for (float dr = -0.08f; dr <= 0.08f; dr += 0.04f) {
              Candidate refined = makeCandidate(top[i].cx + dx, top[i].cy + dy,
                                                top[i].width + dw,
                                                top[i].ratio + dr,
                                                top[i].angle + da);
              considerGridFit(best, refined);
            }
          }
        }
      }
    }
  }

  if (best.valid) {
    Quad refined = refineOuterBorders(best.quad);
    CandidateScore refinedScore = scoreQuadDetailed(refined);
    if (refinedScore.score >= best.score - 8.0f && refinedScore.visibleCells >= 8) {
      best.quad = refined;
      best.score = max(best.score, refinedScore.score);
      best.gridScore = clampValue(static_cast<int>(best.score), 0, 100);
      best.outsideRatio = refinedScore.outsideRatio;
      best.visibleCells = refinedScore.visibleCells;
    }
  }
  return best;
}

Quad refineOuterBorders(const Quad &quad) {
  Quad refined = quad;
  const PointF uFallback(1, 0);
  const PointF vFallback(0, 1);
  const PointF uAxis = normalizePoint(addPoint(subPoint(refined.tr, refined.tl),
                                               subPoint(refined.br, refined.bl)), uFallback);
  const PointF vAxis = normalizePoint(addPoint(subPoint(refined.bl, refined.tl),
                                               subPoint(refined.br, refined.tr)), vFallback);

  float bestTop = 0, bestBottom = 0, bestLeft = 0, bestRight = 0;
  float bestTopScore = -1000, bestBottomScore = -1000, bestLeftScore = -1000, bestRightScore = -1000;
  for (float shift = -6; shift <= 6; shift += 2) {
    const PointF vShift = mulPoint(vAxis, shift);
    float score = scoreLine(addPoint(refined.tl, vShift), addPoint(refined.tr, vShift), 18);
    if (score > bestTopScore) {
      bestTopScore = score;
      bestTop = shift;
    }
    score = scoreLine(addPoint(refined.bl, vShift), addPoint(refined.br, vShift), 18);
    if (score > bestBottomScore) {
      bestBottomScore = score;
      bestBottom = shift;
    }

    const PointF uShift = mulPoint(uAxis, shift);
    score = scoreLine(addPoint(refined.tl, uShift), addPoint(refined.bl, uShift), 14);
    if (score > bestLeftScore) {
      bestLeftScore = score;
      bestLeft = shift;
    }
    score = scoreLine(addPoint(refined.tr, uShift), addPoint(refined.br, uShift), 14);
    if (score > bestRightScore) {
      bestRightScore = score;
      bestRight = shift;
    }
  }

  refined.tl = addPoint(addPoint(refined.tl, mulPoint(vAxis, bestTop)), mulPoint(uAxis, bestLeft));
  refined.tr = addPoint(addPoint(refined.tr, mulPoint(vAxis, bestTop)), mulPoint(uAxis, bestRight));
  refined.bl = addPoint(addPoint(refined.bl, mulPoint(vAxis, bestBottom)), mulPoint(uAxis, bestLeft));
  refined.br = addPoint(addPoint(refined.br, mulPoint(vAxis, bestBottom)), mulPoint(uAxis, bestRight));
  return refined;
}

Quad expandSubgridCandidate(const Quad &quad) {
  Quad bestQuad = quad;
  CandidateScore bestMetrics = scoreQuadDetailed(bestQuad);
  const float starts[] = {-0.333f, -0.167f, 0.0f};
  const float ends[] = {1.0f, 1.167f, 1.333f};
  const float rowStarts[] = {-0.333f, 0.0f};
  const float rowEnds[] = {1.0f, 1.333f};

  for (float u0 : starts) {
    for (float u1 : ends) {
      if (u1 - u0 < 0.85f || u1 - u0 > 1.55f) continue;
      for (float v0 : rowStarts) {
        for (float v1 : rowEnds) {
          if (v1 - v0 < 0.85f || v1 - v0 > 1.55f) continue;
          const Quad candidate = remapQuad(quad, u0, v0, u1, v1);
          const CandidateScore metrics = scoreQuadDetailed(candidate);
          if (metrics.score > bestMetrics.score) {
            bestMetrics = metrics;
            bestQuad = candidate;
          }
        }
      }
    }
  }

  return bestQuad;
}

RgbColor sampleCell(const camera_fb_t *fb, const PointF &center, uint8_t &coverage) {
  uint32_t sumR = 0, sumG = 0, sumB = 0, count = 0;
  uint32_t fallbackR = 0, fallbackG = 0, fallbackB = 0, fallbackCount = 0;
  uint32_t expectedCount = 0;
  const int cx = static_cast<int>(roundf(center.x));
  const int cy = static_cast<int>(roundf(center.y));
  for (int y = cy - SAMPLE_RADIUS; y <= cy + SAMPLE_RADIUS; y++) {
    for (int x = cx - SAMPLE_RADIUS; x <= cx + SAMPLE_RADIUS; x++) {
      expectedCount++;
      if (y < 0 || y >= FRAME_H) continue;
      if (x < 0 || x >= FRAME_W) continue;
      const RgbColor rgb = decodeRgb565(fb->buf + (y * FRAME_W + x) * 2);
      const uint8_t yy = pixelLuma(rgb);
      fallbackR += rgb.r;
      fallbackG += rgb.g;
      fallbackB += rgb.b;
      fallbackCount++;
      if (yy < 35) continue;
      sumR += rgb.r;
      sumG += rgb.g;
      sumB += rgb.b;
      count++;
    }
  }
  const uint32_t coverageDenominator = expectedCount == 0 ? 1 : expectedCount;
  coverage = static_cast<uint8_t>(clampValue(static_cast<int>(fallbackCount * 100 / coverageDenominator), 0, 100));
  if (count == 0) {
    count = fallbackCount == 0 ? 1 : fallbackCount;
    sumR = fallbackR;
    sumG = fallbackG;
    sumB = fallbackB;
  }
  return RgbColor(static_cast<uint8_t>(sumR / count),
                  static_cast<uint8_t>(sumG / count),
                  static_cast<uint8_t>(sumB / count));
}

void normalizedRgb(const RgbColor &rgb, float &rn, float &gn, float &bn) {
  const float sum = max(1.0f, static_cast<float>(rgb.r) + rgb.g + rgb.b);
  rn = rgb.r / sum;
  gn = rgb.g / sum;
  bn = rgb.b / sum;
}

MosaicColor classifyColor(const RgbColor &rgb, uint8_t &confidence) {
  float rn, gn, bn;
  normalizedRgb(rgb, rn, gn, bn);
  const uint8_t yy = pixelLuma(rgb);
  const uint8_t cc = pixelChroma(rgb);
  int bestIndex = 0;
  float best = 100000.0f;
  float second = 100000.0f;
  for (int i = 0; i < COLOR_COUNT; i++) {
    float cr, cg, cb;
    normalizedRgb(calibration[i].rgb, cr, cg, cb);
    float d = sqrtf((rn - cr) * (rn - cr) + (gn - cg) * (gn - cg) + (bn - cb) * (bn - cb));
    if (i == COLOR_WHITE) {
      d += cc > 45 ? 0.25f : 0.0f;
      d += yy < 105 ? 0.25f : 0.0f;
    }
    if (d < best) {
      second = best;
      best = d;
      bestIndex = i;
    } else if (d < second) {
      second = d;
    }
  }
  const int absolute = clampValue(100 - static_cast<int>(best * 260), 0, 100);
  const int margin = second < 99999.0f ? clampValue(static_cast<int>((second - best) * 260), 0, 100) : absolute;
  int adjustedConfidence = (absolute + margin) / 2;
  if (bestIndex == COLOR_WHITE) {
    if (yy < 135 || cc > 75) adjustedConfidence = min(adjustedConfidence, 25);
  } else {
    if (yy < 38 || cc < 22) {
      adjustedConfidence = min(adjustedConfidence, 25);
    } else if (yy < 58 || cc < 34) {
      adjustedConfidence = min(adjustedConfidence, 55);
    }
  }

  if (bestIndex == COLOR_BLUE && rgb.b > rgb.g + 18 && rgb.b > rgb.r + 26 && yy >= 34) {
    adjustedConfidence = max(adjustedConfidence, 58);
  } else if (bestIndex == COLOR_GREEN && rgb.g > rgb.r + 18 && rgb.g > rgb.b + 14 && yy >= 38) {
    adjustedConfidence = max(adjustedConfidence, 52);
  } else if (bestIndex == COLOR_YELLOW && rgb.r > rgb.b + 34 && rgb.g > rgb.b + 26 && yy >= 70) {
    adjustedConfidence = max(adjustedConfidence, 58);
  }
  confidence = static_cast<uint8_t>(clampValue(adjustedConfidence, 0, 100));
  return static_cast<MosaicColor>(bestIndex);
}

void fillGridPoints(DetectionResult &result) {
  for (int row = 0; row <= ROWS; row++) {
    for (int col = 0; col <= COLS; col++) {
      const int index = row * (COLS + 1) + col;
      if (result.projective) {
        result.grid[index] = mapProjective(result.homography, col - 0.5f, row - 0.5f);
      } else {
        result.grid[index] = mapQuad(result.quad, col / static_cast<float>(COLS),
                                     row / static_cast<float>(ROWS));
      }
    }
  }
}

PointF predictedCellCenter(const DetectionResult &result, int col, int row) {
  if (result.projective) return mapProjective(result.homography, col, row);
  return mapQuad(result.quad, (col + 0.5f) / COLS, (row + 0.5f) / ROWS);
}

bool fitProjectiveGrid(DetectionResult &result, const LatticeFit &fit, const BlobComponent comps[]) {
  if (!fit.valid || fit.matchedBlobs < 4) return false;

  float normal[8][9] = {};
  int pairs = 0;
  for (int index = 0; index < POINT_COUNT; index++) {
    const int componentIndex = fit.componentForCell[index];
    if (componentIndex < 0) continue;
    const float u = index % COLS;
    const float v = index / COLS;
    const float x = comps[componentIndex].cx;
    const float y = comps[componentIndex].cy;
    const float rows[2][9] = {
        {u, v, 1.0f, 0, 0, 0, -u * x, -v * x, x},
        {0, 0, 0, u, v, 1.0f, -u * y, -v * y, y}
    };

    for (int equation = 0; equation < 2; equation++) {
      for (int r = 0; r < 8; r++) {
        for (int c = 0; c < 8; c++) {
          normal[r][c] += rows[equation][r] * rows[equation][c];
        }
        normal[r][8] += rows[equation][r] * rows[equation][8];
      }
    }
    pairs++;
  }
  if (pairs < 4) return false;

  for (int i = 0; i < 8; i++) normal[i][i] += 0.00001f;
  float h[8] = {};
  if (!solveLinear8(normal, h)) return false;

  float residual = 0.0f;
  for (int index = 0; index < POINT_COUNT; index++) {
    const int componentIndex = fit.componentForCell[index];
    if (componentIndex < 0) continue;
    const PointF mapped = mapProjective(h, index % COLS, index / COLS);
    residual += sqrtf(pointDistanceSq(mapped, PointF(comps[componentIndex].cx, comps[componentIndex].cy)));
  }
  residual /= max(1, pairs);
  if (residual > 7.5f) return false;

  for (int i = 0; i < 8; i++) result.homography[i] = h[i];
  result.projective = true;
  result.quad = Quad(mapProjective(h, -0.5f, -0.5f),
                     mapProjective(h, COLS - 0.5f, -0.5f),
                     mapProjective(h, -0.5f, ROWS - 0.5f),
                     mapProjective(h, COLS - 0.5f, ROWS - 0.5f));
  return quadArea(result.quad) >= 250.0f;
}

void fillResultCells(const camera_fb_t *fb, DetectionResult &result) {
  result.visibleCells = 0;
  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < COLS; col++) {
      const int index = row * COLS + col;
      CellResult &cell = result.cells[index];
      cell.blobMatched = (result.matchMask & (1U << index)) != 0;
      cell.center = predictedCellCenter(result, col, row);
      cell.inFrame = inFrame(cell.center);
      cell.rgb = sampleCell(fb, cell.center, cell.coverage);
      cell.color = classifyColor(cell.rgb, cell.confidence);
      cell.confidence = static_cast<uint8_t>(cell.confidence * cell.coverage / 100);
      if (!cell.blobMatched) cell.confidence = static_cast<uint8_t>(cell.confidence * 78 / 100);
      if (cell.inFrame && cell.coverage >= 45) result.visibleCells++;
    }
  }
  result.complete = result.visibleCells == POINT_COUNT;
}

DetectionResult detectMosaic(const camera_fb_t *fb) {
  const uint32_t startedAt = millis();
  detectorDeadlineMs = startedAt + DETECTOR_TIME_BUDGET_MS;
  detectorTimedOut = false;
  longWorkCounter = 0;
  buildFeatureImages(fb);
  DetectionResult best;
  for (int i = 0; i < POINT_COUNT; i++) best.componentForCell[i] = -1;
  best.quad = makeCandidateQuad(FRAME_W * 0.5f, FRAME_H * 0.5f, 95.0f, 0.75f, 0.0f);
  best.score = 0;

  const uint8_t componentCount = detectCellComponents(fb, components);
  best.blobCount = componentCount;

  const LatticeFit fit = fitBestLattice(components, componentCount);
  const GridSearchFit gridFit = fitGridByFullFrameSearch();
  bool usedGridSearch = false;
  if (fit.valid) {
    best.quad = fit.quad;
    best.score = fit.score;
    best.outsideRatio = fit.outsideRatio;
    best.residualPx = fit.residualPx;
    best.gridScore = fit.gridScore;
    best.blobScore = fit.blobScore;
    best.matchedBlobs = fit.matchedBlobs;
    best.matchMask = fit.matchMask;
    for (int i = 0; i < POINT_COUNT; i++) best.componentForCell[i] = fit.componentForCell[i];
    fitProjectiveGrid(best, fit, components);
  }

  if (gridFit.valid && (!fit.valid || fit.matchedBlobs < 9 || gridFit.score > best.gridScore + 18.0f)) {
    usedGridSearch = true;
    best.gridSearch = true;
    best.projective = false;
    best.quad = gridFit.quad;
    best.score = gridFit.score;
    best.outsideRatio = gridFit.outsideRatio;
    best.residualPx = 0.0f;
    best.gridScore = gridFit.gridScore;
    best.blobScore = fit.valid ? fit.blobScore : 0.0f;
    best.matchedBlobs = 0;
    best.matchMask = 0;
    for (int i = 0; i < POINT_COUNT; i++) best.componentForCell[i] = -1;
  }
  fillGridPoints(best);

  const uint32_t debugMask = usedGridSearch ? 0UL : (fit.valid ? fit.componentMask : 0xFFFFFFFFUL);
  for (int i = 0; i < componentCount && best.debugComponentCount < MAX_DEBUG_COMPONENTS; i++) {
    if ((debugMask & (1UL << i)) == 0) continue;
    DebugComponent &debug = best.debugComponents[best.debugComponentCount++];
    debug.x = static_cast<uint8_t>(components[i].minX);
    debug.y = static_cast<uint8_t>(components[i].minY);
    debug.w = static_cast<uint8_t>(components[i].maxX - components[i].minX + 1);
    debug.h = static_cast<uint8_t>(components[i].maxY - components[i].minY + 1);
    debug.score = components[i].quality;
  }
  fillResultCells(fb, best);
  if (usedGridSearch) {
    best.found = best.score >= 26.0f && best.visibleCells >= 8;
  } else {
    const bool strongGeometry = fit.valid && best.score >= 105.0f &&
                                best.gridScore >= 28.0f && best.residualPx <= 6.5f;
    const bool enoughBlobSupport = best.matchedBlobs >= 6 ||
                                   (best.matchedBlobs >= 5 && best.gridScore >= 70.0f);
    best.found = strongGeometry && enoughBlobSupport;
  }
  const int confidenceFromScore = usedGridSearch
                                      ? clampValue(static_cast<int>(best.score + best.visibleCells * 4), 0, 100)
                                      : clampValue(static_cast<int>(best.score * 0.75f), 0, 100);
  const int confidenceFromCells = usedGridSearch
                                      ? static_cast<int>(best.visibleCells) * 100 / POINT_COUNT
                                      : static_cast<int>(best.matchedBlobs) * 100 / POINT_COUNT;
  const int outsidePenalty = static_cast<int>(best.outsideRatio * 35.0f);
  best.confidence = static_cast<uint8_t>(clampValue((confidenceFromScore + confidenceFromCells) / 2 - outsidePenalty, 0, 100));
  if (!best.found && best.confidence > 42) best.confidence = 42;
  best.timedOut = detectorTimedOut;
  if (best.timedOut && !best.found) {
    if (best.confidence > 25) best.confidence = 25;
  }
  best.elapsedMs = static_cast<uint16_t>(min(65535UL, millis() - startedAt));
  detectorDeadlineMs = 0;
  return best;
}

String patternString(const DetectionResult &result) {
  String pattern;
  for (int i = 0; i < POINT_COUNT; i++) {
    if (i > 0 && i % COLS == 0) pattern += "/";
    pattern += colorLabel(result.cells[i].color);
  }
  return pattern;
}

void appendPointJson(String &json, const PointF &point) {
  json += "{\"x\":" + String(point.x, 1) + ",\"y\":" + String(point.y, 1) + "}";
}

String resultJson(const DetectionResult &result) {
  String json;
  json.reserve(4200);
  json += "{";
  json += "\"status\":\"" + String(result.found ? "found" : "best_effort") + "\",";
  json += "\"found\":" + String(result.found ? "true" : "false") + ",";
  json += "\"complete\":" + String(result.complete ? "true" : "false") + ",";
  json += "\"timed_out\":" + String(result.timedOut ? "true" : "false") + ",";
  json += "\"score\":" + String(result.score, 1) + ",";
  json += "\"elapsed_ms\":" + String(result.elapsedMs) + ",";
  json += "\"confidence\":" + String(result.confidence) + ",";
  json += "\"visible_cells\":" + String(result.visibleCells) + ",";
  json += "\"blob_count\":" + String(result.blobCount) + ",";
  json += "\"matched_blobs\":" + String(result.matchedBlobs) + ",";
  json += "\"residual_px\":" + String(result.residualPx, 1) + ",";
  json += "\"grid_score\":" + String(result.gridScore, 1) + ",";
  json += "\"blob_score\":" + String(result.blobScore, 1) + ",";
  json += "\"orientation\":\"auto_landscape\",";
  json += "\"source\":\"" + String(result.gridSearch ? "grid_search" : "blob_lattice") + "\",";
  json += "\"projective\":" + String(result.projective ? "true" : "false") + ",";
  json += "\"outside_ratio\":" + String(result.outsideRatio, 3) + ",";
  json += "\"pattern\":\"" + patternString(result) + "\",";
  json += "\"corners\":{";
  json += "\"tl\":"; appendPointJson(json, result.quad.tl); json += ",";
  json += "\"tr\":"; appendPointJson(json, result.quad.tr); json += ",";
  json += "\"bl\":"; appendPointJson(json, result.quad.bl); json += ",";
  json += "\"br\":"; appendPointJson(json, result.quad.br); json += "},";
  json += "\"grid\":[";
  for (int i = 0; i < (COLS + 1) * (ROWS + 1); i++) {
    if (i > 0) json += ",";
    appendPointJson(json, result.grid[i]);
  }
  json += "],";
  json += "\"points\":[";
  for (int i = 0; i < POINT_COUNT; i++) {
    if (i > 0) json += ",";
    const CellResult &cell = result.cells[i];
    json += "{";
    json += "\"index\":" + String(i) + ",";
    json += "\"row\":" + String(i / COLS) + ",";
    json += "\"col\":" + String(i % COLS) + ",";
    json += "\"color\":\"" + String(colorKey(cell.color)) + "\",";
    json += "\"label\":\"" + String(colorLabel(cell.color)) + "\",";
    json += "\"confidence\":" + String(cell.confidence) + ",";
    json += "\"coverage\":" + String(cell.coverage) + ",";
    json += "\"in_frame\":" + String(cell.inFrame ? "true" : "false") + ",";
    json += "\"blob_match\":" + String(cell.blobMatched ? "true" : "false") + ",";
    json += "\"center\":"; appendPointJson(json, cell.center); json += ",";
    json += "\"rgb\":{\"r\":" + String(cell.rgb.r) + ",\"g\":" + String(cell.rgb.g) + ",\"b\":" + String(cell.rgb.b) + "}";
    json += "}";
  }
  json += "],\"components_debug\":[";
  for (int i = 0; i < result.debugComponentCount; i++) {
    if (i > 0) json += ",";
    const DebugComponent &component = result.debugComponents[i];
    json += "{\"x\":" + String(component.x) + ",\"y\":" + String(component.y) +
            ",\"w\":" + String(component.w) + ",\"h\":" + String(component.h) +
            ",\"score\":" + String(component.score) + "}";
  }
  json += "]}";
  return json;
}

String statusJson() {
  String json;
  json.reserve(5200);
  json += "{";
  json += "\"mode\":\"auto_detection\",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"camera\":\"" + String(cameraReady ? "ready" : "not_ready") + "\",";
  json += "\"psram\":\"" + String(psramFound() ? "yes" : "no") + "\",";
  json += "\"free_heap\":" + String(ESP.getFreeHeap()) + ",";
  json += "\"captures\":" + String(captureCount) + ",";
  json += "\"warmup_frames\":" + String(warmupFrames) + ",";
  json += "\"warmup_discards\":" + String(warmupFrameDiscardCount) + ",";
  json += "\"stale_discards\":" + String(staleFrameDiscardCount) + ",";
  json += "\"settings_storage\":\"" + String(settingsStorageReady ? "ready" : "unavailable") + "\",";
  json += "\"settings_saves\":" + String(settingsSaveCount) + ",";
  json += "\"last_error\":\"" + String(esp_err_to_name(lastCameraError)) + "\",";
  json += "\"calibration\":[";
  for (int i = 0; i < COLOR_COUNT; i++) {
    if (i > 0) json += ",";
    json += "{\"color\":\"" + String(COLOR_KEYS[i]) + "\",\"r\":" + String(calibration[i].rgb.r) +
            ",\"g\":" + String(calibration[i].rgb.g) + ",\"b\":" + String(calibration[i].rgb.b) +
            ",\"user\":" + String(calibration[i].user ? "true" : "false") + "}";
  }
  json += "]";
  if (hasLastResult) {
    json += ",\"last_result\":";
    json += resultJson(lastResult);
  } else {
    json += ",\"last_result\":null";
  }
  json += "}";
  return json;
}

camera_config_t makeCameraConfig() {
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
  config.frame_size = FRAMESIZE_QQVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_DRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;
  return config;
}

bool ensureCamera() {
  if (cameraReady) return true;
  Serial.println("camera init: QQVGA RGB565 DRAM");
  esp_camera_deinit();
  delay(50);
  pinMode(PWDN_GPIO_NUM, OUTPUT);
  digitalWrite(PWDN_GPIO_NUM, LOW);
  delay(50);
  camera_config_t config = makeCameraConfig();
  lastCameraError = esp_camera_init(&config);
  if (lastCameraError != ESP_OK) {
    Serial.printf("camera init failed: 0x%08X (%s)\n", lastCameraError, esp_err_to_name(lastCameraError));
    return false;
  }
  cameraReady = true;
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor != nullptr) {
    if (sensor->set_whitebal) sensor->set_whitebal(sensor, 1);
    if (sensor->set_awb_gain) sensor->set_awb_gain(sensor, 1);
    if (sensor->set_exposure_ctrl) sensor->set_exposure_ctrl(sensor, 1);
    if (sensor->set_gain_ctrl) sensor->set_gain_ctrl(sensor, 1);
    if (sensor->set_dcw) sensor->set_dcw(sensor, 1);
  }
  Serial.println("camera ready: QQVGA RGB565");
  return true;
}

struct CaptureLock {
  explicit CaptureLock(bool &locked) : lockedRef(locked) { lockedRef = true; }
  ~CaptureLock() { lockedRef = false; }
  bool &lockedRef;
};

void sendPlainError(int status, const String &message) {
  Serial.printf("http error %d: %s\n", status, message.c_str());
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
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

camera_fb_t *captureFrame(String &error, int warmupCount) {
  if (!ensureCamera()) {
    error = "Camera init failed: ";
    error += esp_err_to_name(lastCameraError);
    return nullptr;
  }
  if (warmupCount > 0 && !discardFrames(error, warmupCount, "warm-up", warmupFrameDiscardCount)) {
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
  if (fb->format != PIXFORMAT_RGB565 || fb->len < FRAME_W * FRAME_H * 2) {
    esp_camera_fb_return(fb);
    error = "Frame failed: expected QQVGA RGB565.";
    return nullptr;
  }
  captureCount++;
  return fb;
}

camera_fb_t *captureFrame(String &error) {
  return captureFrame(error, 0);
}

void printResult(const DetectionResult &result) {
  Serial.printf("v2 result: status=%s complete=%s timeout=%s score=%.1f confidence=%u blobs=%u/%u residual=%.1f grid=%.1f elapsed=%ums pattern=%s\n",
                result.found ? "found" : "best_effort", result.complete ? "yes" : "no",
                result.timedOut ? "yes" : "no", result.score, result.confidence, result.matchedBlobs, result.blobCount,
                result.residualPx, result.gridScore, result.elapsedMs, patternString(result).c_str());
}

bool captureAndDetect(DetectionResult &result, camera_fb_t **fbOut, String &error) {
  if (captureInProgress) {
    error = "Capture already in progress.";
    return false;
  }
  CaptureLock lock(captureInProgress);
  const int requestedWarmupFrames = requestWarmupFrames();
  camera_fb_t *fb = captureFrame(error, requestedWarmupFrames);
  if (fb == nullptr) return false;
  result = detectMosaic(fb);
  lastResult = result;
  hasLastResult = true;
  printResult(result);
  *fbOut = fb;
  return true;
}

void handleRoot() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.send(200, "application/json", statusJson());
}

void handlePreview() {
  if (captureInProgress) {
    sendPlainError(503, "Capture already in progress.");
    return;
  }
  CaptureLock lock(captureInProgress);
  String error;
  camera_fb_t *fb = captureFrame(error);
  if (fb == nullptr) {
    sendPlainError(503, error);
    return;
  }
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("X-Frame-Width", String(FRAME_W));
  server.sendHeader("X-Frame-Height", String(FRAME_H));
  server.sendHeader("X-Capture-Count", String(captureCount));
  server.setContentLength(fb->len);
  server.send(200, "application/octet-stream", "");
  WiFiClient client = server.client();
  client.write(fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void handleFrame() {
  DetectionResult result;
  camera_fb_t *fb = nullptr;
  String error;
  if (!captureAndDetect(result, &fb, error)) {
    sendPlainError(503, error);
    return;
  }
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("X-Frame-Width", String(FRAME_W));
  server.sendHeader("X-Frame-Height", String(FRAME_H));
  server.sendHeader("X-Mosaic-Found", result.found ? "1" : "0");
  server.sendHeader("X-Mosaic-Confidence", String(result.confidence));
  server.sendHeader("X-Mosaic-Elapsed-Ms", String(result.elapsedMs));
  server.setContentLength(fb->len);
  server.send(200, "application/octet-stream", "");
  WiFiClient client = server.client();
  client.write(fb->buf, fb->len);
  esp_camera_fb_return(fb);
}

void handleResult() {
  DetectionResult result;
  camera_fb_t *fb = nullptr;
  String error;
  if (!captureAndDetect(result, &fb, error)) {
    sendPlainError(503, error);
    return;
  }
  esp_camera_fb_return(fb);
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.send(200, "application/json", resultJson(result));
}

void handleModel() {
  sendPlainError(410, "Manual model corners are disabled in mosaic_reader_v2 auto-detection mode.");
}

int colorIndexForKey(String key) {
  key.toLowerCase();
  for (int i = 0; i < COLOR_COUNT; i++) {
    if (key == COLOR_KEYS[i]) return i;
  }
  return -1;
}

void handleCalibrate() {
  const int cell = requestInt("cell", -1, -1, POINT_COUNT - 1);
  const int color = colorIndexForKey(server.arg("color"));
  if (cell < 0 || color < 0) {
    sendPlainError(400, "Use cell=0..11 and color=yellow|green|blue|white.");
    return;
  }

  DetectionResult result;
  camera_fb_t *fb = nullptr;
  String error;
  if (!captureAndDetect(result, &fb, error)) {
    sendPlainError(503, error);
    return;
  }
  calibration[color].rgb = result.cells[cell].rgb;
  calibration[color].user = true;
  saveSettings();
  esp_camera_fb_return(fb);
  Serial.printf("v2 calibrated %s from cell %d rgb=%u,%u,%u\n", COLOR_KEYS[color], cell + 1,
                calibration[color].rgb.r, calibration[color].rgb.g, calibration[color].rgb.b);
  server.send(200, "application/json", statusJson());
}

void handleReset() {
  resetSettings();
  server.send(200, "application/json", statusJson());
}

void handleNotFound() {
  sendPlainError(404, "Not found");
}

void connectWifi() {
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID_VALUE, WIFI_PASSWORD_VALUE);
  Serial.printf("connecting to Wi-Fi SSID: %s\n", WIFI_SSID_VALUE);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.printf("Wi-Fi connected: http://%s/\n", WiFi.localIP().toString().c_str());
  wifiWasConnected = true;
}

void maintainWifi() {
  const bool connected = WiFi.status() == WL_CONNECTED;
  if (connected) {
    if (!wifiWasConnected) {
      Serial.printf("Wi-Fi reconnected: http://%s/\n", WiFi.localIP().toString().c_str());
    }
    wifiWasConnected = true;
    return;
  }

  wifiWasConnected = false;
  const uint32_t now = millis();
  if (now - lastWifiReconnectAttempt < WIFI_RECONNECT_INTERVAL_MS) return;
  lastWifiReconnectAttempt = now;
  Serial.println("Wi-Fi disconnected; reconnecting");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID_VALUE, WIFI_PASSWORD_VALUE);
}

void startHttpServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/preview", HTTP_GET, handlePreview);
  server.on("/frame", HTTP_GET, handleFrame);
  server.on("/result", HTTP_GET, handleResult);
  server.on("/model", HTTP_POST, handleModel);
  server.on("/calibrate", HTTP_POST, handleCalibrate);
  server.on("/settings/reset", HTTP_POST, handleReset);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");
}

void printBootStatus() {
  Serial.println();
  Serial.println("ESP32-CAM mosaic reader v2 firmware");
  Serial.printf("psramFound: %s\n", psramFound() ? "yes" : "no");
  Serial.printf("free heap: %u bytes\n", static_cast<unsigned>(ESP.getFreeHeap()));
  Serial.printf("warm-up frames before detection: %d\n", warmupFrames);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1200);
  loadSettings();
  printBootStatus();
  connectWifi();
  startHttpServer();
  ensureCamera();
}

void loop() {
  maintainWifi();
  server.handleClient();
  delay(2);
}
