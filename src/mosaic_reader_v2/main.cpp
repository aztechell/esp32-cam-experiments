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
  RgbColor rgb;
  PointF center;
};

struct DetectionResult {
  bool found = false;
  float score = 0;
  uint8_t confidence = 0;
  Quad quad;
  CellResult cells[POINT_COUNT];
};

WebServer server(80);
Preferences preferences;

PointNorm modelCorners[4];
CalibrationSample calibration[COLOR_COUNT];
DetectionResult lastResult;
bool hasLastResult = false;
bool cameraReady = false;
bool captureInProgress = false;
bool settingsStorageReady = false;
esp_err_t lastCameraError = ESP_OK;
uint32_t captureCount = 0;
uint32_t settingsSaveCount = 0;
uint8_t luma[FRAME_W * FRAME_H];
uint8_t chroma[FRAME_W * FRAME_H];

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
    .label { fill: #020617; font-size: 7px; font-weight: 800; text-anchor: middle; dominant-baseline: central; pointer-events: none; }
    .panel { border: 1px solid #334155; border-radius: 8px; background: #111827; padding: 12px; display: grid; gap: 12px; }
    .row { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
    label { display: grid; gap: 5px; color: #cbd5e1; font-size: 12px; }
    select, input { min-width: 0; width: 100%; height: 38px; border: 1px solid #475569; border-radius: 6px; background: #1f2937; color: #f8fafc; padding: 0 10px; }
    button { height: 40px; border: 1px solid #2563eb; border-radius: 6px; background: #2563eb; color: #fff; padding: 0 13px; cursor: pointer; }
    button.secondary { background: #0f766e; border-color: #0f766e; }
    button.danger { background: #991b1b; border-color: #b91c1c; }
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
        <button id="save" class="secondary" type="button">Save model</button>
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
        <div class="hint">Drag yellow corners to the approximate mosaic area. Runtime still performs a fresh single-shot search on every result request.</div>
        <div class="row">
          <label>Calibrate cell<select id="cell"></select></label>
          <label>Color<select id="color"><option value="yellow">Yellow</option><option value="green">Green</option><option value="blue">Blue</option><option value="white">White</option></select></label>
        </div>
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
    const colors = { yellow: '#facc15', green: '#22c55e', blue: '#3b82f6', white: '#f8fafc' };
    const labels = { yellow: 'Y', green: 'G', blue: 'B', white: 'W' };
    let model = [];
    let result = null;
    let drag = -1;
    for (let i = 0; i < 12; i++) {
      const option = document.createElement('option');
      option.value = String(i);
      option.textContent = `${i + 1} / r${Math.floor(i / 4) + 1}c${i % 4 + 1}`;
      cellSelect.appendChild(option);
    }
    function showError(text) { errorBox.hidden = !text; errorBox.textContent = text || ''; }
    function pxCorner(c) { return { x: c.x * 159 / 10000, y: c.y * 119 / 10000 }; }
    function normFromEvent(event) {
      const rect = overlay.getBoundingClientRect();
      return {
        x: Math.max(0, Math.min(10000, Math.round((event.clientX - rect.left) * 10000 / rect.width))),
        y: Math.max(0, Math.min(10000, Math.round((event.clientY - rect.top) * 10000 / rect.height)))
      };
    }
    function svg(name, attrs) {
      const element = document.createElementNS('http://www.w3.org/2000/svg', name);
      Object.entries(attrs).forEach(([k, v]) => element.setAttribute(k, String(v)));
      return element;
    }
    function addLine(a, b, cls) {
      overlay.appendChild(svg('line', { class: cls, x1: a.x, y1: a.y, x2: b.x, y2: b.y }));
    }
    function drawOverlay() {
      overlay.replaceChildren();
      if (model.length === 4) {
        const m = model.map(pxCorner);
        addLine(m[0], m[1], 'model-line'); addLine(m[1], m[3], 'model-line');
        addLine(m[3], m[2], 'model-line'); addLine(m[2], m[0], 'model-line');
        m.forEach((p, i) => {
          const g = svg('g', { class: 'corner' });
          g.dataset.index = String(i);
          g.appendChild(svg('circle', { class: 'corner-dot', cx: p.x, cy: p.y, r: 4 }));
          const t = svg('text', { class: 'label', x: p.x, y: p.y + .2 });
          t.textContent = ['TL','TR','BL','BR'][i];
          g.appendChild(t);
          overlay.appendChild(g);
        });
      }
      if (!result) return;
      const q = result.corners;
      const rows = [0, 1/3, 2/3, 1];
      const cols = [0, .25, .5, .75, 1];
      function map(u, v) {
        const top = { x: q.tl.x + (q.tr.x - q.tl.x) * u, y: q.tl.y + (q.tr.y - q.tl.y) * u };
        const bot = { x: q.bl.x + (q.br.x - q.bl.x) * u, y: q.bl.y + (q.br.y - q.bl.y) * u };
        return { x: top.x + (bot.x - top.x) * v, y: top.y + (bot.y - top.y) * v };
      }
      rows.forEach(v => addLine(map(0, v), map(1, v), 'det-line'));
      cols.forEach(u => addLine(map(u, 0), map(u, 1), 'det-line'));
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
    function renderTable() {
      resultTable.innerHTML = '';
      for (let i = 0; i < 12; i++) {
        const p = result && result.points ? result.points[i] : null;
        const color = p ? p.color : 'green';
        const cell = document.createElement('div');
        cell.className = 'cell';
        cell.innerHTML = `<div class="symbol"><span>${labels[color] || '?'}</span><span class="swatch" style="background:${colors[color] || '#64748b'}"></span></div><div class="meta">#${i + 1}<br>conf ${p ? p.confidence : 0}%<br>rgb ${p ? `${p.rgb.r},${p.rgb.g},${p.rgb.b}` : '-'}</div>`;
        resultTable.appendChild(cell);
      }
    }
    async function loadStatus() {
      const response = await fetch('/status', { cache: 'no-store' });
      const data = await response.json();
      model = data.model_corners || model;
      result = data.last_result || result;
      statusBox.textContent = `IP ${data.ip} | camera ${data.camera} | captures ${data.captures}`;
      if (result) diagnostics.textContent = `status ${result.status} | score ${result.score} | confidence ${result.confidence}% | pattern ${result.pattern}`;
      renderTable(); drawOverlay();
    }
    async function analyze() {
      showError('');
      try {
        const response = await fetch(`/frame?t=${Date.now()}`, { cache: 'no-store' });
        if (!response.ok) throw new Error(await response.text());
        const header = response.headers.get('X-Mosaic-Result');
        const buffer = await response.arrayBuffer();
        if (header) result = JSON.parse(header);
        renderRgb565(buffer);
        await loadStatus();
      } catch (error) { showError(error.message || String(error)); }
    }
    async function saveModel() {
      const params = new URLSearchParams();
      model.forEach((p, i) => { params.set(`x${i}`, p.x); params.set(`y${i}`, p.y); });
      const response = await fetch(`/model?${params}`, { method: 'POST', cache: 'no-store' });
      if (!response.ok) throw new Error(await response.text());
      await loadStatus();
    }
    overlay.addEventListener('pointerdown', event => {
      if (model.length !== 4) return;
      const p = normFromEvent(event);
      let best = -1, bestDist = Infinity;
      model.forEach((corner, i) => {
        const dx = p.x - corner.x, dy = p.y - corner.y, d = dx * dx + dy * dy;
        if (d < bestDist) { bestDist = d; best = i; }
      });
      if (best >= 0 && bestDist < 900 * 900) { drag = best; overlay.setPointerCapture(event.pointerId); }
    });
    overlay.addEventListener('pointermove', event => {
      if (drag < 0) return;
      model[drag] = normFromEvent(event);
      drawOverlay();
    });
    overlay.addEventListener('pointerup', async event => {
      if (drag < 0) return;
      drag = -1;
      overlay.releasePointerCapture(event.pointerId);
      try { await saveModel(); } catch (error) { showError(error.message || String(error)); }
    });
    document.getElementById('analyze').addEventListener('click', analyze);
    document.getElementById('save').addEventListener('click', () => saveModel().catch(e => showError(e.message || String(e))));
    document.getElementById('reset').addEventListener('click', async () => {
      if (!confirm('Reset v2 model and calibration?')) return;
      const response = await fetch('/settings/reset', { method: 'POST', cache: 'no-store' });
      if (!response.ok) { showError(await response.text()); return; }
      await loadStatus(); await analyze();
    });
    document.getElementById('calibrate').addEventListener('click', async () => {
      showError('');
      const params = new URLSearchParams({ cell: cellSelect.value, color: colorSelect.value });
      const response = await fetch(`/calibrate?${params}`, { method: 'POST', cache: 'no-store' });
      if (!response.ok) { showError(await response.text()); return; }
      await loadStatus(); await analyze();
    });
    loadStatus().then(analyze).catch(e => showError(e.message || String(e)));
  </script>
</body>
</html>
)HTML";

int clampValue(int value, int minValue, int maxValue) {
  if (value < minValue) return minValue;
  if (value > maxValue) return maxValue;
  return value;
}

int requestInt(const char *name, int fallback, int minValue, int maxValue) {
  if (!server.hasArg(name)) return fallback;
  return clampValue(server.arg(name).toInt(), minValue, maxValue);
}

const char *colorKey(MosaicColor color) {
  return COLOR_KEYS[static_cast<int>(color)];
}

const char *colorLabel(MosaicColor color) {
  return COLOR_LABELS[static_cast<int>(color)];
}

void applyDefaultModel() {
  modelCorners[0] = PointNorm(1800, 1800);
  modelCorners[1] = PointNorm(8200, 1800);
  modelCorners[2] = PointNorm(1800, 8200);
  modelCorners[3] = PointNorm(8200, 8200);
}

void applyDefaultCalibration() {
  calibration[COLOR_YELLOW] = CalibrationSample(RgbColor(235, 220, 30), false);
  calibration[COLOR_GREEN] = CalibrationSample(RgbColor(55, 155, 95), false);
  calibration[COLOR_BLUE] = CalibrationSample(RgbColor(40, 105, 220), false);
  calibration[COLOR_WHITE] = CalibrationSample(RgbColor(225, 225, 220), false);
}

PointF normToPixel(const PointNorm &point) {
  return {point.x * (FRAME_W - 1) / 10000.0f, point.y * (FRAME_H - 1) / 10000.0f};
}

Quad modelQuadPixels() {
  return {normToPixel(modelCorners[0]), normToPixel(modelCorners[1]),
          normToPixel(modelCorners[2]), normToPixel(modelCorners[3])};
}

void saveSettings() {
  if (!settingsStorageReady) return;
  preferences.putUInt("version", SETTINGS_VERSION);
  for (int i = 0; i < 4; i++) {
    preferences.putInt(("mx" + String(i)).c_str(), modelCorners[i].x);
    preferences.putInt(("my" + String(i)).c_str(), modelCorners[i].y);
  }
  for (int i = 0; i < COLOR_COUNT; i++) {
    preferences.putInt(("cr" + String(i)).c_str(), calibration[i].rgb.r);
    preferences.putInt(("cg" + String(i)).c_str(), calibration[i].rgb.g);
    preferences.putInt(("cb" + String(i)).c_str(), calibration[i].rgb.b);
    preferences.putBool(("cu" + String(i)).c_str(), calibration[i].user);
  }
  settingsSaveCount++;
}

void loadSettings() {
  applyDefaultModel();
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
  for (int i = 0; i < 4; i++) {
    modelCorners[i].x = static_cast<uint16_t>(clampValue(preferences.getInt(("mx" + String(i)).c_str(), modelCorners[i].x), 0, 10000));
    modelCorners[i].y = static_cast<uint16_t>(clampValue(preferences.getInt(("my" + String(i)).c_str(), modelCorners[i].y), 0, 10000));
  }
  for (int i = 0; i < COLOR_COUNT; i++) {
    calibration[i].rgb.r = static_cast<uint8_t>(clampValue(preferences.getInt(("cr" + String(i)).c_str(), calibration[i].rgb.r), 0, 255));
    calibration[i].rgb.g = static_cast<uint8_t>(clampValue(preferences.getInt(("cg" + String(i)).c_str(), calibration[i].rgb.g), 0, 255));
    calibration[i].rgb.b = static_cast<uint8_t>(clampValue(preferences.getInt(("cb" + String(i)).c_str(), calibration[i].rgb.b), 0, 255));
    calibration[i].user = preferences.getBool(("cu" + String(i)).c_str(), calibration[i].user);
  }
}

void resetSettings() {
  if (settingsStorageReady) preferences.clear();
  applyDefaultModel();
  applyDefaultCalibration();
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
  for (int i = 0; i < FRAME_W * FRAME_H; i++) {
    const RgbColor rgb = decodeRgb565(fb->buf + i * 2);
    luma[i] = pixelLuma(rgb);
    chroma[i] = pixelChroma(rgb);
  }
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

Quad transformQuad(const Quad &base, float dx, float dy, float scale, float angleDeg) {
  const float angle = angleDeg * 3.1415926f / 180.0f;
  const float ca = cosf(angle);
  const float sa = sinf(angle);
  const PointF center = {(base.tl.x + base.tr.x + base.bl.x + base.br.x) * 0.25f,
                         (base.tl.y + base.tr.y + base.bl.y + base.br.y) * 0.25f};
  auto transformPoint = [&](const PointF &p) {
    const float x = (p.x - center.x) * scale;
    const float y = (p.y - center.y) * scale;
    return PointF{center.x + x * ca - y * sa + dx, center.y + x * sa + y * ca + dy};
  };
  return {transformPoint(base.tl), transformPoint(base.tr),
          transformPoint(base.bl), transformPoint(base.br)};
}

float darkScoreAt(const PointF &p) {
  if (!inFrame(p)) return -0.35f;
  const int x = clampValue(static_cast<int>(roundf(p.x)), 0, FRAME_W - 1);
  const int y = clampValue(static_cast<int>(roundf(p.y)), 0, FRAME_H - 1);
  const uint8_t yy = luma[y * FRAME_W + x];
  if (yy < 55) return 1.0f;
  if (yy < 90) return (90.0f - yy) / 35.0f;
  return -0.2f;
}

float centerScoreAt(const PointF &p) {
  if (!inFrame(p)) return -0.35f;
  const int x = clampValue(static_cast<int>(roundf(p.x)), 0, FRAME_W - 1);
  const int y = clampValue(static_cast<int>(roundf(p.y)), 0, FRAME_H - 1);
  const int offset = y * FRAME_W + x;
  const uint8_t yy = luma[offset];
  const uint8_t cc = chroma[offset];
  if (yy < 35) return -0.3f;
  const float colorPart = min(1.0f, cc / 55.0f);
  const float whitePart = yy > 145 ? min(1.0f, (yy - 120) / 80.0f) : 0.0f;
  return max(colorPart, whitePart);
}

float scoreQuad(const Quad &quad) {
  float lineTotal = 0;
  int lineCount = 0;
  const float cols[5] = {0, 0.25f, 0.5f, 0.75f, 1};
  const float rows[4] = {0, 1.0f / 3.0f, 2.0f / 3.0f, 1};
  for (float u : cols) {
    for (int i = 0; i <= 12; i++) {
      lineTotal += darkScoreAt(mapQuad(quad, u, i / 12.0f));
      lineCount++;
    }
  }
  for (float v : rows) {
    for (int i = 0; i <= 16; i++) {
      lineTotal += darkScoreAt(mapQuad(quad, i / 16.0f, v));
      lineCount++;
    }
  }

  float centerTotal = 0;
  int centerCount = 0;
  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < COLS; col++) {
      centerTotal += centerScoreAt(mapQuad(quad, (col + 0.5f) / COLS,
                                           (row + 0.5f) / ROWS));
      centerCount++;
    }
  }

  const float lineScore = lineTotal / max(1, lineCount);
  const float centerScore = centerTotal / max(1, centerCount);
  return 100.0f * (0.68f * lineScore + 0.32f * centerScore);
}

RgbColor sampleCell(const camera_fb_t *fb, const PointF &center) {
  uint32_t sumR = 0, sumG = 0, sumB = 0, count = 0;
  uint32_t fallbackR = 0, fallbackG = 0, fallbackB = 0, fallbackCount = 0;
  const int cx = static_cast<int>(roundf(center.x));
  const int cy = static_cast<int>(roundf(center.y));
  for (int y = cy - SAMPLE_RADIUS; y <= cy + SAMPLE_RADIUS; y++) {
    if (y < 0 || y >= FRAME_H) continue;
    for (int x = cx - SAMPLE_RADIUS; x <= cx + SAMPLE_RADIUS; x++) {
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
  confidence = static_cast<uint8_t>((absolute + margin) / 2);
  return static_cast<MosaicColor>(bestIndex);
}

void fillResultCells(const camera_fb_t *fb, DetectionResult &result) {
  for (int row = 0; row < ROWS; row++) {
    for (int col = 0; col < COLS; col++) {
      const int index = row * COLS + col;
      CellResult &cell = result.cells[index];
      cell.center = mapQuad(result.quad, (col + 0.5f) / COLS, (row + 0.5f) / ROWS);
      cell.rgb = sampleCell(fb, cell.center);
      cell.color = classifyColor(cell.rgb, cell.confidence);
    }
  }
}

DetectionResult detectMosaic(const camera_fb_t *fb) {
  buildFeatureImages(fb);
  const Quad base = modelQuadPixels();
  DetectionResult best;
  best.quad = base;
  best.score = -100000.0f;

  const float dxValues[] = {-60, -48, -36, -24, -12, 0, 12, 24, 36, 48, 60};
  const float dyValues[] = {-45, -36, -27, -18, -9, 0, 9, 18, 27, 36, 45};
  const float scales[] = {0.72f, 0.86f, 1.0f, 1.14f, 1.28f};
  const float rotations[] = {-24, -12, 0, 12, 24};

  float bestDx = 0, bestDy = 0, bestScale = 1, bestRot = 0;
  for (float dx : dxValues) {
    for (float dy : dyValues) {
      for (float scale : scales) {
        for (float rot : rotations) {
          const Quad candidate = transformQuad(base, dx, dy, scale, rot);
          const float score = scoreQuad(candidate);
          if (score > best.score) {
            best.score = score;
            best.quad = candidate;
            bestDx = dx;
            bestDy = dy;
            bestScale = scale;
            bestRot = rot;
          }
        }
      }
    }
  }

  for (float dx = bestDx - 10; dx <= bestDx + 10; dx += 2.5f) {
    for (float dy = bestDy - 10; dy <= bestDy + 10; dy += 2.5f) {
      for (float scale = bestScale - 0.08f; scale <= bestScale + 0.08f; scale += 0.04f) {
        for (float rot = bestRot - 6; rot <= bestRot + 6; rot += 3.0f) {
          const Quad candidate = transformQuad(base, dx, dy, scale, rot);
          const float score = scoreQuad(candidate);
          if (score > best.score) {
            best.score = score;
            best.quad = candidate;
          }
        }
      }
    }
  }

  best.found = best.score >= 42.0f;
  best.confidence = static_cast<uint8_t>(clampValue(static_cast<int>(best.score), 0, 100));
  fillResultCells(fb, best);
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
  json.reserve(2400);
  json += "{";
  json += "\"status\":\"" + String(result.found ? "found" : "best_effort") + "\",";
  json += "\"found\":" + String(result.found ? "true" : "false") + ",";
  json += "\"score\":" + String(result.score, 1) + ",";
  json += "\"confidence\":" + String(result.confidence) + ",";
  json += "\"pattern\":\"" + patternString(result) + "\",";
  json += "\"corners\":{";
  json += "\"tl\":"; appendPointJson(json, result.quad.tl); json += ",";
  json += "\"tr\":"; appendPointJson(json, result.quad.tr); json += ",";
  json += "\"bl\":"; appendPointJson(json, result.quad.bl); json += ",";
  json += "\"br\":"; appendPointJson(json, result.quad.br); json += "},";
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
    json += "\"center\":"; appendPointJson(json, cell.center); json += ",";
    json += "\"rgb\":{\"r\":" + String(cell.rgb.r) + ",\"g\":" + String(cell.rgb.g) + ",\"b\":" + String(cell.rgb.b) + "}";
    json += "}";
  }
  json += "]}";
  return json;
}

void appendModelJson(String &json) {
  json += "\"model_corners\":[";
  for (int i = 0; i < 4; i++) {
    if (i > 0) json += ",";
    json += "{\"x\":" + String(modelCorners[i].x) + ",\"y\":" + String(modelCorners[i].y) + "}";
  }
  json += "]";
}

String statusJson() {
  String json;
  json.reserve(3200);
  json += "{";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"camera\":\"" + String(cameraReady ? "ready" : "not_ready") + "\",";
  json += "\"psram\":\"" + String(psramFound() ? "yes" : "no") + "\",";
  json += "\"free_heap\":" + String(ESP.getFreeHeap()) + ",";
  json += "\"captures\":" + String(captureCount) + ",";
  json += "\"settings_storage\":\"" + String(settingsStorageReady ? "ready" : "unavailable") + "\",";
  json += "\"settings_saves\":" + String(settingsSaveCount) + ",";
  json += "\"last_error\":\"" + String(esp_err_to_name(lastCameraError)) + "\",";
  appendModelJson(json);
  json += ",\"calibration\":[";
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
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
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
  server.send(status, "text/plain", message);
}

camera_fb_t *captureFrame(String &error) {
  if (!ensureCamera()) {
    error = "Camera init failed: ";
    error += esp_err_to_name(lastCameraError);
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

void printResult(const DetectionResult &result) {
  Serial.printf("v2 result: status=%s score=%.1f confidence=%u pattern=%s\n",
                result.found ? "found" : "best_effort", result.score,
                result.confidence, patternString(result).c_str());
}

bool captureAndDetect(DetectionResult &result, camera_fb_t **fbOut, String &error) {
  if (captureInProgress) {
    error = "Capture already in progress.";
    return false;
  }
  CaptureLock lock(captureInProgress);
  camera_fb_t *fb = captureFrame(error);
  if (fb == nullptr) return false;
  result = detectMosaic(fb);
  lastResult = result;
  hasLastResult = true;
  printResult(result);
  *fbOut = fb;
  return true;
}

void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", INDEX_HTML);
}

void handleStatus() {
  server.send(200, "application/json", statusJson());
}

void handleFrame() {
  DetectionResult result;
  camera_fb_t *fb = nullptr;
  String error;
  if (!captureAndDetect(result, &fb, error)) {
    sendPlainError(503, error);
    return;
  }
  const String json = resultJson(result);
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("X-Frame-Width", String(FRAME_W));
  server.sendHeader("X-Frame-Height", String(FRAME_H));
  server.sendHeader("X-Mosaic-Result", json);
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
  server.send(200, "application/json", resultJson(result));
}

void handleModel() {
  bool changed = false;
  for (int i = 0; i < 4; i++) {
    const String xKey = "x" + String(i);
    const String yKey = "y" + String(i);
    if (server.hasArg(xKey) && server.hasArg(yKey)) {
      const uint16_t x = static_cast<uint16_t>(requestInt(xKey.c_str(), modelCorners[i].x, 0, 10000));
      const uint16_t y = static_cast<uint16_t>(requestInt(yKey.c_str(), modelCorners[i].y, 0, 10000));
      if (modelCorners[i].x != x || modelCorners[i].y != y) {
        modelCorners[i].x = x;
        modelCorners[i].y = y;
        changed = true;
      }
    }
  }
  if (changed) saveSettings();
  server.send(200, "application/json", statusJson());
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
  server.handleClient();
  delay(2);
}
