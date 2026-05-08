#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
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
constexpr char SETTINGS_NAMESPACE[] = "cam_live";
constexpr uint32_t SETTINGS_VERSION = 1;
constexpr int DEFAULT_RESOLUTION_INDEX = 1;
constexpr int DEFAULT_FPS = 2;

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
  bool unstableWithoutPsram;
};

struct CameraSettings {
  int jpegQuality = 14;
  int brightness = 0;
  int contrast = 0;
  int saturation = 0;
  int sharpness = 0;
  int whiteBalance = 1;
  int awbGain = 1;
  int wbMode = 0;
  int exposureCtrl = 1;
  int aec2 = 0;
  int aeLevel = 0;
  int aecValue = 300;
  int gainCtrl = 1;
  int agcGain = 0;
  int gainCeiling = 0;
  int hmirror = 0;
  int vflip = 0;
  int lenc = 1;
  int rawGma = 1;
  int bpc = 0;
  int wpc = 1;
  int dcw = 1;
  int discardFrames = 0;
  int settleMs = 0;
};

const ResolutionOption RESOLUTIONS[] = {
    {"qqvga", "QQVGA 160x120", FRAMESIZE_QQVGA, false},
    {"qvga", "QVGA 320x240", FRAMESIZE_QVGA, false},
    {"vga", "VGA 640x480 (unstable without PSRAM)", FRAMESIZE_VGA, true},
};

WebServer server(80);
Preferences preferences;

CameraSettings currentSettings;
int currentResolutionIndex = DEFAULT_RESOLUTION_INDEX;
bool cameraReady = false;
bool frameInProgress = false;
bool settingsStorageReady = false;
esp_err_t lastCameraError = ESP_OK;
const ResolutionOption *activeResolution = nullptr;
String lastFrameRequestId;
unsigned long lastFrameRequestAt = 0;
uint32_t frameCount = 0;
uint32_t frameRequestCount = 0;
uint32_t lastFrameBytes = 0;
uint32_t settingsApplyCount = 0;
uint32_t settingsSaveCount = 0;
int lastRequestedFps = DEFAULT_FPS;

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="ru">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESP32-CAM Live</title>
  <style>
    :root { color-scheme: dark; font-family: system-ui, -apple-system, Segoe UI, sans-serif; }
    * { box-sizing: border-box; }
    body { margin: 0; background: #0f172a; color: #f8fafc; overflow: hidden; }
    button, input, select { font: inherit; }
    .app { height: 100vh; display: flex; flex-direction: column; gap: 12px; padding: 16px; }
    header { display: flex; align-items: center; justify-content: space-between; gap: 12px; }
    h1 { margin: 0; font-size: 24px; line-height: 1.1; }
    .top-controls { display: flex; align-items: center; gap: 10px; }
    .workspace { min-height: 0; flex: 1; display: grid; grid-template-columns: minmax(0, 1fr) 460px; gap: 14px; }
    .viewer { min-width: 0; min-height: 0; display: flex; flex-direction: column; gap: 10px; }
    .stream-shell { min-height: 0; flex: 1; border: 1px solid #334155; border-radius: 8px; background: #020617; overflow: hidden; display: grid; place-items: center; }
    #stream { display: block; width: 100%; height: 100%; object-fit: contain; }
    .empty { color: #94a3b8; font-size: 14px; }
    .status-row { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 10px; }
    .status, .error, .help { border-radius: 6px; padding: 9px 10px; font-size: 13px; line-height: 1.35; }
    .status { color: #dbeafe; background: #172554; border: 1px solid #1d4ed8; }
    .error { color: #fecaca; background: #450a0a; border: 1px solid #991b1b; min-height: 39px; }
    .help { color: #dbeafe; background: #172554; border: 1px solid #1d4ed8; min-height: 60px; }
    .panel { min-height: 0; overflow: auto; border: 1px solid #334155; border-radius: 8px; background: #111827; padding: 12px; }
    .panel h2 { margin: 0 0 10px; font-size: 16px; }
    .settings { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); gap: 11px 12px; align-items: end; }
    .wide { grid-column: 1 / -1; }
    label { display: grid; gap: 5px; min-width: 0; color: #cbd5e1; font-size: 12px; }
    label.inline { display: flex; align-items: center; gap: 7px; min-height: 34px; cursor: help; }
    .setting { cursor: help; }
    .setting-name { display: flex; flex-wrap: wrap; gap: 4px; align-items: baseline; }
    .setting-name strong { color: #f8fafc; font-size: 13px; font-weight: 700; }
    .setting-name small { color: #94a3b8; font-size: 11px; }
    select, input { min-width: 0; border-radius: 6px; border: 1px solid #475569; background: #1f2937; color: #f8fafc; }
    select { width: 100%; height: 38px; padding: 0 10px; }
    input[type="number"] { width: 100%; height: 38px; padding: 0 9px; }
    input[type="range"] { width: 100%; accent-color: #38bdf8; }
    input[type="checkbox"] { width: 18px; height: 18px; accent-color: #38bdf8; flex: 0 0 auto; }
    button { height: 40px; border-radius: 6px; border: 1px solid #2563eb; padding: 0 14px; background: #2563eb; color: white; cursor: pointer; }
    button.paused { background: #0f766e; border-color: #0f766e; }
    button.danger { background: #991b1b; border-color: #b91c1c; }
    .value { color: #f8fafc; font-size: 13px; }
    @media (max-width: 980px) {
      body { overflow: auto; }
      .app { min-height: 100vh; height: auto; }
      header { align-items: flex-start; flex-direction: column; }
      .top-controls { width: 100%; display: grid; grid-template-columns: 1fr 1fr; }
      .workspace { grid-template-columns: 1fr; }
      .stream-shell { height: 58vh; }
      .status-row { grid-template-columns: 1fr; }
      .panel { overflow: visible; }
    }
    @media (max-width: 560px) {
      .app { padding: 10px; }
      .top-controls, .settings { grid-template-columns: 1fr; }
      button { width: 100%; }
    }
  </style>
</head>
<body>
  <main class="app">
    <header>
      <h1>ESP32-CAM Live</h1>
      <div class="top-controls">
        <button id="streamToggle" type="button">Pause</button>
        <button id="resetSettings" class="danger" type="button">Reset settings</button>
        <label class="setting" data-help="Целевая частота кадров. Если плата не успевает, фактический FPS будет ниже, но запросы не будут копиться.">
          <span class="setting-name"><strong>FPS</strong><small>частота стрима</small></span>
          <select id="fps">
            <option value="1">1 fps</option>
            <option value="2" selected>2 fps</option>
            <option value="5">5 fps</option>
            <option value="8">8 fps</option>
            <option value="10">10 fps</option>
          </select>
        </label>
      </div>
    </header>

    <div class="workspace">
      <section class="viewer">
        <div class="stream-shell">
          <img id="stream" alt="ESP32-CAM live stream">
          <div id="empty" class="empty">stream starting...</div>
        </div>
        <div class="status-row">
          <div id="status" class="status">status loading...</div>
          <div id="error" class="error"></div>
        </div>
        <div id="liveStats" class="status">target 2 fps | actual 0.0 fps | frames 0 | last 0 bytes</div>
      </section>

      <aside class="panel">
        <h2>Camera settings</h2>
        <div class="settings">
          <label class="wide setting" data-help="Разрешение кадра. Без PSRAM практичные режимы обычно QQVGA и QVGA; VGA может вернуть ошибку памяти.">
            <span class="setting-name"><strong>Resolution</strong><small>размер кадра</small></span>
            <select id="resolution">
              <option value="qqvga">QQVGA 160x120</option>
              <option value="qvga" selected>QVGA 320x240</option>
              <option value="vga">VGA 640x480 (unstable without PSRAM)</option>
            </select>
          </label>

          <label class="setting" data-help="Качество JPEG управляет сжатием. Меньшее число дает файл больше и чище; большее число сильнее сжимает и портит детали."><span class="setting-name"><strong>JPEG quality</strong><small>сжатие</small></span><span class="value" id="qualityValue">14</span><input id="quality" type="range" min="10" max="40" value="14"></label>
          <label class="setting" data-help="Brightness сдвигает всю картинку темнее или светлее после обработки сенсором."><span class="setting-name"><strong>Brightness</strong><small>яркость</small></span><span class="value" id="brightnessValue">0</span><input id="brightness" type="range" min="-2" max="2" value="0"></label>
          <label class="setting" data-help="Contrast меняет разницу между темными и светлыми зонами. Слишком высокий контраст может съесть детали."><span class="setting-name"><strong>Contrast</strong><small>контраст</small></span><span class="value" id="contrastValue">0</span><input id="contrast" type="range" min="-2" max="2" value="0"></label>
          <label class="setting" data-help="Saturation меняет силу цветов. Минус делает цвета бледнее, плюс делает насыщеннее."><span class="setting-name"><strong>Saturation</strong><small>цветность</small></span><span class="value" id="saturationValue">0</span><input id="saturation" type="range" min="-2" max="2" value="0"></label>
          <label class="setting" data-help="Sharpness усиливает края. Может казаться четче, но также добавляет шум и ореолы."><span class="setting-name"><strong>Sharpness</strong><small>резкость</small></span><span class="value" id="sharpnessValue">0</span><input id="sharpness" type="range" min="-2" max="2" value="0"></label>
          <label class="setting" data-help="WB mode выбирает пресет баланса белого. Auto дает камере решать самой; фиксированные режимы делают повторные кадры стабильнее."><span class="setting-name"><strong>WB mode</strong><small>баланс белого</small></span>
            <select id="wbMode">
              <option value="0">Auto</option>
              <option value="1">Sunny</option>
              <option value="2">Cloudy</option>
              <option value="3">Office</option>
              <option value="4">Home</option>
            </select>
          </label>
          <label class="setting" data-help="AE level смещает автоэкспозицию светлее или темнее, когда включен AEC."><span class="setting-name"><strong>AE level</strong><small>смещение</small></span><span class="value" id="aeLevelValue">0</span><input id="aeLevel" type="range" min="-2" max="2" value="0"></label>
          <label class="setting" data-help="AEC value — ручная экспозиция, используется когда AEC выключен."><span class="setting-name"><strong>AEC value</strong><small>ручная экспозиция</small></span><input id="aecValue" type="number" min="0" max="1200" value="300"></label>
          <label class="setting" data-help="AGC gain — ручное усиление, используется когда AGC выключен. Больше усиление — светлее, но шумнее."><span class="setting-name"><strong>AGC gain</strong><small>ручное усиление</small></span><input id="agcGain" type="number" min="0" max="30" value="0"></label>
          <label class="setting" data-help="Gain ceiling ограничивает максимальное автоусиление. Выше потолок помогает в темноте, но добавляет шум."><span class="setting-name"><strong>Gain ceiling</strong><small>потолок</small></span>
            <select id="gainCeiling">
              <option value="0">2x</option>
              <option value="1">4x</option>
              <option value="2">8x</option>
              <option value="3">16x</option>
              <option value="4">32x</option>
              <option value="5">64x</option>
              <option value="6">128x</option>
            </select>
          </label>
          <label class="setting" data-help="Discard выбрасывает указанное число прогревочных кадров перед JPEG. Ставь 0, если нужен ровно один захват сенсора."><span class="setting-name"><strong>Discard</strong><small>выкинуть кадры</small></span><input id="discardFrames" type="number" min="0" max="2" value="0"></label>
          <label class="setting" data-help="Settle ms — задержка после применения настроек перед кадром. Полезно, если автоматике нужно время стабилизироваться."><span class="setting-name"><strong>Settle ms</strong><small>задержка</small></span><input id="settleMs" type="number" min="0" max="2000" value="0"></label>

          <label class="inline setting" data-help="AWB — Auto White Balance, автоматический баланс белого. Камера сама правит цвета; между кадрами цвет может поменяться."><input id="whiteBalance" type="checkbox" checked><span class="setting-name"><strong>AWB</strong><small>авто баланс белого</small></span></label>
          <label class="inline setting" data-help="AWB gain — усиление каналов для авто-баланса белого. Обычно имеет смысл вместе с AWB."><input id="awbGain" type="checkbox" checked><span class="setting-name"><strong>AWB gain</strong><small>усиление цвета</small></span></label>
          <label class="inline setting" data-help="AEC — Auto Exposure Control, автоэкспозиция. Камера сама меняет экспозицию, поэтому кадры могут отличаться по яркости."><input id="exposureCtrl" type="checkbox" checked><span class="setting-name"><strong>AEC</strong><small>автоэкспозиция</small></span></label>
          <label class="inline setting" data-help="AEC2 — альтернативный алгоритм автоэкспозиции. Иногда помогает в сложном свете, но может сделать яркость менее стабильной."><input id="aec2" type="checkbox"><span class="setting-name"><strong>AEC2</strong><small>альт. экспозиция</small></span></label>
          <label class="inline setting" data-help="AGC — Auto Gain Control, автоусиление. Делает темную картинку светлее, но добавляет шум и может меняться между кадрами."><input id="gainCtrl" type="checkbox" checked><span class="setting-name"><strong>AGC</strong><small>автоусиление</small></span></label>
          <label class="inline setting" data-help="Mirror отражает картинку по горизонтали, как зеркало."><input id="hmirror" type="checkbox"><span class="setting-name"><strong>Mirror</strong><small>зеркально</small></span></label>
          <label class="inline setting" data-help="Flip переворачивает картинку по вертикали. Нужно, если модуль установлен вверх ногами."><input id="vflip" type="checkbox"><span class="setting-name"><strong>Flip</strong><small>перевернуть</small></span></label>
          <label class="inline setting" data-help="Lens corr — коррекция объектива и неравномерной яркости по краям. Обычно лучше оставить включенной."><input id="lenc" type="checkbox" checked><span class="setting-name"><strong>Lens corr</strong><small>коррекция линзы</small></span></label>
          <label class="inline setting" data-help="Gamma — гамма-коррекция полутонов. Обычно оставляют включенной, чтобы JPEG выглядел естественнее."><input id="rawGma" type="checkbox" checked><span class="setting-name"><strong>Gamma</strong><small>полутона</small></span></label>
          <label class="inline setting" data-help="BPC — Black Pixel Correction, коррекция дефектных темных пикселей. Часто необязательно."><input id="bpc" type="checkbox"><span class="setting-name"><strong>BPC</strong><small>темные пиксели</small></span></label>
          <label class="inline setting" data-help="WPC — White Pixel Correction, коррекция дефектных светлых пикселей. Обычно можно держать включенной."><input id="wpc" type="checkbox" checked><span class="setting-name"><strong>WPC</strong><small>светлые пиксели</small></span></label>
          <label class="inline setting" data-help="DCW — внутренняя обработка уменьшения кадра сенсором OV2640. Для маленьких разрешений обычно лучше включить."><input id="dcw" type="checkbox" checked><span class="setting-name"><strong>DCW</strong><small>уменьшение кадра</small></span></label>
        </div>
        <div id="help" class="help">Автоматические режимы AWB, AEC и AGC могут менять картинку между кадрами. Наведи или тапни настройку, чтобы увидеть расшифровку.</div>
      </aside>
    </div>
  </main>

  <script>
    const streamEl = document.getElementById('stream');
    const emptyEl = document.getElementById('empty');
    const statusEl = document.getElementById('status');
    const errorEl = document.getElementById('error');
    const liveStatsEl = document.getElementById('liveStats');
    const streamToggleBtn = document.getElementById('streamToggle');
    const resetSettingsBtn = document.getElementById('resetSettings');
    const fpsEl = document.getElementById('fps');
    const resolutionEl = document.getElementById('resolution');
    const helpEl = document.getElementById('help');
    const defaultHelp = helpEl.textContent;

    const numericSettings = [
      'quality', 'brightness', 'contrast', 'saturation', 'sharpness', 'wbMode',
      'aeLevel', 'aecValue', 'agcGain', 'gainCeiling', 'discardFrames', 'settleMs'
    ];
    const booleanSettings = [
      'whiteBalance', 'awbGain', 'exposureCtrl', 'aec2', 'gainCtrl', 'hmirror',
      'vflip', 'lenc', 'rawGma', 'bpc', 'wpc', 'dcw'
    ];

    let streamRunning = true;
    let frameInFlight = false;
    let frameTimer = null;
    let objectUrl = null;
    let frameSeq = 0;
    let displayedFrames = 0;
    let lastFrameBytes = 0;
    let lastFrameCompletedAt = 0;
    let actualFps = 0;
    let controlsLoaded = false;

    function bindValueEcho(id) {
      const input = document.getElementById(id);
      const output = document.getElementById(`${id}Value`);
      if (!input || !output) return;
      const sync = () => { output.textContent = input.value; };
      input.addEventListener('input', sync);
      sync();
    }

    function bindSettingHelp() {
      for (const setting of document.querySelectorAll('.setting[data-help]')) {
        const show = () => { helpEl.textContent = setting.dataset.help; };
        const reset = () => { helpEl.textContent = defaultHelp; };
        setting.title = setting.dataset.help;
        setting.addEventListener('mouseenter', show);
        setting.addEventListener('focusin', show);
        setting.addEventListener('click', show);
        setting.addEventListener('mouseleave', reset);
      }
    }

    function syncValueEcho(id) {
      const output = document.getElementById(`${id}Value`);
      const input = document.getElementById(id);
      if (input && output) output.textContent = input.value;
    }

    function setControlValue(id, value) {
      const input = document.getElementById(id);
      if (!input || value === undefined || value === null) return;
      if (input.type === 'checkbox') {
        input.checked = Boolean(value);
      } else {
        input.value = String(value);
        syncValueEcho(id);
      }
    }

    function applyStatusSettings(status, force = false) {
      if (!force && controlsLoaded) return;
      const settings = status.settings || {};
      setControlValue('resolution', status.configured_resolution || status.resolution || 'qvga');
      setControlValue('fps', status.last_requested_fps || 2);
      setControlValue('quality', settings.quality);
      setControlValue('brightness', settings.brightness);
      setControlValue('contrast', settings.contrast);
      setControlValue('saturation', settings.saturation);
      setControlValue('sharpness', settings.sharpness);
      setControlValue('whiteBalance', settings.white_balance);
      setControlValue('awbGain', settings.awb_gain);
      setControlValue('wbMode', settings.wb_mode);
      setControlValue('exposureCtrl', settings.exposure_ctrl);
      setControlValue('aec2', settings.aec2);
      setControlValue('aeLevel', settings.ae_level);
      setControlValue('aecValue', settings.aec_value);
      setControlValue('gainCtrl', settings.gain_ctrl);
      setControlValue('agcGain', settings.agc_gain);
      setControlValue('gainCeiling', settings.gain_ceiling);
      setControlValue('hmirror', settings.hmirror);
      setControlValue('vflip', settings.vflip);
      setControlValue('lenc', settings.lenc);
      setControlValue('rawGma', settings.raw_gma);
      setControlValue('bpc', settings.bpc);
      setControlValue('wpc', settings.wpc);
      setControlValue('dcw', settings.dcw);
      setControlValue('discardFrames', settings.discard_frames);
      setControlValue('settleMs', settings.settle_ms);
      controlsLoaded = true;
      updateLiveStats();
    }

    function frameIntervalMs() {
      return Math.max(100, Math.round(1000 / Number(fpsEl.value || 2)));
    }

    function settingsParams() {
      const params = new URLSearchParams();
      params.set('res', resolutionEl.value);
      params.set('fps', fpsEl.value);
      params.set('request', `${Date.now()}-${++frameSeq}`);
      for (const id of numericSettings) {
        params.set(id, document.getElementById(id).value);
      }
      for (const id of booleanSettings) {
        params.set(id, document.getElementById(id).checked ? '1' : '0');
      }
      return params;
    }

    function updateLiveStats() {
      liveStatsEl.textContent = `target ${fpsEl.value} fps | actual ${actualFps.toFixed(1)} fps | frames ${displayedFrames} | last ${lastFrameBytes} bytes`;
    }

    function setStreamRunning(running) {
      streamRunning = running;
      streamToggleBtn.textContent = running ? 'Pause' : 'Start';
      streamToggleBtn.classList.toggle('paused', !running);
      if (!running && frameTimer) {
        clearTimeout(frameTimer);
        frameTimer = null;
      }
    }

    function scheduleNextFrame(delayMs) {
      if (!streamRunning) return;
      if (frameTimer) clearTimeout(frameTimer);
      frameTimer = setTimeout(fetchFrame, Math.max(0, delayMs));
    }

    async function refreshStatus() {
      try {
        const response = await fetch('/status', { cache: 'no-store' });
        const status = await response.json();
        applyStatusSettings(status);
        statusEl.textContent = `IP ${status.ip} | camera ${status.camera} | PSRAM ${status.psram} | active ${status.resolution} | server frames ${status.frames}`;
      } catch (error) {
        statusEl.textContent = 'Status unavailable';
      }
    }

    async function resetSettings() {
      const wasRunning = streamRunning;
      setStreamRunning(false);
      errorEl.textContent = '';
      try {
        const response = await fetch('/settings/reset', {
          method: 'POST',
          cache: 'no-store',
          headers: { 'Cache-Control': 'no-store' }
        });
        if (!response.ok) {
          throw new Error(await response.text());
        }
        const status = await response.json();
        applyStatusSettings(status, true);
        displayedFrames = 0;
        lastFrameBytes = 0;
        lastFrameCompletedAt = 0;
        actualFps = 0;
        updateLiveStats();
      } catch (error) {
        errorEl.textContent = error.message || 'Reset failed';
      } finally {
        setStreamRunning(wasRunning);
        if (streamRunning) fetchFrame();
      }
    }

    async function fetchFrame() {
      if (!streamRunning || frameInFlight) return;
      frameInFlight = true;
      const startedAt = performance.now();
      try {
        const response = await fetch(`/frame?${settingsParams().toString()}`, {
          cache: 'no-store',
          headers: { 'Cache-Control': 'no-store' }
        });
        if (!response.ok) {
          throw new Error(await response.text());
        }
        const blob = await response.blob();
        if (objectUrl) URL.revokeObjectURL(objectUrl);
        objectUrl = URL.createObjectURL(blob);
        streamEl.src = objectUrl;
        emptyEl.style.display = 'none';
        displayedFrames++;
        lastFrameBytes = blob.size;
        const completedAt = performance.now();
        if (lastFrameCompletedAt > 0) {
          actualFps = 1000 / Math.max(1, completedAt - lastFrameCompletedAt);
        }
        lastFrameCompletedAt = completedAt;
        errorEl.textContent = '';
        updateLiveStats();
      } catch (error) {
        errorEl.textContent = error.message || 'Frame failed';
      } finally {
        const elapsed = performance.now() - startedAt;
        frameInFlight = false;
        if (streamRunning) {
          scheduleNextFrame(frameIntervalMs() - elapsed);
        }
      }
    }

    function toggleStream() {
      setStreamRunning(!streamRunning);
      if (streamRunning) {
        fetchFrame();
      }
    }

    for (const id of ['quality', 'brightness', 'contrast', 'saturation', 'sharpness', 'aeLevel']) {
      bindValueEcho(id);
    }
    bindSettingHelp();
    streamToggleBtn.addEventListener('click', toggleStream);
    resetSettingsBtn.addEventListener('click', resetSettings);
    fpsEl.addEventListener('change', updateLiveStats);
    updateLiveStats();
    async function bootUi() {
      await refreshStatus();
      setInterval(refreshStatus, 2500);
      fetchFrame();
    }
    bootUi();
  </script>
</body>
</html>
)HTML";

int clampValue(int value, int minValue, int maxValue);

const ResolutionOption &configuredResolution() {
  return RESOLUTIONS[currentResolutionIndex];
}

int resolutionIndexForKey(const String &key) {
  for (size_t i = 0; i < sizeof(RESOLUTIONS) / sizeof(RESOLUTIONS[0]); i++) {
    if (key.equals(RESOLUTIONS[i].key)) {
      return static_cast<int>(i);
    }
  }
  return DEFAULT_RESOLUTION_INDEX;
}

int resolutionIndexForOption(const ResolutionOption &resolution) {
  for (size_t i = 0; i < sizeof(RESOLUTIONS) / sizeof(RESOLUTIONS[0]); i++) {
    if (&resolution == &RESOLUTIONS[i]) {
      return static_cast<int>(i);
    }
  }
  return resolutionIndexForKey(resolution.key);
}

bool settingsEqual(const CameraSettings &left, const CameraSettings &right) {
  return left.jpegQuality == right.jpegQuality &&
         left.brightness == right.brightness &&
         left.contrast == right.contrast &&
         left.saturation == right.saturation &&
         left.sharpness == right.sharpness &&
         left.whiteBalance == right.whiteBalance &&
         left.awbGain == right.awbGain &&
         left.wbMode == right.wbMode &&
         left.exposureCtrl == right.exposureCtrl &&
         left.aec2 == right.aec2 &&
         left.aeLevel == right.aeLevel &&
         left.aecValue == right.aecValue &&
         left.gainCtrl == right.gainCtrl &&
         left.agcGain == right.agcGain &&
         left.gainCeiling == right.gainCeiling &&
         left.hmirror == right.hmirror &&
         left.vflip == right.vflip &&
         left.lenc == right.lenc &&
         left.rawGma == right.rawGma &&
         left.bpc == right.bpc &&
         left.wpc == right.wpc &&
         left.dcw == right.dcw &&
         left.discardFrames == right.discardFrames &&
         left.settleMs == right.settleMs;
}

void savePersistentSettings() {
  if (!settingsStorageReady) {
    Serial.println("settings storage: save skipped, storage unavailable");
    return;
  }

  preferences.putUInt("ver", SETTINGS_VERSION);
  preferences.putInt("res", currentResolutionIndex);
  preferences.putInt("fps", lastRequestedFps);
  preferences.putInt("quality", currentSettings.jpegQuality);
  preferences.putInt("br", currentSettings.brightness);
  preferences.putInt("contrast", currentSettings.contrast);
  preferences.putInt("sat", currentSettings.saturation);
  preferences.putInt("sharp", currentSettings.sharpness);
  preferences.putInt("wb", currentSettings.whiteBalance);
  preferences.putInt("awbg", currentSettings.awbGain);
  preferences.putInt("wbm", currentSettings.wbMode);
  preferences.putInt("aec", currentSettings.exposureCtrl);
  preferences.putInt("aec2", currentSettings.aec2);
  preferences.putInt("ael", currentSettings.aeLevel);
  preferences.putInt("aecv", currentSettings.aecValue);
  preferences.putInt("agc", currentSettings.gainCtrl);
  preferences.putInt("agcg", currentSettings.agcGain);
  preferences.putInt("gceil", currentSettings.gainCeiling);
  preferences.putInt("hm", currentSettings.hmirror);
  preferences.putInt("vf", currentSettings.vflip);
  preferences.putInt("lenc", currentSettings.lenc);
  preferences.putInt("rgma", currentSettings.rawGma);
  preferences.putInt("bpc", currentSettings.bpc);
  preferences.putInt("wpc", currentSettings.wpc);
  preferences.putInt("dcw", currentSettings.dcw);
  preferences.putInt("disc", currentSettings.discardFrames);
  preferences.putInt("settle", currentSettings.settleMs);
  settingsSaveCount++;
  Serial.printf("settings saved: res=%s fps=%d save_count=%lu\n",
                configuredResolution().key, lastRequestedFps,
                static_cast<unsigned long>(settingsSaveCount));
}

void loadPersistentSettings() {
  if (!preferences.begin(SETTINGS_NAMESPACE, false)) {
    Serial.println("settings storage: open failed, using defaults");
    return;
  }
  settingsStorageReady = true;

  const uint32_t version = preferences.getUInt("ver", 0);
  if (version != SETTINGS_VERSION) {
    currentSettings = CameraSettings();
    currentResolutionIndex = DEFAULT_RESOLUTION_INDEX;
    lastRequestedFps = DEFAULT_FPS;
    savePersistentSettings();
    Serial.println("settings storage: initialized defaults");
    return;
  }

  currentResolutionIndex = clampValue(preferences.getInt("res", DEFAULT_RESOLUTION_INDEX),
                                      0, static_cast<int>((sizeof(RESOLUTIONS) / sizeof(RESOLUTIONS[0])) - 1));
  lastRequestedFps = clampValue(preferences.getInt("fps", DEFAULT_FPS), 1, 10);
  currentSettings.jpegQuality = clampValue(preferences.getInt("quality", currentSettings.jpegQuality), 10, 63);
  currentSettings.brightness = clampValue(preferences.getInt("br", currentSettings.brightness), -2, 2);
  currentSettings.contrast = clampValue(preferences.getInt("contrast", currentSettings.contrast), -2, 2);
  currentSettings.saturation = clampValue(preferences.getInt("sat", currentSettings.saturation), -2, 2);
  currentSettings.sharpness = clampValue(preferences.getInt("sharp", currentSettings.sharpness), -2, 2);
  currentSettings.whiteBalance = clampValue(preferences.getInt("wb", currentSettings.whiteBalance), 0, 1);
  currentSettings.awbGain = clampValue(preferences.getInt("awbg", currentSettings.awbGain), 0, 1);
  currentSettings.wbMode = clampValue(preferences.getInt("wbm", currentSettings.wbMode), 0, 4);
  currentSettings.exposureCtrl = clampValue(preferences.getInt("aec", currentSettings.exposureCtrl), 0, 1);
  currentSettings.aec2 = clampValue(preferences.getInt("aec2", currentSettings.aec2), 0, 1);
  currentSettings.aeLevel = clampValue(preferences.getInt("ael", currentSettings.aeLevel), -2, 2);
  currentSettings.aecValue = clampValue(preferences.getInt("aecv", currentSettings.aecValue), 0, 1200);
  currentSettings.gainCtrl = clampValue(preferences.getInt("agc", currentSettings.gainCtrl), 0, 1);
  currentSettings.agcGain = clampValue(preferences.getInt("agcg", currentSettings.agcGain), 0, 30);
  currentSettings.gainCeiling = clampValue(preferences.getInt("gceil", currentSettings.gainCeiling), 0, 6);
  currentSettings.hmirror = clampValue(preferences.getInt("hm", currentSettings.hmirror), 0, 1);
  currentSettings.vflip = clampValue(preferences.getInt("vf", currentSettings.vflip), 0, 1);
  currentSettings.lenc = clampValue(preferences.getInt("lenc", currentSettings.lenc), 0, 1);
  currentSettings.rawGma = clampValue(preferences.getInt("rgma", currentSettings.rawGma), 0, 1);
  currentSettings.bpc = clampValue(preferences.getInt("bpc", currentSettings.bpc), 0, 1);
  currentSettings.wpc = clampValue(preferences.getInt("wpc", currentSettings.wpc), 0, 1);
  currentSettings.dcw = clampValue(preferences.getInt("dcw", currentSettings.dcw), 0, 1);
  currentSettings.discardFrames = clampValue(preferences.getInt("disc", currentSettings.discardFrames), 0, 2);
  currentSettings.settleMs = clampValue(preferences.getInt("settle", currentSettings.settleMs), 0, 2000);
  Serial.printf("settings loaded: res=%s fps=%d\n", configuredResolution().key,
                lastRequestedFps);
}

void resetPersistentSettings() {
  if (settingsStorageReady) {
    preferences.clear();
  }
  currentSettings = CameraSettings();
  currentResolutionIndex = DEFAULT_RESOLUTION_INDEX;
  lastRequestedFps = DEFAULT_FPS;
  cameraReady = false;
  activeResolution = nullptr;
  savePersistentSettings();
  Serial.println("settings reset to defaults");
}

int clampValue(int value, int minValue, int maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

int requestInt(const char *name, int fallback, int minValue, int maxValue) {
  if (!server.hasArg(name)) {
    return fallback;
  }
  return clampValue(server.arg(name).toInt(), minValue, maxValue);
}

int requestBool(const char *name, int fallback) {
  if (!server.hasArg(name)) {
    return fallback;
  }
  const String value = server.arg(name);
  return value == "1" || value == "true" || value == "on" ? 1 : 0;
}

void updateSettingsFromRequest(const ResolutionOption &resolution, int requestedFps) {
  const CameraSettings previousSettings = currentSettings;
  const int previousResolutionIndex = currentResolutionIndex;
  const int previousFps = lastRequestedFps;

  currentResolutionIndex = resolutionIndexForOption(resolution);
  lastRequestedFps = requestedFps;
  currentSettings.jpegQuality = requestInt("quality", currentSettings.jpegQuality, 10, 63);
  currentSettings.brightness = requestInt("brightness", currentSettings.brightness, -2, 2);
  currentSettings.contrast = requestInt("contrast", currentSettings.contrast, -2, 2);
  currentSettings.saturation = requestInt("saturation", currentSettings.saturation, -2, 2);
  currentSettings.sharpness = requestInt("sharpness", currentSettings.sharpness, -2, 2);
  currentSettings.whiteBalance = requestBool("whiteBalance", currentSettings.whiteBalance);
  currentSettings.awbGain = requestBool("awbGain", currentSettings.awbGain);
  currentSettings.wbMode = requestInt("wbMode", currentSettings.wbMode, 0, 4);
  currentSettings.exposureCtrl = requestBool("exposureCtrl", currentSettings.exposureCtrl);
  currentSettings.aec2 = requestBool("aec2", currentSettings.aec2);
  currentSettings.aeLevel = requestInt("aeLevel", currentSettings.aeLevel, -2, 2);
  currentSettings.aecValue = requestInt("aecValue", currentSettings.aecValue, 0, 1200);
  currentSettings.gainCtrl = requestBool("gainCtrl", currentSettings.gainCtrl);
  currentSettings.agcGain = requestInt("agcGain", currentSettings.agcGain, 0, 30);
  currentSettings.gainCeiling = requestInt("gainCeiling", currentSettings.gainCeiling, 0, 6);
  currentSettings.hmirror = requestBool("hmirror", currentSettings.hmirror);
  currentSettings.vflip = requestBool("vflip", currentSettings.vflip);
  currentSettings.lenc = requestBool("lenc", currentSettings.lenc);
  currentSettings.rawGma = requestBool("rawGma", currentSettings.rawGma);
  currentSettings.bpc = requestBool("bpc", currentSettings.bpc);
  currentSettings.wpc = requestBool("wpc", currentSettings.wpc);
  currentSettings.dcw = requestBool("dcw", currentSettings.dcw);
  currentSettings.discardFrames = requestInt("discardFrames", currentSettings.discardFrames, 0, 2);
  currentSettings.settleMs = requestInt("settleMs", currentSettings.settleMs, 0, 2000);

  if (!settingsEqual(previousSettings, currentSettings) ||
      previousResolutionIndex != currentResolutionIndex ||
      previousFps != lastRequestedFps) {
    savePersistentSettings();
  }
}

const ResolutionOption *findResolution(const String &key) {
  for (const ResolutionOption &resolution : RESOLUTIONS) {
    if (key.equals(resolution.key)) {
      return &resolution;
    }
  }
  return nullptr;
}

const char *httpMethodName(HTTPMethod method) {
  switch (method) {
    case HTTP_GET:
      return "GET";
    case HTTP_POST:
      return "POST";
    case HTTP_DELETE:
      return "DELETE";
    case HTTP_PUT:
      return "PUT";
    case HTTP_PATCH:
      return "PATCH";
    case HTTP_HEAD:
      return "HEAD";
    case HTTP_OPTIONS:
      return "OPTIONS";
    default:
      return "OTHER";
  }
}

bool isJpeg(const camera_fb_t *fb) {
  return fb != nullptr && fb->len >= 4 && fb->buf[0] == 0xFF &&
         fb->buf[1] == 0xD8 && fb->buf[fb->len - 2] == 0xFF &&
         fb->buf[fb->len - 1] == 0xD9;
}

camera_config_t makeCameraConfig(const ResolutionOption &resolution) {
  const bool hasPsram = psramFound();
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
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = resolution.frameSize;
  config.jpeg_quality = currentSettings.jpegQuality;
  config.fb_count = 1;
  config.fb_location = hasPsram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  return config;
}

bool applySensorSettings() {
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor == nullptr) {
    Serial.println("sensor settings failed: sensor unavailable");
    return false;
  }

  int failures = 0;
  if (sensor->set_quality && sensor->set_quality(sensor, currentSettings.jpegQuality) != 0) failures++;
  if (sensor->set_brightness && sensor->set_brightness(sensor, currentSettings.brightness) != 0) failures++;
  if (sensor->set_contrast && sensor->set_contrast(sensor, currentSettings.contrast) != 0) failures++;
  if (sensor->set_saturation && sensor->set_saturation(sensor, currentSettings.saturation) != 0) failures++;
  if (sensor->set_sharpness && sensor->set_sharpness(sensor, currentSettings.sharpness) != 0) failures++;
  if (sensor->set_whitebal && sensor->set_whitebal(sensor, currentSettings.whiteBalance) != 0) failures++;
  if (sensor->set_awb_gain && sensor->set_awb_gain(sensor, currentSettings.awbGain) != 0) failures++;
  if (sensor->set_wb_mode && sensor->set_wb_mode(sensor, currentSettings.wbMode) != 0) failures++;
  if (sensor->set_exposure_ctrl && sensor->set_exposure_ctrl(sensor, currentSettings.exposureCtrl) != 0) failures++;
  if (sensor->set_aec2 && sensor->set_aec2(sensor, currentSettings.aec2) != 0) failures++;
  if (sensor->set_ae_level && sensor->set_ae_level(sensor, currentSettings.aeLevel) != 0) failures++;
  if (sensor->set_aec_value && sensor->set_aec_value(sensor, currentSettings.aecValue) != 0) failures++;
  if (sensor->set_gain_ctrl && sensor->set_gain_ctrl(sensor, currentSettings.gainCtrl) != 0) failures++;
  if (sensor->set_agc_gain && sensor->set_agc_gain(sensor, currentSettings.agcGain) != 0) failures++;
  if (sensor->set_gainceiling && sensor->set_gainceiling(sensor, static_cast<gainceiling_t>(currentSettings.gainCeiling)) != 0) failures++;
  if (sensor->set_hmirror && sensor->set_hmirror(sensor, currentSettings.hmirror) != 0) failures++;
  if (sensor->set_vflip && sensor->set_vflip(sensor, currentSettings.vflip) != 0) failures++;
  if (sensor->set_lenc && sensor->set_lenc(sensor, currentSettings.lenc) != 0) failures++;
  if (sensor->set_raw_gma && sensor->set_raw_gma(sensor, currentSettings.rawGma) != 0) failures++;
  if (sensor->set_bpc && sensor->set_bpc(sensor, currentSettings.bpc) != 0) failures++;
  if (sensor->set_wpc && sensor->set_wpc(sensor, currentSettings.wpc) != 0) failures++;
  if (sensor->set_dcw && sensor->set_dcw(sensor, currentSettings.dcw) != 0) failures++;

  settingsApplyCount++;
  Serial.printf("sensor settings: quality=%d brightness=%d contrast=%d saturation=%d sharpness=%d wb=%d awb_gain=%d wb_mode=%d aec=%d aec2=%d ae_level=%d aec_value=%d agc=%d agc_gain=%d gain_ceiling=%d mirror=%d flip=%d failures=%d apply_count=%lu\n",
                currentSettings.jpegQuality, currentSettings.brightness,
                currentSettings.contrast, currentSettings.saturation,
                currentSettings.sharpness, currentSettings.whiteBalance,
                currentSettings.awbGain, currentSettings.wbMode,
                currentSettings.exposureCtrl, currentSettings.aec2,
                currentSettings.aeLevel, currentSettings.aecValue,
                currentSettings.gainCtrl, currentSettings.agcGain,
                currentSettings.gainCeiling, currentSettings.hmirror,
                currentSettings.vflip, failures,
                static_cast<unsigned long>(settingsApplyCount));
  return true;
}

bool ensureCamera(const ResolutionOption &resolution) {
  if (cameraReady && activeResolution == &resolution) {
    return true;
  }

  cameraReady = false;
  Serial.printf("camera init: %s, psram=%s\n", resolution.key,
                psramFound() ? "yes" : "no");
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
    activeResolution = nullptr;
    return false;
  }

  activeResolution = &resolution;
  cameraReady = true;
  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor != nullptr) {
    Serial.printf("camera ready: sensor=0x%02X resolution=%s fb=%s quality=%d\n",
                  sensor->id.PID, resolution.key,
                  config.fb_location == CAMERA_FB_IN_PSRAM ? "PSRAM" : "DRAM",
                  config.jpeg_quality);
  }
  applySensorSettings();
  return true;
}

struct FrameLock {
  explicit FrameLock(bool &locked) : lockedRef(locked) {
    lockedRef = true;
  }

  ~FrameLock() {
    lockedRef = false;
  }

  bool &lockedRef;
};

void sendPlainError(int status, const String &message) {
  Serial.printf("http error %d: %s\n", status, message.c_str());
  server.send(status, "text/plain", message);
}

void handleRoot() {
  server.send_P(200, "text/html", INDEX_HTML);
}

void appendSettingsJson(String &json) {
  json += "\"settings\":{";
  json += "\"quality\":" + String(currentSettings.jpegQuality) + ",";
  json += "\"brightness\":" + String(currentSettings.brightness) + ",";
  json += "\"contrast\":" + String(currentSettings.contrast) + ",";
  json += "\"saturation\":" + String(currentSettings.saturation) + ",";
  json += "\"sharpness\":" + String(currentSettings.sharpness) + ",";
  json += "\"white_balance\":" + String(currentSettings.whiteBalance) + ",";
  json += "\"awb_gain\":" + String(currentSettings.awbGain) + ",";
  json += "\"wb_mode\":" + String(currentSettings.wbMode) + ",";
  json += "\"exposure_ctrl\":" + String(currentSettings.exposureCtrl) + ",";
  json += "\"aec2\":" + String(currentSettings.aec2) + ",";
  json += "\"ae_level\":" + String(currentSettings.aeLevel) + ",";
  json += "\"aec_value\":" + String(currentSettings.aecValue) + ",";
  json += "\"gain_ctrl\":" + String(currentSettings.gainCtrl) + ",";
  json += "\"agc_gain\":" + String(currentSettings.agcGain) + ",";
  json += "\"gain_ceiling\":" + String(currentSettings.gainCeiling) + ",";
  json += "\"hmirror\":" + String(currentSettings.hmirror) + ",";
  json += "\"vflip\":" + String(currentSettings.vflip) + ",";
  json += "\"lenc\":" + String(currentSettings.lenc) + ",";
  json += "\"raw_gma\":" + String(currentSettings.rawGma) + ",";
  json += "\"bpc\":" + String(currentSettings.bpc) + ",";
  json += "\"wpc\":" + String(currentSettings.wpc) + ",";
  json += "\"dcw\":" + String(currentSettings.dcw) + ",";
  json += "\"discard_frames\":" + String(currentSettings.discardFrames) + ",";
  json += "\"settle_ms\":" + String(currentSettings.settleMs);
  json += "}";
}

void handleStatus() {
  String json = "{";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"camera\":\"" + String(cameraReady ? "ready" : "not_ready") + "\",";
  json += "\"psram\":\"" + String(psramFound() ? "yes" : "no") + "\",";
  json += "\"psram_free\":" + String(ESP.getFreePsram()) + ",";
  json += "\"configured_resolution\":\"" + String(configuredResolution().key) + "\",";
  json += "\"resolution\":\"" + String(activeResolution ? activeResolution->key : "none") + "\",";
  json += "\"frame_requests\":" + String(frameRequestCount) + ",";
  json += "\"frames\":" + String(frameCount) + ",";
  json += "\"last_frame_bytes\":" + String(lastFrameBytes) + ",";
  json += "\"last_requested_fps\":" + String(lastRequestedFps) + ",";
  json += "\"settings_storage\":\"" + String(settingsStorageReady ? "ready" : "unavailable") + "\",";
  json += "\"settings_saves\":" + String(settingsSaveCount) + ",";
  json += "\"settings_apply_count\":" + String(settingsApplyCount) + ",";
  json += "\"last_error\":\"" + String(esp_err_to_name(lastCameraError)) + "\",";
  appendSettingsJson(json);
  json += "}";
  server.send(200, "application/json", json);
}

void handleSettingsReset() {
  resetPersistentSettings();
  handleStatus();
}

bool discardWarmupFrames(int count) {
  for (int i = 0; i < count; i++) {
    camera_fb_t *discarded = esp_camera_fb_get();
    if (discarded == nullptr) {
      Serial.printf("discard frame failed: index=%d\n", i + 1);
      return false;
    }
    Serial.printf("discard frame: index=%d bytes=%u\n", i + 1,
                  static_cast<unsigned>(discarded->len));
    esp_camera_fb_return(discarded);
    delay(30);
  }
  return true;
}

void handleFrame() {
  String key = server.arg("res");
  if (key.isEmpty()) {
    key = "qvga";
  }

  const int requestedFps = requestInt("fps", lastRequestedFps, 1, 10);
  frameRequestCount++;
  const String requestId = server.arg("request");
  const String remoteIp = server.client().remoteIP().toString();
  Serial.printf("frame request: method=%s uri=%s res=%s fps=%d request=%s remote=%s request_count=%lu\n",
                httpMethodName(server.method()), server.uri().c_str(), key.c_str(),
                requestedFps, requestId.isEmpty() ? "-" : requestId.c_str(),
                remoteIp.c_str(), static_cast<unsigned long>(frameRequestCount));

  const unsigned long now = millis();
  if (!requestId.isEmpty() && requestId == lastFrameRequestId &&
      now - lastFrameRequestAt < 10000) {
    sendPlainError(409, "Duplicate frame request ignored.");
    return;
  }

  lastFrameRequestId = requestId;
  lastFrameRequestAt = now;

  if (frameInProgress) {
    sendPlainError(503, "Frame already in progress.");
    return;
  }

  FrameLock frameLock(frameInProgress);

  const ResolutionOption *resolution = findResolution(key);
  if (resolution == nullptr) {
    sendPlainError(400, "Unknown resolution. Use qqvga, qvga, or vga.");
    return;
  }

  updateSettingsFromRequest(*resolution, requestedFps);

  if (!ensureCamera(*resolution)) {
    String message = "Camera init failed for ";
    message += resolution->key;
    message += ": ";
    message += esp_err_to_name(lastCameraError);
    if (resolution->unstableWithoutPsram && !psramFound()) {
      message += " (VGA is unstable without PSRAM)";
    }
    sendPlainError(503, message);
    return;
  }

  if (!applySensorSettings()) {
    sendPlainError(503, "Sensor settings failed.");
    return;
  }

  if (currentSettings.settleMs > 0) {
    Serial.printf("settle delay: %d ms\n", currentSettings.settleMs);
    delay(currentSettings.settleMs);
  }

  if (!discardWarmupFrames(currentSettings.discardFrames)) {
    sendPlainError(503, "Discard frame failed before capture.");
    return;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (fb == nullptr) {
    sendPlainError(503, "Frame failed: esp_camera_fb_get returned null.");
    return;
  }

  if (!isJpeg(fb)) {
    esp_camera_fb_return(fb);
    sendPlainError(500, "Frame failed: JPEG markers are invalid.");
    return;
  }

  frameCount++;
  lastFrameBytes = fb->len;
  Serial.printf("frame ok: res=%s bytes=%u quality=%d fps=%d count=%lu at=%lu ms\n",
                resolution->key, static_cast<unsigned>(fb->len),
                currentSettings.jpegQuality, lastRequestedFps,
                static_cast<unsigned long>(frameCount),
                static_cast<unsigned long>(millis()));
  Serial.println("jpeg markers: ok");

  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("X-Frame-Count", String(frameCount));
  server.sendHeader("X-Frame-Bytes", String(lastFrameBytes));
  server.setContentLength(fb->len);
  server.send(200, "image/jpeg", "");
  WiFiClient client = server.client();
  client.write(fb->buf, fb->len);
  esp_camera_fb_return(fb);
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
  server.on("/settings/reset", HTTP_GET, handleSettingsReset);
  server.on("/settings/reset", HTTP_POST, handleSettingsReset);
  server.on("/frame", HTTP_GET, handleFrame);
  server.on("/capture", HTTP_GET, handleFrame);
  server.on("/capture", HTTP_POST, handleFrame);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started");
}

void printBootStatus() {
  Serial.println();
  Serial.println("ESP32-CAM web photo firmware");
  Serial.printf("psramFound: %s\n", psramFound() ? "yes" : "no");
  Serial.printf("free heap: %u bytes\n", static_cast<unsigned>(ESP.getFreeHeap()));
  Serial.printf("free psram: %u bytes\n", static_cast<unsigned>(ESP.getFreePsram()));
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1200);
  loadPersistentSettings();
  printBootStatus();
  connectWifi();
  startHttpServer();
  ensureCamera(configuredResolution());
}

void loop() {
  server.handleClient();
  delay(2);
}
