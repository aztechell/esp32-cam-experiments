#include <Arduino.h>
#include <Wire.h>
#include "esp_heap_caps.h"
#include "esp_err.h"
#include "esp_camera.h"

#define CAMERA_MODEL_AI_THINKER

#if defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22
#else
#error "Unsupported camera model"
#endif

namespace {

bool cameraReady = false;
esp_err_t cameraInitError = ESP_OK;
const char *activeFrameSize = "unknown";
const char *activeFrameBuffer = "unknown";
unsigned long lastHeartbeatAt = 0;
unsigned long lastCaptureAt = 0;
unsigned long lastInitAttemptAt = 0;
unsigned long lastPsramProbeAt = 0;
uint32_t heartbeatCount = 0;
uint32_t initAttemptCount = 0;

void printPsramProbe() {
  Serial.println("PSRAM probe:");
  Serial.printf("  BOARD_HAS_PSRAM: %s\n",
#if defined(BOARD_HAS_PSRAM)
                "defined"
#else
                "not defined"
#endif
  );
  Serial.printf("  psramFound(): %s\n", psramFound() ? "yes" : "no");
  Serial.printf("  psramInit(): %s\n", psramInit() ? "ok" : "failed");
  Serial.printf("  ESP.getPsramSize(): %u bytes\n",
                static_cast<unsigned>(ESP.getPsramSize()));
  Serial.printf("  ESP.getFreePsram(): %u bytes\n",
                static_cast<unsigned>(ESP.getFreePsram()));
  Serial.printf("  heap SPIRAM free: %u bytes\n",
                static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
  Serial.printf("  heap SPIRAM largest block: %u bytes\n",
                static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM)));

  void *testBlock = ps_malloc(1024);
  Serial.printf("  ps_malloc(1024): %s\n", testBlock != nullptr ? "ok" : "failed");
  if (testBlock != nullptr) {
    memset(testBlock, 0xA5, 1024);
    free(testBlock);
  }
}

camera_config_t makeCameraConfig() {
  camera_config_t config = {};
  const bool hasPsram = psramFound();
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
  config.frame_size = hasPsram ? FRAMESIZE_QVGA : FRAMESIZE_QQVGA;
  config.jpeg_quality = 14;
  config.fb_count = 1;
  config.fb_location = hasPsram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  activeFrameSize = hasPsram ? "QVGA" : "QQVGA";
  activeFrameBuffer = hasPsram ? "PSRAM" : "DRAM";

  return config;
}

const char *cameraErrorHint(esp_err_t err) {
  switch (err) {
    case ESP_OK:
      return "ok";
    case ESP_ERR_CAMERA_NOT_DETECTED:
      return "camera sensor not detected on SCCB/I2C; check ribbon cable, camera orientation, 5V power, and AI Thinker pinout";
    case ESP_ERR_CAMERA_FAILED_TO_SET_FRAME_SIZE:
      return "sensor responded, but frame-size setup failed; possible wrong sensor module or unstable camera power";
    case ESP_ERR_CAMERA_FAILED_TO_SET_OUT_FORMAT:
      return "sensor responded, but JPEG/output-format setup failed; possible non-OV2640 module or unstable camera power";
    case ESP_ERR_CAMERA_NOT_SUPPORTED:
      return "camera/sensor combination is not supported by this driver";
    case ESP_ERR_NO_MEM:
      return "not enough RAM for frame buffers; lower frame size or force one DRAM frame buffer";
    case ESP_ERR_INVALID_ARG:
      return "invalid camera configuration; likely wrong pin map for this module";
    case ESP_ERR_INVALID_STATE:
      return "camera driver is already initialized or stuck in an invalid state";
    case ESP_FAIL:
      return "generic camera driver failure; recent ESP log lines show the concrete cause";
    default:
      return "unknown camera init failure; enable verbose logs and inspect wiring/power";
  }
}

bool isJpeg(const camera_fb_t *fb) {
  return fb != nullptr && fb->len >= 4 && fb->buf[0] == 0xFF &&
         fb->buf[1] == 0xD8 && fb->buf[fb->len - 2] == 0xFF &&
         fb->buf[fb->len - 1] == 0xD9;
}

void printCameraStatus(sensor_t *sensor) {
  Serial.printf("sensor pid: 0x%02X\n", sensor->id.PID);
  Serial.printf("sensor ver: 0x%02X\n", sensor->id.VER);
  Serial.printf("psram: %s\n", psramFound() ? "yes" : "no");
  Serial.printf("frame size: %s JPEG\n", activeFrameSize);
  Serial.printf("frame buffer: %s, count: 1\n", activeFrameBuffer);
}

void printBoardStatus() {
  Serial.printf("reset reason: %d\n", static_cast<int>(esp_reset_reason()));
  Serial.printf("free heap: %u bytes\n", static_cast<unsigned>(ESP.getFreeHeap()));
  Serial.printf("psram: %s\n", psramFound() ? "yes" : "no");
  if (psramFound()) {
    Serial.printf("free psram: %u bytes\n", static_cast<unsigned>(ESP.getFreePsram()));
  } else {
    Serial.println("psram warning: AI Thinker normally has PSRAM; using tiny DRAM capture mode");
  }
  Serial.println("pin map: AI Thinker ESP32-CAM");
  Serial.println("  PWDN=32 RESET=-1 XCLK=0 SIOD=26 SIOC=27");
  Serial.println("  Y9=35 Y8=34 Y7=39 Y6=36 Y5=21 Y4=19 Y3=18 Y2=5 VSYNC=25 HREF=23 PCLK=22");
}

void scanSccbBus() {
  Serial.println("SCCB scan on SIOD=26 SIOC=27...");
  Wire.begin(SIOD_GPIO_NUM, SIOC_GPIO_NUM);
  Wire.setClock(100000);

  uint8_t found = 0;
  for (uint8_t address = 0x08; address <= 0x77; address++) {
    Wire.beginTransmission(address);
    uint8_t result = Wire.endTransmission();
    if (result == 0) {
      Serial.printf("  SCCB/I2C device found at 0x%02X\n", address);
      found++;
    }
  }

  if (found == 0) {
    Serial.println("  no SCCB/I2C ACKs; check camera ribbon orientation, seating, and camera power");
  }

  Wire.end();
}

void printCameraInitError() {
  if (cameraInitError == ESP_OK) {
    return;
  }

  Serial.printf("camera init error: 0x%08X (%s)\n", cameraInitError,
                esp_err_to_name(cameraInitError));
  Serial.printf("diagnosis: %s\n", cameraErrorHint(cameraInitError));
}

bool initCamera() {
  initAttemptCount++;
  cameraReady = false;

  Serial.printf("camera init attempt: %lu\n",
                static_cast<unsigned long>(initAttemptCount));

  esp_camera_deinit();

  pinMode(PWDN_GPIO_NUM, OUTPUT);
  digitalWrite(PWDN_GPIO_NUM, LOW);
  delay(50);
  scanSccbBus();

  camera_config_t config = makeCameraConfig();
  Serial.printf("camera config: frame=%s jpeg_quality=%d fb=%s fb_count=%u\n",
                activeFrameSize, config.jpeg_quality, activeFrameBuffer,
                static_cast<unsigned>(config.fb_count));
  cameraInitError = esp_camera_init(&config);
  if (cameraInitError != ESP_OK) {
    printCameraInitError();
    return false;
  }

  cameraReady = true;
  Serial.println("camera init ok");

  sensor_t *sensor = esp_camera_sensor_get();
  if (sensor != nullptr) {
    printCameraStatus(sensor);
  } else {
    Serial.println("sensor metadata unavailable");
  }

  return true;
}

void captureFrame() {
  if (!cameraReady) {
    Serial.println("capture skipped: camera not ready");
    return;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (fb == nullptr) {
    Serial.println("capture failed: esp_camera_fb_get returned null");
    return;
  }

  Serial.printf("capture ok: %u bytes\n", static_cast<unsigned>(fb->len));
  Serial.printf("jpeg markers: %s\n", isJpeg(fb) ? "ok" : "bad");
  esp_camera_fb_return(fb);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println();
  Serial.println("ESP32-CAM serial probe");
  Serial.println("camera model: AI Thinker / OV2640");
  printBoardStatus();
  printPsramProbe();

  if (!initCamera()) {
    return;
  }

  captureFrame();
  Serial.println("probe done");
}

void loop() {
  const unsigned long now = millis();

  if (now - lastHeartbeatAt >= 2000) {
    lastHeartbeatAt = now;
    heartbeatCount++;
    Serial.printf("heartbeat: %lu ms, camera: %s, psram: %s, psram_free: %u, fb: %s, frame: %s, count: %lu\n",
                  static_cast<unsigned long>(now), cameraReady ? "ready" : "not ready",
                  psramFound() ? "yes" : "no", static_cast<unsigned>(ESP.getFreePsram()),
                  activeFrameBuffer, activeFrameSize,
                  static_cast<unsigned long>(heartbeatCount));
    printCameraInitError();
  }

  if (cameraReady && now - lastCaptureAt >= 10000) {
    lastCaptureAt = now;
    captureFrame();
  }

  if (!psramFound() && now - lastPsramProbeAt >= 10000) {
    lastPsramProbeAt = now;
    printPsramProbe();
  }

  if (!cameraReady && now - lastInitAttemptAt >= 10000) {
    lastInitAttemptAt = now;
    initCamera();
  }

  delay(20);
}
