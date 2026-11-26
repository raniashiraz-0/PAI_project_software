// ============================================================================
//  PHARMA ROVER - ESP32-CAM (AI-Thinker) HAND CAMERA
//
//  * Connects to the hotspot
//  * Finds the laptop server automatically (listens for the server's UDP
//    "PRSRV <port>" broadcast) or uses SERVER_IP from config.h
//  * Uploads JPEG frames with HTTP POST to /api/vision/hand_frame, where the
//    server runs YOLO / ArUco / barcode checks to find the right medicine box
//  * Sets the flash LED brightness the server asks for in its reply
//  Upload: VS Code -> PlatformIO -> PROJECT TASKS -> esp32_cam -> Upload
// ============================================================================
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <HTTPClient.h>
#include "esp_camera.h"
#include "config.h"

// AI-Thinker ESP32-CAM pin map
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

WiFiUDP udp;
HTTPClient http;
String serverIp = SERVER_IP;
uint16_t serverPort = SERVER_PORT;
uint32_t lastFrame = 0, lastWifiAttempt = 0, lastLog = 0;
uint32_t okCount = 0, failCount = 0;
int consecutiveFails = 0;
bool wifiWasUp = false;
int ledDuty = FLASH_DEFAULT_DUTY;

bool cameraBegin() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM; c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAME_SIZE;
  c.jpeg_quality = JPEG_QUALITY;
  c.fb_count = psramFound() ? 2 : 1;
  c.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;        // always send the newest picture

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("[CAM] init failed 0x%x - check the ribbon cable and power\n", err);
    return false;
  }
  sensor_t* s = esp_camera_sensor_get();
  s->set_vflip(s, FLIP_VERTICAL);
  s->set_hmirror(s, MIRROR_IMAGE);
  return true;
}

void setLed(int duty) {
  duty = constrain(duty, 0, 255);
  if (duty == ledDuty) return;
  ledDuty = duty;
  analogWrite(FLASH_LED_PIN, duty);
}

void wifiUpdate() {
  bool up = WiFi.status() == WL_CONNECTED;
  if (up && !wifiWasUp) {
    Serial.printf("[WIFI] connected, IP %s\n", WiFi.localIP().toString().c_str());
    udp.stop();
    udp.begin(DISCOVERY_UDP_PORT);
  }
  if (!up && millis() - lastWifiAttempt > 15000) {
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWifiAttempt = millis();
  }
  wifiWasUp = up;
}

// Server broadcasts "PRSRV 8000" every ~2 s
void discoveryUpdate() {
  int n = udp.parsePacket();
  if (n <= 0) return;
  char buf[64];
  int len = udp.read(buf, sizeof(buf) - 1);
  if (len <= 0) return;
  buf[len] = 0;
  if (strncmp(buf, "PRSRV", 5) != 0) return;

  String ip = udp.remoteIP().toString();
  int port = atoi(buf + 5);
  if (strlen(SERVER_IP) == 0 && (ip != serverIp || (port > 0 && port != serverPort))) {
    serverIp = ip;
    if (port > 0) serverPort = port;
    Serial.printf("[DISCOVERY] server found at %s:%u\n", serverIp.c_str(), serverPort);
    http.end();
  }
  char reply[64];
  snprintf(reply, sizeof(reply), "PRDEV cam %s", WiFi.localIP().toString().c_str());
  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  udp.write((const uint8_t*)reply, strlen(reply));
  udp.endPacket();
}

void uploadFrame() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { Serial.println("[CAM] capture failed"); return; }

  String url = "http://" + serverIp + ":" + String(serverPort) + UPLOAD_PATH;
  http.setReuse(true);
  http.setTimeout(1500);
  if (!http.begin(url)) { esp_camera_fb_return(fb); return; }
  http.addHeader("Content-Type", "image/jpeg");
  int code = http.POST(fb->buf, fb->len);
  esp_camera_fb_return(fb);

  if (code == 200) {
    okCount++; consecutiveFails = 0;
    String body = http.getString();           // {"ok": true, ..., "led": 0}
    int k = body.indexOf("\"led\":");
    if (k >= 0) setLed(body.substring(k + 6).toInt());
  } else {
    failCount++; consecutiveFails++;
    http.end();
    if (consecutiveFails >= 10 && strlen(SERVER_IP) == 0) {
      Serial.println("[UPLOAD] server not answering, waiting for discovery again");
      serverIp = "";
      consecutiveFails = 0;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== Pharma Rover ESP32-CAM ===");
  pinMode(FLASH_LED_PIN, OUTPUT);
  analogWrite(FLASH_LED_PIN, ledDuty);

  while (!cameraBegin()) delay(2000);

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(DEVICE_HOSTNAME);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lastWifiAttempt = millis();
}

void loop() {
  wifiUpdate();
  if (WiFi.status() != WL_CONNECTED) { delay(50); return; }

  discoveryUpdate();

  uint32_t now = millis();
  if (serverIp.length() > 0 && now - lastFrame >= 1000 / MAX_FPS) {
    lastFrame = now;
    uploadFrame();
  }
  if (now - lastLog > 5000) {
    lastLog = now;
    Serial.printf("[STATUS] server=%s frames ok=%lu failed=%lu led=%d\n",
                  serverIp.length() ? serverIp.c_str() : "(searching...)",
                  (unsigned long)okCount, (unsigned long)failCount, ledDuty);
  }
  delay(2);
}
