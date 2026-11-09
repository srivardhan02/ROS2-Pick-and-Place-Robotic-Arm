// ESP32-CAM (AI-Thinker) -> UDP JPEG streamer
// Packet: 6-byte header (frame_id, chunk_idx, chunk_count; uint16 LE) + JPEG bytes
#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>

const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASSWORD";
const char* HOST_IP   = "10.42.22.1";   // laptop IP (hostname -I)
const uint16_t HOST_PORT = 5005;

#define CHUNK 1200

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

WiFiUDP udp;
uint16_t frameId = 0;

void setup() {
  Serial.begin(115200);

  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;   c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size   = FRAMESIZE_QVGA;   // 320x240; FRAMESIZE_VGA if the link is good
  c.jpeg_quality = 12;               // higher number = smaller/worse
  c.fb_count     = 2;
  c.grab_mode    = CAMERA_GRAB_LATEST;

  if (esp_camera_init(&c) != ESP_OK) {
    Serial.println("Camera init failed");
    while (true) delay(1000);
  }

  // Camera is mounted upside down: rotate the image 180 degrees.
  // (If the picture is upright but mirrored, remove the set_hmirror line.)
  sensor_t* sen = esp_camera_sensor_get();
  sen->set_vflip(sen, 1);
  sen->set_hmirror(sen, 1);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) { delay(300); Serial.print("."); }
  Serial.println(WiFi.localIP());
}

void loop() {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return;

  uint16_t total = (fb->len + CHUNK - 1) / CHUNK;
  for (uint16_t i = 0; i < total; i++) {
    size_t off = (size_t)i * CHUNK;
    size_t n = min((size_t)CHUNK, fb->len - off);

    uint8_t hdr[6] = {
      (uint8_t)(frameId & 0xFF), (uint8_t)(frameId >> 8),
      (uint8_t)(i & 0xFF),       (uint8_t)(i >> 8),
      (uint8_t)(total & 0xFF),   (uint8_t)(total >> 8)
    };
    udp.beginPacket(HOST_IP, HOST_PORT);
    udp.write(hdr, 6);
    udp.write(fb->buf + off, n);
    udp.endPacket();
    if ((i & 3) == 3) delay(1);
  }
  frameId++;
  esp_camera_fb_return(fb);
  delay(30);
}
