#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <WiFi.h>
#include "esp_timer.h"
#include "esp_http_server.h"

#define CAMERA_MODEL_XIAO_ESP32S3
#include "camera_pins.h"

// Wi-Fi Config
const char* ssid = "ESP32-farhan";
const char* password = "keepguessing";
IPAddress local_IP(192,168,4,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,0,0);

// Stream HTTP definitions
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
httpd_handle_t stream_httpd = NULL;

// Global vars
unsigned long lastCaptureTime = 0;
unsigned long startTime = 0;
int imageCount = 1;
bool camera_sign = false;
bool sd_sign = false;

// Stream handler
static esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char * part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK) return res;

  while(true){
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      _jpg_buf_len = fb->len;
      _jpg_buf = fb->buf;
    }

    if(res == ESP_OK){
      size_t hlen = snprintf((char *)part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    esp_camera_fb_return(fb);

    if(res != ESP_OK) break;
    delay(2000);
  }

  return res;
}

// Start server
void startCameraServer(){
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = stream_handler,
    .user_ctx  = NULL
  };

  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &index_uri);
    Serial.println("Camera stream ready at http://192.168.4.1");
  }
}

// Write file to SD
void writeFile(fs::FS &fs, const char * path, uint8_t * data, size_t len){
    Serial.printf("Writing file: %s\r\n", path);
    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.write(data, len) == len){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

// Save photo to SD
void photo_save(const char * fileName) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Failed to get camera frame buffer");
    return;
  }
  writeFile(SD, fileName, fb->buf, fb->len);
  esp_camera_fb_return(fb);
  Serial.println("Photo saved to file");
}

// Setup
void setup() {
  Serial.begin(115200);
  //while(!Serial); // wait for serial to open

  Serial.println("Waiting for power to stabilize...");
  delay(5000);  // ✅ ADD THIS DELAY EARLY — before initializing peripherals

  // Configure camera
  camera_config_t config;
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_VGA;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if(config.pixel_format == PIXFORMAT_JPEG){
    if(psramFound()){
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    config.frame_size = FRAMESIZE_240X240;
  }

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    return;
  }
  camera_sign = true;

  // Initialize SD card
  if(!SD.begin(21)){
    Serial.println("Card Mount Failed");
    return;
  }
  if(SD.cardType() == CARD_NONE){
    Serial.println("No SD card attached");
    return;
  }
  sd_sign = true;

  Serial.println("Photos will begin in 2 seconds...");

  //Start Wi-Fi
  WiFi.softAP(ssid, password);
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("Failed to configure static IP");
  }


  startCameraServer();

  // Mark start time
  startTime = millis();
}

// Main loop
void loop() {
  if(camera_sign && sd_sign){
    unsigned long now = millis();

    if ((now - lastCaptureTime) >= 4000) {
      char filename[32];
      sprintf(filename, "/image%d.jpg", imageCount);
      photo_save(filename);

      if (now - startTime <= 180000) {
        Serial.printf("[Streaming + Saving] Saved picture: %s\n", filename);
      } else {
        Serial.printf("[Saving Only] Saved picture: %s\n", filename);
      }

      imageCount++;
      lastCaptureTime = now;
    }
  }
}
