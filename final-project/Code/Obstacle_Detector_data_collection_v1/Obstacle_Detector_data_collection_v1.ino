#include <vl53l8cx.h>
#include <vl53l8cx_api.h>

#include <NTPClient.h>
#include <WiFiUdp.h>

#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <WiFi.h>
#include <time.h>
#include "esp_timer.h"
#include "esp_http_server.h"
#include <sstream>
#include <string>


#define CAMERA_MODEL_XIAO_ESP32S3
#include "camera_pins.h"

// VL53L8CX Definitions

#ifdef ARDUINO_SAM_DUE
#define DEV_I2C Wire1
#else
#define DEV_I2C Wire
#endif
#define SerialPort Serial

#define LPN_PIN A3
#define PWREN_PIN 11

// VL53L8CX Configurations
VL53L8CX tof_sensor(&DEV_I2C, LPN_PIN);

bool EnableAmbient = true;
bool EnableSignal = true;
uint8_t res = VL53L8CX_RESOLUTION_4X4;
std::stringstream sensorDataString;
uint8_t status;

// Wi-Fi Config
const char* ssid = "Mehmaan";
const char* password = "AshenB0ish@Jan";

// NTP settings
const char* googleNtpServer = "time.google.com";
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -6 * 3600;  // GMT offset in seconds (e.g., 0 for UTC)
const long daylightOffset_sec = 3600;  // DST offset in seconds (3600 for +1hr)

WiFiUDP ntpUDP;

// You can specify the time server pool and the offset (in seconds, can be
// changed later with setTimeOffset() ). Additionally you can specify the
// update interval (in milliseconds, can be changed using setUpdateInterval() ).
NTPClient timeClient(ntpUDP, ntpServer, gmtOffset_sec, 60000);

// Global vars
unsigned long lastCaptureTime = 0;
unsigned long startTime = 0;
int imageCount = 1;
bool camera_sign = false;
bool hasSDcard = false;

String timeString = "";
const int DATA_CAPTURES_PER_IMAGE = 5;



// Setup
void setup() {
  Serial.begin(115200);
  //while(!Serial); // wait for serial to open

  Serial.println("Waiting for power to stabilize...");
  delay(5000);  // ADD THIS DELAY EARLY — before initializing peripherals

  cameraSetup();
  sensorSetup();
  timeSetup();
}

const char* wl_status_to_string(wl_status_t status) {
  switch (status) {
    case WL_NO_SHIELD: return "WL_NO_SHIELD";
    case WL_IDLE_STATUS: return "WL_IDLE_STATUS";
    case WL_NO_SSID_AVAIL: return "WL_NO_SSID_AVAIL";
    case WL_SCAN_COMPLETED: return "WL_SCAN_COMPLETED";
    case WL_CONNECTED: return "WL_CONNECTED";
    case WL_CONNECT_FAILED: return "WL_CONNECT_FAILED";
    case WL_CONNECTION_LOST: return "WL_CONNECTION_LOST";
    case WL_DISCONNECTED: return "WL_DISCONNECTED";
  }
}

void timeSetup() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(wl_status_to_string(WiFi.status()));
  }
  Serial.println(" Connected!");

  timeClient.begin();
}

// Setup
void cameraSetup() {
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

  if (config.pixel_format == PIXFORMAT_JPEG) {
    if (psramFound()) {
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

  Serial.println("Photos will begin in 2 seconds...");
  startTime = millis();
}

void sensorSetup() {

  // Enable PWREN pin if present
  if (PWREN_PIN >= 0) {
    pinMode(PWREN_PIN, OUTPUT);
    digitalWrite(PWREN_PIN, HIGH);
    delay(10);
  }

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Configure VL53L8CX component.
  tof_sensor.begin();
  status = tof_sensor.init();

  // Start Measurements
  status = tof_sensor.start_ranging();
}

// Main loop
void loop() {
  // String timeString = getLocalTimeString();
  timeClient.update();
  timeString = timeClient.getFormattedTime();

  Serial.println(timeString);

  // Initialize SD card
  if (!SD.begin(21)) {
    Serial.println("Card Mount Failed");
    return;
  }
  if (SD.cardType() == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
  hasSDcard = true;

  cameraLoop();
  int8_t i;
  for (i = 0; i < DATA_CAPTURES_PER_IMAGE; i++) {
    sensorLoop();
  }
  writeSensorDataInFile();

  // Clear contents
  sensorDataString.str("");
  //sensorDataString.clear();

  delay(1000);
}

// Main loop
void cameraLoop() {
  if (camera_sign && hasSDcard) {
    unsigned long now = millis();

    if ((now - lastCaptureTime) >= 4000) {
      char filename[32];
      sprintf(filename, "/image-%s.jpg", timeString);
      photo_save(filename);

      if (now - startTime <= 180000) {
        Serial.printf("[Streaming + Saving] Saved picture: %s\n", filename);
      } else {
        Serial.printf("[Saving Only] Saved picture: %s\n", filename);
      }

      imageCount++;
      lastCaptureTime = now;
    }
  } else {
    Serial.printf("Insert SD card to record sensor Data!");
  }
}

void sensorLoop() {
  if (hasSDcard) {
    VL53L8CX_ResultsData Results;
    uint8_t NewDataReady = 0;

    do {
      status = tof_sensor.check_data_ready(&NewDataReady);
    } while (!NewDataReady);

    if ((!status) && (NewDataReady != 0)) {
      status = tof_sensor.get_ranging_data(&Results);
      saveSensorData(&Results);
    }
    delay(100);
  } else {
    Serial.printf("Insert SD card to record sensor Data!");
  }
}

void saveSensorData(VL53L8CX_ResultsData* Result) {
  int8_t i, j, k, l;
  uint8_t zones_per_line;
  uint8_t number_of_zones = res;

  zones_per_line = (number_of_zones == 16) ? 4 : 8;

  //SerialPort.print("\n\n");
  //snprintf(report, sizeof(report), "%s,",timeString);
  //offset += snprintf(report + offset, sizeof(report) - offset,  "%s,",timeString);
  //Serial.println(timeString);
  sensorDataString << timeString.c_str() << ",";
  //SerialPort.print(timeString);
  for (j = 0; j < number_of_zones; j += zones_per_line) {
    for (l = 0; l < VL53L8CX_NB_TARGET_PER_ZONE; l++) {
      // Print distance and status
      for (k = (zones_per_line - 1); k >= 0; k--) {
        if (Result->nb_target_detected[j + k] > 0) {
          int zoneIndex = j + k;
          int dataIndex = (VL53L8CX_NB_TARGET_PER_ZONE * zoneIndex) + l;
          long signal = (long)Result->signal_per_spad[dataIndex];
          long ambient = (long)Result->ambient_per_spad[zoneIndex];
          long distance = (long)Result->distance_mm[dataIndex];
          long status = (long)Result->target_status[dataIndex];
          sensorDataString << distance << "," << status << "," << signal << "," << ambient << ",";
        } else {
          sensorDataString << "X"
                           << ","
                           << "X"
                           << ","
                           << "X"
                           << ","
                           << "X"
                           << ",";
        }
      }
    }
  }
  sensorDataString << "\n";
  // Convert the stream buffer into a string
  std::string str = sensorDataString.str();
  //SerialPort.print(str.c_str());
  //SerialPort.print("\n");
}

// Write file to SD
void writeFile(fs::FS& fs, const char* path, uint8_t* data, size_t len) {
  Serial.printf("Writing file: %s\r\n", path);
  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.write(data, len) == len) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void writeSensorDataInFile() {
  String str = String(sensorDataString.str().c_str());
  Serial.println("writeSensorDataInFile");
  Serial.println(str);
  File file = SD.open("/datafile.txt", FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open datafile file for writing sensor data!");
    return;
  }
  if (file.print(str)) {
    Serial.println("Sensor data written in file!");
  } else {
    Serial.println("Sensor data write failed!");
  }
  file.close();
}

// Save photo to SD
void photo_save(const char* fileName) {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Failed to get camera frame buffer");
    return;
  }
  writeFile(SD, fileName, fb->buf, fb->len);
  esp_camera_fb_return(fb);
  Serial.println("Photo saved to file");
}
