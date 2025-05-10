#include <vl53l8cx.h>
#include <vl53l8cx_api.h>

#include <NTPClient.h>
#include <WiFiUdp.h>

#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <WiFi.h>
#include <time.h>
#include "esp_timer.h"
#include "esp_http_server.h"
#include <sstream>
#include <string>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>



// VL53L8CX Definitions

#ifdef ARDUINO_SAM_DUE
#define DEV_I2C Wire1
#else
#define DEV_I2C Wire
#endif
#define SerialPort Serial

#define LPN_PIN A3
#define PWREN_PIN 11


// +---------------+                                +---------------+
// |  BLE Client   |                                |  BLE Server   |
// | (e.g. ESP32S3)|                                | (e.g. MG24)|
// +---------------+                                +---------------+
//         |                                                  |
//         |--- Scan for Advertisements --------------------->|         ***> Implement this here
//         |                                                  |
//         |<--- Advertise Service UUID, Name ----------------|
//         |                                                  |
//         |--- Connect Request ----------------------------->|         ***> Implement this here
//         |                                                  |
//         |<-- Connection Established (unencrypted) ---------|
//         |                                                  | ________________
//         |--- Initiate Pairing ---------------------------->| ***> Implement  \
//         |                                                  |       here*      \
//         |<-- Pairing Request / Confirm / Encrypt ----------|                   \    *Not needed for BLE Two-Way
//         |                                                  |                   /   Communication — Without Pairing*
//         | (Optional: Bonding to save keys)                 |                  /
//         |                                                  | ________________/
//         |--- Discover Services --------------------------->|         ***> Implement this here
//         |--- Discover Characteristics -------------------->|         ***> Implement this here
//         |                                                  |
//         |--- Write to Characteristic (TX) ---------------->|         ***> Implement this here
//         |                                                  |
//         |<-- Notify Characteristic Value (RX) -------------|
//         |                                                  |
//         | (Two-way data exchange continues securely)       |

// Notify: https://github.com/espressif/arduino-esp32/blob/15e71a6afd21f9723a0777fa2f38d4c647279933/libraries/BLE/examples/Notify/Notify.ino
// BLuetooth Definitions

// Bluetooth Device Name
const char *BLUETOOTH_DEVICE_NAME = "Obstacle-Detector";
BLEScan *pBLEScan;

// Service UUID
static BLEUUID GENERAL_SERVICE_UUID("abb8feff-68ce-4bf2-aef7-713ca4a82c4b");
// Device Name UUID
static BLEUUID DEVICE_NAME_CHARACTERISTIC_UUID("6f2173d8-7cd3-4511-b3fa-e77f4afc3404");
// IMU Service UUID
static BLEUUID IMU_DATA_SERVICE_UUID("64d0457d-7976-421e-acb5-ed881a36418d");
// IMU Data request UUID
static BLEUUID IMU_DATA_CHARACTERISTIC_UUID("6137df56-b267-46bb-b86c-f4614a9f67a6");
// Haptic Service UUID
static BLEUUID HAPTIC_SERVICE_UUID("d48330a4-ad02-40fb-9e1c-d2cc5ebff234");
// Haptic Status
static BLEUUID HAPTIC_STATUS_CHARACTERISTIC_UUID("3506f523-e28b-4d9c-a4a3-648e1a943ee0");
// Haptic feedback data sent to MG24s
static BLEUUID HAPTIC_FEEDBACK_CHARACTERISTIC_UUID("ff422d26-d740-47a7-a364-2a7aed5ab23d");

static BLEUUID CCCD_UUID((uint16_t)0x2902);

#define A_1 "64d0457d-7976-421e-acb5-ed881a36418d"
#define A_2 "4fce1ae4-217a-469d-83d2-92c87d45f7c9"
#define A_3 "ab63ab1f-a7c1-4740-95d3-0a47150e4289"





static boolean readyToConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic *pDeviceNameCharacteristic;
static BLERemoteCharacteristic *pIMUdataCharacteristic;
static BLERemoteCharacteristic *pHapticFeedbackCharacteristic;
static BLERemoteCharacteristic *pHapticStatusCharacteristic;


static BLEAdvertisedDevice *myDevice;

// static BLEAdvertisedDevice *sLeftHand;
// static BLEAdvertisedDevice *sRightHand;
// static BLEAdvertisedDevice *sLeftLeg;
// static BLEAdvertisedDevice *sRightLeg;


/**
  Client Array for holding all four clients
  Use the following line for holding multiple clients in a vector
  clients.push_back(BLEDevice::createClient());
*/
std::vector<BLEClient*> clients;

BLEClient *pClientLeftHand;
BLEClient *pClientRightHand;
BLEClient *pClientLeftLeg;
BLEClient *pClientRightLeg;

int scanTime = 5;  //In seconds

// VL53L8CX Configurations
VL53L8CX tof_sensor(&DEV_I2C, LPN_PIN);

bool EnableAmbient = true;
bool EnableSignal = true;
uint8_t res = VL53L8CX_RESOLUTION_4X4;
std::stringstream toF8x8SensorDataStringSteam;
uint8_t status;

// Wi-Fi Config
const char *ssid = "Mehmaan";
const char *password = "AshenB0ish@Jan";

// NTP settings
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -6 * 3600;  // GMT offset in seconds (e.g., 0 for UTC)
const long daylightOffset_sec = 3600;  // DST offset in seconds (3600 for +1hr)

WiFiUDP ntpUDP;

// You can specify the time server pool and the offset (in seconds, can be
// changed later with setTimeOffset() ). Additionally you can specify the
// update interval (in milliseconds, can be changed using setUpdateInterval() ).
NTPClient timeClient(ntpUDP, ntpServer, gmtOffset_sec + daylightOffset_sec, 60000);

// Global vars
unsigned long lastCaptureTime = 0;
unsigned long startTime = 0;
int imageCount = 1;
bool camera_sign = false;
bool sd_sign = false;

String timeString = "";
const int DATA_CAPTURES_PER_IMAGE = 5;

// Last read IMU sensor data to check with current read IMU data
std::string lastReadimuData = "";

bool writeToF8x8SensorDataSuccess = false;


// Setup
void setup() {
  Serial.begin(115200);
  //while(!Serial); // wait for serial to open

  Serial.println("Waiting for power to stabilize...");
  delay(5000);  // ADD THIS DELAY EARLY — before initializing peripherals

  // Initialize SD card
  if (!SD.begin(21)) {
    Serial.println("Card Mount Failed");
    return;
  }
  if (SD.cardType() == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  sensorSetup();
  wifiAndTimeSetup();
  runBluetoothScan();
}

// Main loop
void loop() {
  // String timeString = getLocalTimeString();
  timeClient.update();
  timeString = timeClient.getFormattedTime();

  Serial.println(timeString);

  attemptBLEConnection();

  int8_t i;
  for (i = 0; i < DATA_CAPTURES_PER_IMAGE; i++) {
    sensorLoop();
  }
  writeToF8x8SensorDataInFile();

  // Clear contents
  toF8x8SensorDataStringSteam.str("");

  delay(1000);
}

/**
 Wifi and Time Setup
*/
void wifiAndTimeSetup() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println(wl_status_to_string(WiFi.status()));
  }
  Serial.println(" Connected!");

  timeClient.begin();
}

// VL53L8CX Sensor Setup
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



// VL53L8CX Sensor Loop code
void sensorLoop() {
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
}

// VL53L8CX Data Save on SD
void saveSensorData(VL53L8CX_ResultsData *Result) {
  int8_t i, j, k, l;
  uint8_t zones_per_line;
  uint8_t number_of_zones = res;

  zones_per_line = (number_of_zones == 16) ? 4 : 8;

  //SerialPort.print("\n\n");
  //snprintf(report, sizeof(report), "%s,",timeString);
  //offset += snprintf(report + offset, sizeof(report) - offset,  "%s,",timeString);
  //Serial.println(timeString);
  toF8x8SensorDataStringSteam << timeString.c_str() << ",";
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
          toF8x8SensorDataStringSteam << distance << "," << status << "," << signal << "," << ambient << ",";
        } else {
          toF8x8SensorDataStringSteam << "X"
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
  toF8x8SensorDataStringSteam << "\n";
  // Convert the stream buffer into a string
  std::string str = toF8x8SensorDataStringSteam.str();
  //SerialPort.print(str.c_str());
  //SerialPort.print("\n");
}



void writeIMUSensorDataInFile() {
  if (writeToF8x8SensorDataSuccess) {
    writeSensorDataInFile(lastReadimuData, "/imuSensorDatafile.txt");
  }
}

void writeToF8x8SensorDataInFile() {
  writeSensorDataInFile(toF8x8SensorDataStringSteam.str(), "/tofSensorDatafile.txt");
}

void writeSensorDataInFile(std::string sensorData, String filename) {
  Serial.println("sensorData");
  File tofSensorfile = SD.open(filename, FILE_APPEND);
  if (!tofSensorfile) {
    Serial.println("Failed to open ToF sensor datafile file for writing sensor data!");
    writeToF8x8SensorDataSuccess = false;
    return;
  }
  if (tofSensorfile.print(String(sensorData.c_str()))) {
    Serial.println("Sensor data written in file!");
  } else {
    writeToF8x8SensorDataSuccess = false;
    Serial.println("Sensor data write failed!");
  }
  tofSensorfile.close();
  writeToF8x8SensorDataSuccess = true;
}

const char *wl_status_to_string(wl_status_t status) {
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

//                                                                                  BLUETOOTH code

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(GENERAL_SERVICE_UUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      readyToConnect = true;
      doScan = true;

    }  // Found our server
  }    // onResult
};

// Run in setup
void runBluetoothScan() {
  Serial.println("Starting BLE scan!");

  BLEDevice::init(BLUETOOTH_DEVICE_NAME);

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  pBLEScan = BLEDevice::getScan();  //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(449);       // less or equal setInterval value
  pBLEScan->setActiveScan(true);  //active scan uses more power, but get results faster
  pBLEScan->start(5, false);
}

// IMU data Chang notification
static void imuDataChangeNotifyCallback(BLERemoteCharacteristic *pIMUdataCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
  Serial.print("IMU Data Change Notification Callback ");
  readIMUdataFromCharacteristic();
}

// We can get the haptic execution status update here
static void hapticExecutionStatusNotifyCallback(BLERemoteCharacteristic *pHapticStatusCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
  Serial.print("Haptic Execution Status: ");
  Serial.println(pHapticStatusCharacteristic->getUUID().toString().c_str());
}


class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient *pclient) {
    Serial.println("Connected");
  }

  void onDisconnect(BLEClient *pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

bool connectToServer() {
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  BLEClient *pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remove BLE Server.
  pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  Serial.println(" - Connected to server");
  pClient->setMTU(247);  //set client to request maximum MTU from server (default is 23 otherwise)

  // Obtain a reference to the General service we are after in the remote BLE server.
  BLERemoteService *pRemoteService = pClient->getService(GENERAL_SERVICE_UUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(GENERAL_SERVICE_UUID.toString());
    pClient->disconnect();
    return false;
  } else {
    Serial.println("GENERAL_SERVICE_UUID - Found our service");
  }

  pDeviceNameCharacteristic = pRemoteService->getCharacteristic(DEVICE_NAME_CHARACTERISTIC_UUID);
  if (pHapticFeedbackCharacteristic == nullptr) {
    Serial.println("Failed to find our characteristic UUID: DEVICE_NAME_CHARACTERISTIC_UUID");
    pClient->disconnect();
    return false;
  } else {
    Serial.println("DEVICE_NAME_CHARACTERISTIC_UUID - Found");
  }

  // Obtain a reference to the IMU DATA Service we are after in the remote BLE server.
  BLERemoteService *pIMUDataService = pClient->getService(IMU_DATA_SERVICE_UUID);
  if (pIMUDataService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(IMU_DATA_SERVICE_UUID.toString());
    pClient->disconnect();
    return false;
  } else {
    Serial.println("IMU_DATA_SERVICE_UUID - Found our service");
  }

  // Obtain a reference to the characteristic in the IMU Data service of the remote BLE server.
  pIMUdataCharacteristic = pIMUDataService->getCharacteristic(IMU_DATA_CHARACTERISTIC_UUID);
  if (pIMUdataCharacteristic == nullptr) {
    Serial.println("Failed to find our characteristic UUID: IMU_DATA_CHARACTERISTIC_UUID");
    pClient->disconnect();
    return false;
  }
  Serial.println("IMU_DATA_CHARACTERISTIC_UUID - Found");

  // Enable notification of IMU Data Change so that we can read IMU data
  if (pIMUdataCharacteristic->canNotify()) {
      pIMUdataCharacteristic->registerForNotify(imuDataChangeNotifyCallback);
      // Write 0x02 0x00 to CCCD to enable indication
      uint8_t notificationOn[] = {0x02, 0x00};
      pIMUdataCharacteristic->getDescriptor(CCCD_UUID)->writeValue(notificationOn, 2, true);
      Serial.println("Notification enabled for IMU Data Change");
    }

  // Obtain a reference to the Haptic Data and Feedback Service we are after in the remote BLE server.
  BLERemoteService *pHapticDataFeedbackService = pClient->getService(HAPTIC_SERVICE_UUID);
  if (pIMUDataService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(HAPTIC_SERVICE_UUID.toString());
    pClient->disconnect();
    return false;
  } else {
    Serial.println("HAPTIC_SERVICE_UUID - Found our service");
  }


  // Obtain a reference to the characteristic in the Haptic Feedback service of the remote BLE server.
  pHapticFeedbackCharacteristic = pRemoteService->getCharacteristic(HAPTIC_FEEDBACK_CHARACTERISTIC_UUID);
  if (pHapticFeedbackCharacteristic == nullptr) {
    Serial.println("Failed to find our characteristic UUID: HAPTIC_FEEDBACK_CHARACTERISTIC_UUID");
    pClient->disconnect();
    return false;
  }
  Serial.println("HAPTIC_FEEDBACK_CHARACTERISTIC_UUID - Found");


  // Obtain a reference to the characteristic in the Haptic Status service of the remote BLE server.
  pHapticStatusCharacteristic = pRemoteService->getCharacteristic(HAPTIC_STATUS_CHARACTERISTIC_UUID);
  if (pHapticStatusCharacteristic == nullptr) {
    Serial.println("Failed to find our characteristic UUID: HAPTIC_STATUS_CHARACTERISTIC_UUID");
    pClient->disconnect();
    return false;
  }
  Serial.println("HAPTIC_STATUS_CHARACTERISTIC_UUID - Found");

  connected = true;
  return true;
}

void readIMUdataFromCharacteristic() {
  // Read the IMU Sensor Data value of the characteristic.
  if (pIMUdataCharacteristic->canRead()) {
    std::string imuData = std::string(pIMUdataCharacteristic->readValue().c_str());
    if (imuData != lastReadimuData) {
      lastReadimuData = imuData;
      size_t numFloats = imuData.length() / sizeof(float);  // should be 714
      float *imu_floats = (float *)imuData.data();             // reinterpret bytes
      for (int i = 0; i < numFloats / 6; i++) {
        float ax = imu_floats[i * 6 + 0];
        float ay = imu_floats[i * 6 + 1];
        float az = imu_floats[i * 6 + 2];
        float gx = imu_floats[i * 6 + 3];
        float gy = imu_floats[i * 6 + 4];
        float gz = imu_floats[i * 6 + 5];
        Serial.printf("IMU Sensor Data from MG24 Sense %d:\n  Accel:  X=%.2f  Y=%.2f  Z=%.2f\n  Gyro:   X=%.2f  Y=%.2f  Z=%.2f\n", i+1, ax, ay, az, gx, gy, gz);
      }
    }
  }
}

void sendHapticMotorPattern(uint8_t vibrationPattern) {
  // Write the Haptic Motor value of the characteristic.
  if (pHapticFeedbackCharacteristic->canWrite()) {
    pHapticFeedbackCharacteristic->writeValue((uint8_t *)vibrationPattern, sizeof(vibrationPattern), true);
  }
}

void getHapticExecutionStatusNotification() {
  // Set notification for Hatpic Motor execution Notification
  if (pHapticStatusCharacteristic->canNotify()) {
    pHapticStatusCharacteristic->registerForNotify(hapticExecutionStatusNotifyCallback);
  }
}


// This is the Arduino main loop function.
void attemptBLEConnection() {

  // If the flag "readyToConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  if (readyToConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothing more we will do.");
    }
    readyToConnect = false;
  }

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected) {
    String newValue = "Time since boot: " + String(millis() / 1000);
    Serial.println("Setting new characteristic value to \"" + newValue + "\"");

    // Set the characteristic's value to be the array of bytes that is actually a string.
    lastReadimuData = std::string(pIMUdataCharacteristic->readValue().c_str());
  } else if (doScan) {
    BLEDevice::getScan()->start(0);  // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
  }

  delay(1000);  // Delay a second between loops.
}  // End of loop
