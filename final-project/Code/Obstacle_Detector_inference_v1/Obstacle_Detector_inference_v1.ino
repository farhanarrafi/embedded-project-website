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
const char* BLUETOOTH_DEVICE_NAME = "Obstacle-Detector";
BLEScan* pBLEScan;

// Service UUID
#define SERVICE_UUID  "abb8feff-68ce-4bf2-aef7-713ca4a82c4b"
// IMU Data request UUID
#define IMU_DATA_UUID "6137df56-b267-46bb-b86c-f4614a9f67a6"
// Haptic Status
#define HAPTIC_STATUS_UUID "3506f523-e28b-4d9c-a4a3-648e1a943ee0"
// Haptic feedback data sent to MG24s
#define HAPTIC_FEEDBACK_UUID "ff422d26-d740-47a7-a364-2a7aed5ab23d"

// BLEServer *bleServer = NULL;
// BLEService *bleService = NULL;
// BLECharacteristic *bleCharacteristic = NULL;
// BLE2901 *descriptor_2901 = NULL;


static boolean readyToConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic *pIMUdataCharacteristic;
static BLERemoteCharacteristic *pHapticFeedbackCharacteristic;
static BLERemoteCharacteristic *pHapticStatusCharacteristic;
static BLEAdvertisedDevice *myDevice;

int scanTime = 5;  //In seconds

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
const char* ntpServer = "pool.ntp.org";
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



// IMU sensor data
String imuSensorData = "";


// Setup
void setup() {
  Serial.begin(115200);
  //while(!Serial); // wait for serial to open

  Serial.println("Waiting for power to stabilize...");
  delay(5000);  // ADD THIS DELAY EARLY — before initializing peripherals

  // Initialize SD card
  if(!SD.begin(21)){
    Serial.println("Card Mount Failed");
    return;
  }
  if(SD.cardType() == CARD_NONE){
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
  writeSensorDataInFile();

  // Clear contents
  sensorDataString.str("");

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
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(SERVICE_UUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      readyToConnect = true;
      doScan = true;

    }  // Found our server
  }  // onResult
}; 

// Run in setup
void runBluetoothScan() {
  Serial.println("Starting BLE scan!");

  BLEDevice::init(BLUETOOTH_DEVICE_NAME);

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(449); // less or equal setInterval value
  pBLEScan->setActiveScan(true);  //active scan uses more power, but get results faster
  pBLEScan->start(5, false);
}

// IMU data recevied here from MG24 Sense
static void imuDataNotifyCallback(BLERemoteCharacteristic *pIMUdataCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
  Serial.print("IMU Data receipt Notify Callback ");
  Serial.print(pIMUdataCharacteristic->getUUID().toString().c_str());
  Serial.print(" of data length ");
  Serial.println(length);
  Serial.print("data: ");
  Serial.write(pData, length);
  Serial.println();
}

// We can get the haptic execution status update here
static void hapticExecutionStatusNotifyCallback(BLERemoteCharacteristic *pHapticStatusCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
  Serial.print("Haptic Execution Status Callback ");
  Serial.print(pHapticStatusCharacteristic->getUUID().toString().c_str());
  Serial.print(" of data length ");
  Serial.println(length);
  Serial.print("data: ");
  Serial.write(pData, length);
  Serial.println();
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

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService *pRemoteService = pClient->getService(BLEUUID(SERVICE_UUID));
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(SERVICE_UUID);
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

  // Obtain a reference to the characteristic in the IMU Data service of the remote BLE server.
  pIMUdataCharacteristic = pRemoteService->getCharacteristic(IMU_DATA_UUID);
  if (pIMUdataCharacteristic == nullptr) {
    Serial.println("Failed to find our characteristic UUID: IMU_DATA_UUID");
    pClient->disconnect();
    return false;
  }
  Serial.println("IMU_DATA - Found our characteristic");

  // Obtain a reference to the characteristic in the Haptic Feedback service of the remote BLE server.
  pHapticFeedbackCharacteristic = pRemoteService->getCharacteristic(HAPTIC_FEEDBACK_UUID);
  if (pHapticFeedbackCharacteristic == nullptr) {
    Serial.println("Failed to find our characteristic UUID: HAPTIC_FEEDBACK_UUID");
    pClient->disconnect();
    return false;
  }
  Serial.println("HAPTIC_FEEDBACK - Found our characteristic");


  // Obtain a reference to the characteristic in the Haptic Status service of the remote BLE server.
  pHapticStatusCharacteristic = pRemoteService->getCharacteristic(HAPTIC_STATUS_UUID);
  if (pHapticStatusCharacteristic == nullptr) {
    Serial.println("Failed to find our characteristic UUID: HAPTIC_STATUS_UUID");
    pClient->disconnect();
    return false;
  }
  Serial.println("HAPTIC_STATUS - Found our characteristic");


  // Read the IMU Sensor Data value of the characteristic.
  if (pIMUdataCharacteristic->canRead()) {
    std::string imuRaw = pIMUdataCharacteristic->readValue();
    imuSensorData = String(imuRaw.c_str());
    Serial.print("IMU Sensor Data from MG24 Sense: " + imuSensorData);
  }

  // Write the Haptic Motor value of the characteristic.
  int motorValues[] = {25, 50, 75, 100};
  if (pHapticFeedbackCharacteristic->canWrite()) {
    pHapticFeedbackCharacteristic->writeValue((uint8_t*)motorValues, sizeof(motorValues), true);
  }

  // Set notification for Hatpic Motor execution Notification
  if (pHapticStatusCharacteristic->canNotify()) {
    pHapticStatusCharacteristic->registerForNotify(hapticExecutionStatusNotifyCallback);
  }

  // Register for IMU data notification (push) from MG24 Sense
  if (pIMUdataCharacteristic->canIndicate() || pIMUdataCharacteristic->canNotify()) {
    pIMUdataCharacteristic->registerForNotify(imuDataNotifyCallback);
  }

  connected = true;
  return true;
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
    imuSensorData= pIMUdataCharacteristic->readValue();
  } else if (doScan) {
    BLEDevice::getScan()->start(0);  // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
  }

  delay(1000);  // Delay a second between loops.
}  // End of loop

