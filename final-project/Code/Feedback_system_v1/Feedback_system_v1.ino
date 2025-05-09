#include "SPI.h"
#include <time.h>
#include <sstream>
#include <string>
#include <Wire.h>
#include <LSM6DS3.h>

#include "Adafruit_DRV2605.h"

//                                        READ for Security (Bonding and Encryption)
// https://github.com/SiliconLabs/bluetooth_stack_features/tree/master/security/authenticating_devices_with_no_user_interface


// +---------------+                                +---------------+
// |  BLE Client   |                                |  BLE Server   |
// | (e.g. ESP32S3)|                                | (e.g. MG24)   |
// +---------------+                                +---------------+
//         |                                                  |
//         |--- Scan for Advertisements --------------------->|
//         |                                                  |
//         |<--- Advertise Service UUID, Name ----------------|          ***> Implement this here
//         |                                                  |
//         |--- Connect Request ----------------------------->|
//         |                                                  |
//         |<-- Connection Established (unencrypted) ---------|          ***> Implement this here
//         |                                                  | ________________
//         |--- Initiate Pairing ---------------------------->|                 \
//         |                                                  |                  \
//         |<-- Pairing Request / Confirm / Encrypt ----------|***> Implement     \    *Not needed for BLE Two-Way
//         |                                                  |     here          /   Communication — Without Pairing
//         | (Optional: Bonding to save keys)                 |                  /
//         |                                                  | ________________/
//         |--- Discover Services --------------------------->|
//         |--- Discover Characteristics -------------------->|
//         |                                                  |
//         |--- Write to Characteristic (TX) ---------------->|
//         |                                                  |
//         |<-- Notify Characteristic Value (RX) -------------|         ***> Implement this here
//         |                                                  |
//         | (Two-way data exchange continues securely)       |


// Create a BLE server that, once we receive a connection, will send periodic notifications.
// The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
// Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE"
// Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"


  // SERVICE_UUID: abb8feff-68ce-4bf2-aef7-713ca4a82c4b
const uint8_t generic_access_service_uuid[] = {
  0x4b, 0x2c, 0xa8, 0xa4, 0x3c, 0x71, 0xf7, 0xae, 0xf2, 0x4b, 0xce, 0x68, 0xff, 0xfe, 0xb8, 0xab
};


static const uint8_t advertised_name[] = "FEEDBACK-SYSTEM";


static uint16_t gattdb_session_id;
static uint16_t generic_access_service_handle;
static uint16_t name_characteristic_handle;

// Sends IMU data to ESP32S3
static uint16_t imu_data_access_service_handle;
static uint16_t imu_data_characteristic_handle;
// Sends Haptic Status to ESP32S3 after driving haptic motors to acknowledge feedback to user
static uint16_t haptic_status_service_handle;
static uint16_t haptic_status_characteristic_handle;
// Receives Haptic Vibration pattern from ESP32S3
static uint16_t haptic_feedback_characteristic_handle;

static uint8_t connection_handle = SL_BT_INVALID_CONNECTION_HANDLE;


static void ble_initialize_gatt_db();
static void ble_start_advertising();

// Adafruit Motor Driver Setup
// https://www.adafruit.com/product/2305
// https://github.com/adafruit/Adafruit_DRV2605_Library
// http://www.adafruit.com/datasheets/DRV2605.pdf
Adafruit_DRV2605 drv;

// Turn this to true to vibrate motor
bool HAPTIC_VIBRATE_ON = false;

// Check the list for effects here: https://github.com/adafruit/Adafruit_DRV2605_Library/blob/master/examples/basic/basic.ino
// 17 − Strong Click 1 - 100%
int effect = 17;

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);  //I2C device address 0x6A
float aX, aY, aZ, gX, gY, gZ;
const float accelerationThreshold = 2.5;  // threshold of significant in G's
const int numSamples = 119;
int samplesRead = numSamples;
std::stringstream sensorDataString;

static void ble_initialize_gatt_db();
static void ble_start_advertising();

// Setup
void setup() {
  Serial.begin(115200);
  //while(!Serial); // wait for serial to open

  Serial.println("Waiting for power to stabilize...");
  delay(5000);  // ADD THIS DELAY EARLY — before initializing peripherals


  bluetoothSetup();
  timeSetup();
  imuSensorSetup();
  motorDriverSetup();
}



// Main loop
void loop() {
  // String timeString = getLocalTimeString();
  // timeClient.update();
  // timeString = timeClient.getFormattedTime();
  // Serial.print("Time: ");
  // Serial.println(timeString);

  if (HAPTIC_VIBRATE_ON) {
    //motorDriverLoop();
  }
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(1000);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(1000); 
  delay(100);
  bluetoothLoop();
}

void timeSetup() {
}

// Bluetooth Communication Setup
void bluetoothSetup() {
  // LED works as indicator for bluetooth
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW); 
}
// Motor Driver Setup
/*!
  @brief Select the waveform library to use.
  @param lib Library to use, 0 = Empty, 1-5 are ERM, 6 is LRA.

    See section 7.6.4 in the datasheet for more details:
  http://www.adafruit.com/datasheets/DRV2605.pdf
*/
void motorDriverSetup() {
  Serial.println("Adafruit DRV2605 Basic test");
  if (!drv.begin()) {
    Serial.println("Could not find DRV2605");
    while (1) delay(10);
  }

  // @param lib Library to use, 0 = Empty, 1-5 are ERM, 6 is LRA.
  drv.selectLibrary(1);

  // I2C trigger by sending 'go' command
  // default, internal trigger when sending GO command
  // 0: Internal trigger, call go() to start playback\n
  // 1: External trigger, rising edge on IN pin starts playback\n
  // 2: External trigger, playback follows the state of IN pin\n
  // 3: PWM/analog input\n
  // 4: Audio\n
  // 5: Real-time playback\n
  // 6: Diagnostics\n
  // 7: Auto calibration
  drv.setMode(DRV2605_MODE_INTTRIG);
}



void motorDriverLoop() {
  // set the effect to play
  drv.setWaveform(0, effect);  // play effect
  drv.setWaveform(1, 0);       // end waveform

  // play the effect!
  drv.go();

  // wait a bit
  delay(100);
}

void bluetoothLoop() {
}

void imuSensorSetup() {
  //Call .begin() to configure the IMUs
  if (myIMU.begin() != 0) {
    Serial.println("Device error");
  } else {
    Serial.println("aX,aY,aZ,gX,gY,gZ");
  }
}

void imuSensorLoop() {
  // wait for significant motion
  while (samplesRead == numSamples) {
    // read the acceleration data
    aX = myIMU.readFloatAccelX();
    aY = myIMU.readFloatAccelY();
    aZ = myIMU.readFloatAccelZ();

    // sum up the absolutes
    float aSum = fabs(aX) + fabs(aY) + fabs(aZ);

    // check if it's above the threshold
    if (aSum >= accelerationThreshold) {
      // reset the sample read count
      samplesRead = 0;
      break;
    }
  }
  // check if the all the required samples have been read since
  // the last time the significant motion was detected
  while (samplesRead < numSamples) {
    // check if both new acceleration and gyroscope data is
    // available
    // read the acceleration and gyroscope data
    samplesRead++;
    sensorDataString << myIMU.readFloatAccelX() << "," << myIMU.readFloatAccelY() << "," << myIMU.readFloatAccelZ() << ",";
    sensorDataString << myIMU.readFloatGyroX() << "," << myIMU.readFloatGyroY() << "," << myIMU.readFloatGyroZ() << "\n";
  }

  //const uint8_t* sensorData = reinterpret_cast<const uint8_t*>(sensorDataString.str().data());
  //Serial.print("Sensor Data:");
  //Serial.println(String(sensorDataString.str().c_str()));
  // // Update the local GATT with the current IMU Data
  // sl_bt_gatt_server_send_indication(connection_handle,
  //                                   imu_data_characteristic_handle,
  //                                   sizeof(sensorData),
  //                                   sensorData);
  std::string data_str = sensorDataString.str();  // Store in a named variable
  //Serial.print("Sensor Data:");
  Serial.println(data_str.c_str());

  if (connection_handle != SL_BT_INVALID_CONNECTION_HANDLE) {  // Only send if connected
    sl_status_t sc = sl_bt_gatt_server_send_indication(connection_handle,
                                                       imu_data_characteristic_handle,
                                                       data_str.length(),                  // <-- CORRECTED: Use actual data length
                                                       (const uint8_t *)data_str.data());  // data() is fine here as data_str is in scope
    if (sc != SL_STATUS_OK) {
      Serial.print("Error sending IMU indication: 0x");
      Serial.println(sc, HEX);
    }
  }
  Serial.println(data_str.c_str());
  // Clear contents
  sensorDataString.str("");
}



/**************************************************************************/ /**
 * Bluetooth stack event handler
 * Called when an event happens on BLE the stack
 *
 * @param[in] evt Event coming from the Bluetooth stack
 *****************************************************************************/
void sl_bt_on_event(sl_bt_msg_t *evt) {
  Serial.println("Inside sl_bt_on_event");
  switch (SL_BT_MSG_ID(evt->header)) {
    // -------------------------------
    // This event indicates the device has started and the radio is ready.
    // Do not call any stack command before receiving this boot event!
    case sl_bt_evt_system_boot_id:
      Serial.println("BLE stack booted");
      // Initialize the application specific GATT DB
      ble_initialize_gatt_db();
      // Start advertising
      ble_start_advertising();
      Serial.println("BLE advertisement started");
      break;

    // -------------------------------
    // This event indicates that a new connection was opened
    case sl_bt_evt_connection_opened_id:
      Serial.println("BLE connection opened");
      connection_handle = evt->data.evt_connection_opened.connection;
      Serial.println(connection_handle, HEX);
      digitalWrite(LED_BUILTIN, HIGH);
      break;

    // -------------------------------
    // This event indicates that a connection was closed
    case sl_bt_evt_connection_closed_id:
      Serial.println("BLE connection closed");
      Serial.print(connection_handle, HEX);
      Serial.print(" reason: 0x");
      Serial.println(evt->data.evt_connection_closed.reason, HEX);

      connection_handle = SL_BT_INVALID_CONNECTION_HANDLE;
      // Restart advertising
      ble_start_advertising();
      Serial.println("BLE advertisement restarted");
      digitalWrite(LED_BUILTIN, LOW);
      break;

    // -------------------------------
    // -------------------------------
    // This event indicates that the value of an attribute in the local GATT
    // database was changed by a remote GATT client
    case sl_bt_evt_gatt_server_attribute_value_id:
      // Check if the changed characteristic

      break;
    // -------------------------------
    // Indicates that a remote GATT client is attempting to write a value of
    // an attribute into the local GATT database,  where the attribute was defined in
    // the GATT database XML file to have the type="user"
    case sl_bt_evt_gatt_server_user_write_request_id:
      if (imu_data_characteristic_handle == evt->data.evt_gatt_server_attribute_value.attribute) {
        Serial.println("IMU Data characteristic data received");
        // Check the length of the received data
        if (evt->data.evt_gatt_server_attribute_value.value.len == 0) {
          break;
        }
        // Get the received byte
        uint8_t received_data = evt->data.evt_gatt_server_attribute_value.value.data[0];

      } else if (haptic_feedback_characteristic_handle == evt->data.evt_gatt_server_attribute_value.attribute) {
        Serial.println("Haptic Feedbacl characteristic data received");
        // Check the length of the received data
        if (evt->data.evt_gatt_server_attribute_value.value.len == 0) {
          break;
        } else {
          motorDriverLoop();
          motorDriverLoop();
        }
        
      }
      break;
    case sl_bt_evt_gatt_server_characteristic_status_id:
      if (haptic_status_characteristic_handle == evt->data.evt_gatt_server_attribute_value.attribute) {
        Serial.println("Haptic Status characteristic data received");
        // Check the length of the received data
        if (evt->data.evt_gatt_server_attribute_value.value.len == 0) {
          break;
        }
      }
      break;
    // Default event handler
    default:
      break;
  }
}

/**************************************************************************//**
 * Starts BLE advertisement
 *****************************************************************************/
static void ble_start_advertising() {
  Serial.println("Inside ble_start_advertising");
  static uint8_t advertising_set_handle = 0xff; // Or a specific handle if you need multiple sets
  sl_status_t sc;

  // Create an advertising set if not already created.
  if (advertising_set_handle == 0xff) {
    sc = sl_bt_advertiser_create_set(&advertising_set_handle);
    app_assert_status(sc);

    // Set advertising interval to 100ms.
    sc = sl_bt_advertiser_set_timing(
      advertising_set_handle,
      160, // 100ms min interval (160 * 0.625ms)
      160, // 100ms max interval
      0,   // duration: 0 for continuous advertising
      0);  // max_events: 0 for no limit
    app_assert_status(sc);
  }

  // --- Prepare Advertisement Data ---
  // Max 31 bytes for legacy advertising packet
  uint8_t adv_data[31];
  uint8_t adv_data_len = 0;

  // Flags: LE General Discoverable Mode, BR/EDR Not Supported
  adv_data[adv_data_len++] = 2; // Length of this field
  adv_data[adv_data_len++] = 0x01; // Type: Flags
  adv_data[adv_data_len++] = 0x06; // Data: LE General Discoverable, BR/EDR Not Supported

  // Complete List of 128-bit Service Class UUIDs
  adv_data[adv_data_len++] = 17; // Length: 1 (type) + 16 (UUID)
  adv_data[adv_data_len++] = 0x07; // Type: Complete List of 128-bit Service UUIDs
  memcpy(&adv_data[adv_data_len], generic_access_service_uuid, 16);
  adv_data_len += 16;

  // Complete Local Name (if space permits, otherwise use short name or put in scan response)
  uint8_t name_len = sizeof(advertised_name) -1;
  if (adv_data_len + 2 + name_len <= 31) {
      adv_data[adv_data_len++] = name_len + 1; // Length
      adv_data[adv_data_len++] = 0x09; // Type: Complete Local Name
      memcpy(&adv_data[adv_data_len], advertised_name, name_len);
      adv_data_len += name_len;
  } else {
      Serial.println("Advertised name too long for main packet, consider scan response.");
      // Or use a shortened name
  }


  sc = sl_bt_legacy_advertiser_set_data(advertising_set_handle,
                                        sl_bt_advertiser_advertising_data_packet,
                                        adv_data_len,
                                        adv_data);
  app_assert_status(sc);

  // Start advertising and enable connections.
  sc = sl_bt_legacy_advertiser_start(advertising_set_handle,
                                     sl_bt_advertiser_connectable_scannable);
  app_assert_status(sc);

  Serial.print("Started advertising with Service UUID and name '");
  Serial.print((const char *)advertised_name);
  Serial.println("'...");
}

/**************************************************************************/ /**
 * Initializes the GATT database
 * Creates a new GATT session and adds certain services and characteristics
 *****************************************************************************/
static void ble_initialize_gatt_db() {
  Serial.println("Inside ble_initialize_gatt_db");

  sl_status_t sc;
  // Create a new GATT database
  sc = sl_bt_gattdb_new_session(&gattdb_session_id);
  app_assert_status(sc);

  // Add the Generic Access service to the GATT DB
  sc = sl_bt_gattdb_add_service(gattdb_session_id,
                                sl_bt_gattdb_primary_service,
                                SL_BT_GATTDB_ADVERTISED_SERVICE,
                                sizeof(generic_access_service_uuid),
                                generic_access_service_uuid,
                                &generic_access_service_handle);
  app_assert_status(sc);

  // Add the Device Name characteristic to the Generic Access service
  // The value of the Device Name characteristic will be advertised
  const sl_bt_uuid_16_t device_name_characteristic_uuid = { .data = { 0x00, 0x2A } };
  sc = sl_bt_gattdb_add_uuid16_characteristic(gattdb_session_id,
                                              generic_access_service_handle,
                                              SL_BT_GATTDB_CHARACTERISTIC_READ,
                                              0x00,
                                              0x00,
                                              device_name_characteristic_uuid,
                                              sl_bt_gattdb_fixed_length_value,
                                              sizeof(advertised_name) - 1,
                                              sizeof(advertised_name) - 1,
                                              advertised_name,
                                              &name_characteristic_handle);
  app_assert_status(sc);


  // Start the Generic Access service
  sc = sl_bt_gattdb_start_service(gattdb_session_id, generic_access_service_handle);
  app_assert_status(sc);

  // Add the IMU Data service to the GATT DB
  const uint8_t imu_data_access_service_uuid[] = { 0x00, 0x19 };
  sc = sl_bt_gattdb_add_service(gattdb_session_id,
                                sl_bt_gattdb_primary_service,
                                SL_BT_GATTDB_ADVERTISED_SERVICE,
                                sizeof(imu_data_access_service_uuid),
                                imu_data_access_service_uuid,
                                &imu_data_access_service_handle);
  app_assert_status(sc);

  // Add the IMU Data characteristic to the Generic Access service
  // IMU_DATA_UUID: 6137df56-b267-46bb-b86c-f4614a9f67a6
  const uuid_128 imu_data_characteristic_uuid = {
    .data = { 0xa6, 0x67, 0x9f, 0x4a, 0x61, 0xf4, 0x6c, 0xb8, 0xbb, 0x46, 0x67, 0xb2, 0x56, 0xdf, 0x37, 0x61 }
  };
  sc = sl_bt_gattdb_add_uuid128_characteristic(gattdb_session_id,
                                               imu_data_access_service_handle,
                                               SL_BT_GATTDB_CHARACTERISTIC_READ | SL_BT_GATTDB_CHARACTERISTIC_INDICATE,
                                               0x00,
                                               0x00,
                                               imu_data_characteristic_uuid,
                                               sl_bt_gattdb_variable_length_value,
                                               0,
                                               30,
                                               NULL,
                                               &imu_data_characteristic_handle);
  app_assert_status(sc);

  // Start the IMU Data Access service
  sc = sl_bt_gattdb_start_service(gattdb_session_id, imu_data_access_service_handle);
  app_assert_status(sc);


  //
  // Haptic Status Service
  //
  // Add the Haptic Status service to the GATT DB
  const uint8_t haptic_status_service_uuid[] = { 0x02, 0x19 };
  sc = sl_bt_gattdb_add_service(gattdb_session_id,
                                sl_bt_gattdb_primary_service,
                                SL_BT_GATTDB_ADVERTISED_SERVICE,  
                                sizeof(haptic_status_service_uuid),
                                haptic_status_service_uuid,
                                &haptic_status_service_handle);  // Use a new handle for this service
  app_assert_status(sc);


  // HAPTIC_FEEDBACK_UUID: ff422d26-d740-47a7-a364-2a7aed5ab23d
  const uuid_128 haptic_feedback_characteristic_uuid = {
    .data = { 0x3d, 0xb2, 0x5a, 0xed, 0x7a, 0x2a, 0x64, 0xa3, 0xa7, 0x47, 0x40, 0xd7, 0x26, 0x2d, 0x42, 0xff }
  };
  sc = sl_bt_gattdb_add_uuid128_characteristic(gattdb_session_id,
                                               haptic_status_service_handle,
                                               SL_BT_GATTDB_CHARACTERISTIC_WRITE,  
                                               0x00,
                                               0x00,
                                               haptic_feedback_characteristic_uuid,
                                               sl_bt_gattdb_variable_length_value,
                                               0,   // Initial length
                                               20,  // Max length for haptic command
                                               NULL,
                                               &haptic_feedback_characteristic_handle);  // Use the global handle
  app_assert_status(sc);

  // HAPTIC_STATUS_UUID: 3506f523-e28b-4d9c-a4a3-648e1a943ee0
  const uuid_128 haptic_status_characteristic_uuid = {
    .data = { 0xe0, 0x3e, 0x94, 0x1a, 0x8e, 0x64, 0xa3, 0xa4, 0x9c, 0x4d, 0x8b, 0xe2, 0x23, 0xf5, 0x06, 0x35 }
  };
  sc = sl_bt_gattdb_add_uuid128_characteristic(gattdb_session_id,
                                               haptic_status_service_handle,
                                               SL_BT_GATTDB_CHARACTERISTIC_NOTIFY,  // Example
                                               0x00,
                                               0x00,
                                               haptic_status_characteristic_uuid,
                                               sl_bt_gattdb_variable_length_value,
                                               0,
                                               10,  // Max length for status
                                               NULL,
                                               &haptic_status_characteristic_handle);
  app_assert_status(sc);

  sc = sl_bt_gattdb_start_service(gattdb_session_id, haptic_status_service_handle);
  app_assert_status(sc);


  // Commit the GATT DB changes
  sc = sl_bt_gattdb_commit(gattdb_session_id);
  app_assert_status(sc);
}

#ifndef BLE_STACK_SILABS
#error "This example is only compatible with the Silicon Labs BLE stack. Please select 'BLE (Silabs)' in 'Tools > Protocol stack'."
#endif
