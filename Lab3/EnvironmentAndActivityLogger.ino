# include <Arduino.h>
#include <ArduinoBLE.h>
# include <U8x8lib.h> //Display library
# include <Wire.h> //I2C protocol library (the display uses I2C to interact with MCU)
# include <PCF8563.h>
# include "LSM6DS3.h"
# include "Wire.h"
# include <PDM.h>

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/ PIN_WIRE_SCL, /* data=*/ PIN_WIRE_SDA, /* reset=*/ U8X8_PIN_NONE); // OLEDs without Reset of the Display

const int buttonPin = 1;     // the number of the pushbutton pin
int buttonState = 0;   

// Resistor values in your voltage divider (in ohms)
const float R1 = 2700.0;  // 1kΩ resistor
const float R2 = 10000.0;  // 4.7kΩ resistor

// Reference voltage of the ADC (usually 5V or 3.3V)
const float referenceVoltage = 3.3;

const int ledPin = LED_BLUE; // set ledPin to on-board LED
const int buttonPin = 4; // set buttonPin to digital pin 4

// PDM start
// default number of output channels
static const char channels = 1;

// default PCM output frequency
static const int frequency = 16000;

// Buffer to read samples into, each sample is 16-bits
short sampleBuffer[512];

// Number of audio samples read
volatile int samplesRead;
// PDM end

BLEService ledService("19B10010-E8F2-537E-4F6C-D104768A1214"); // create service

// create switch characteristic and allow remote device to read and write
BLEByteCharacteristic ledCharacteristic("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
// create button characteristic and allow remote device to get notifications
BLEByteCharacteristic buttonCharacteristic("19B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

PCF8563 pcf;

void setup() {
  // Initialize serial communication at 9600 bits per second:
  // Serial.begin(9600);

  // setup Bluetooth

  // setup sensors
  setupIMUAndMicrophone();
  // setup RTC
  setupClock();
  // setup SD card
  
  // setup OLED
  setupOLED();
}

void loop() {
  // put your main code here, to run repeatedly:
  float batteryVoltage = readBattery();

  runBluetooth();

  char timestamp[] = getTimeStamp(); 
}

float readBattery() {
  delay(500); // Delay in between reads for stability
  // Read the input on analog pin A0:
  int sensorValue = analogRead(A0);

  // Convert the ADC value to voltage:
  float measuredVoltage = sensorValue * (referenceVoltage / 1023.0);

  // Account for the voltage divider to get the actual battery voltage:
  float batteryVoltage = measuredVoltage * ((R1 + R2) / R2);

  // Print out the battery voltage:
  Serial.print("Battery Voltage: ");
  Serial.print(batteryVoltage);
  Serial.println(" V");

  return batteryVoltage;
  
}

void setupOLED() {
    u8x8.begin();
    u8x8.setFlipMode( 1 ); // set number from 1 to 3, the screen word will rotary 180
}

void showVoltageAndSystemStatus(float batteryVoltage, char systemStatus[]) {
  u8x8.setFont(u8x8_font_chroma48medium8_r); //try u8x8_font_px437wyse700a_2x2_r
  u8x8.setCursor( 0 , 0 ); // It will start printing from (0,0) location
  u8x8.print("Batt voltage: " + batteryVoltage);
  u8x8.setCursor( 0 , 1); // (columns, row)
  u8x8.print("                    ");
  u8x8.setCursor( 0 , 2 );
  u8x8.print("System status: " + systemStatus);
  u8x8.setCursor( 0 , 3 );
  u8x8.print("                    ");
}

char getTimeStamp() {
  Time now = pcf.getTime();//get current time
  return(now.day+ "/" +now.month + "/" + now.year + " " + now.hour + ":" + now.minute + ":" + now.second);
}

void setupClock() {
  
  pcf.init();//initialize the clock

  pcf.stopClock();//stop the clock

  // //set time to to 31/3/2018 17:33:0

  // Run only first time. Then comment out.
  // pcf.setYear(24);//set year
  // pcf.setMonth(1);//set month
  // pcf.setDay(21);//set dat
  // pcf.setHour(20);//set hour
  // pcf.setMinut(7);//set minut
  // pcf.setSecond(13);//set second

  pcf.startClock();//start the clock
}

void setupBluetooth() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(ledPin, OUTPUT); // use the LED as an output
  pinMode(buttonPin, INPUT); // use button pin as an input

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1);
  }

  // set the local name peripheral advertises
  BLE.setLocalName("Farhan-Arduino");
  // set the UUID for the service this peripheral advertises:
  BLE.setAdvertisedService(ledService);

  // add the characteristics to the service
  ledService.addCharacteristic(ledCharacteristic);
  ledService.addCharacteristic(buttonCharacteristic);

  // add the service
  BLE.addService(ledService);

  ledCharacteristic.writeValue(0);
  buttonCharacteristic.writeValue(0);

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth® device active, waiting for connections...");
}

void runBluetooth() {
  // poll for Bluetooth® Low Energy events
  BLE.poll();

  // read the current button pin state
  char buttonValue = digitalRead(buttonPin);

  // has the value changed since the last read
  bool buttonChanged = (buttonCharacteristic.value() != buttonValue);

  if (buttonChanged) {
    // button state changed, update characteristics
    ledCharacteristic.writeValue(buttonValue);
    buttonCharacteristic.writeValue(buttonValue);
  }

  if (ledCharacteristic.written() || buttonChanged) {
    // update LED, either central has written to characteristic or button state has changed
    if (ledCharacteristic.value()) {
      Serial.println("LED on");
      digitalWrite(ledPin, HIGH);
    } else {
      Serial.println("LED off");
      digitalWrite(ledPin, LOW);
    }
  }
}

void setupIMUAndMicrophone() {
  //Call .begin() to configure the IMUs
    if (myIMU.begin() != 0) {
        Serial.println("Device error");
    } else {
        Serial.println("Device OK!");
    }
}

char getIMUAccelerometer() {
  return("Accelerometer: " + " X1 = " + myIMU.readFloatAccelX()+ " Y1 = " + myIMU.readFloatAccelY() +  " Z1 = " + myIMU.readFloatAccelZ());
}
char getIMUGyroscope() {
    return("Gyroscope: " + " X1 = " + myIMU.readFloatGyroX()+ " Y1 = " + myIMU.readFloatGyroY() +  " Z1 = " + myIMU.readFloatGyroZ());
}
char getIMUThermometer() {
  return("Thermometer: " + "  Degrees C1 = " + myIMU.readTempC()+ " Degrees F1 = " + myIMU.readTempF());
}

void setupPDM() {
  while (!Serial);

  // Configure the data receive callback
  PDM.onReceive(onPDMdata);

  // Optionally set the gain
  // Defaults to 20 on the BLE Sense and 24 on the Portenta Vision Shield
  // PDM.setGain(30);

  // Initialize PDM with:
  // - one channel (mono mode)
  // - a 16 kHz sample rate for the Arduino Nano 33 BLE Sense
  // - a 32 kHz or 64 kHz sample rate for the Arduino Portenta Vision Shield
  if (!PDM.begin(channels, frequency)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }
}

void readPDMMicrophoneData() {
  // Wait for samples to be read
  if (samplesRead) {

    // Print samples to the serial monitor or plotter
    for (int i = 0; i < samplesRead; i++) {
      if(channels == 2) {
        Serial.print("L:");
        Serial.print(sampleBuffer[i]);
        Serial.print(" R:");
        i++;
      }
      Serial.println(sampleBuffer[i]);
    }

    // Clear the read count
    samplesRead = 0;
  }
}

/**
 * Callback function to process the data from the PDM microphone.
 * NOTE: This callback is executed as part of an ISR.
 * Therefore using `Serial` to print messages inside this function isn't supported.
 * */
void onPDMdata() {
  // Query the number of available bytes
  int bytesAvailable = PDM.available();

  // Read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}
