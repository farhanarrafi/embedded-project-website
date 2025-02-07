# include <Arduino.h>
# include <ArduinoBLE.h>
# include <U8x8lib.h> //Display library
# include <Wire.h> //I2C protocol library (the display uses I2C to interact with MCU)
# include <PCF8563.h>
# include "LSM6DS3.h"
# include "Wire.h"

# include <SPI.h>
# include <SD.h>

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

//const int ledPin = LED_BLUE; // set ledPin to on-board LED
const int buttonPinBLE = 4; // set buttonPin to digital pin 4

// Battery Voltage Threshold

const float batteryThreshold = 3.80;


// SD Card Start

const int chipSelect = 2;

// SD Card End

BLEService ledService("19B10010-E8F2-537E-4F6C-D104768A1214"); // create service

// create switch characteristic and allow remote device to read and write
BLEByteCharacteristic ledCharacteristic("19B10011-E8F2-537E-4F6C-D104768A1214", BLERead | BLEWrite);
// create button characteristic and allow remote device to get notifications
BLEByteCharacteristic buttonCharacteristic("19B10012-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

PCF8563 pcf;

bool SYSTEM_ON = false;

bool LOW_BATTERY = false;

void setup() {
  // Initialize serial communication at 9600 bits per second:
  // Serial.begin(9600);
  Serial.begin(9600);
  while (!Serial);
  SYSTEM_ON = false;
  // setup Bluetooth
  setupBluetooth();
  // setup sensors
  setupIMUAndMicrophone();
  // setup RTC
  setupClock();
  // setup SD card
  setupSDCard();
  // setup OLED
  setupOLED();
}

void loop() {
  // put your main code here, to run repeatedly:
  
  runBluetooth();



  // update LED, either central has written to characteristic or button state has changed
  if (SYSTEM_ON) {
    //Serial.println("System on");
    showVoltageAndSystemStatus(true);
    writeDatainSDCard();
  } else {
    //Serial.println("System off");
    showVoltageAndSystemStatus(false);
  }
  
}

float readBattery() {
  // Read the input on analog pin A0:
  int sensorValue = analogRead(A0);

  // Convert the ADC value to voltage:
  float measuredVoltage = sensorValue * (referenceVoltage / 1023.0);

  // Account for the voltage divider to get the actual battery voltage:
  float batteryVoltage = measuredVoltage * ((R1 + R2) / R2);

  return batteryVoltage;
  
}

void setupOLED() {
    u8x8.begin();
    u8x8.setFlipMode( 1 ); // set number from 1 to 3, the screen word will rotary 180
}

void showVoltageAndSystemStatus(bool systemStatus) {
  
  float batteryVoltage = readBattery();

  Time now = pcf.getTime();//get current time
  //print current time
  char timeString[20];
  sprintf(timeString, "%d/%d/%d %d:%d:%d", now.day, now.month, now.year, now.hour, now.minute ,now.second);

  char batteryString[20];
  sprintf(batteryString, "Batt: %.2f V    ", batteryVoltage);

  if(batteryVoltage > batteryThreshold) {
    if(!systemStatus) {
      u8x8.setFont(u8x8_font_chroma48medium8_r); //try u8x8_font_px437wyse700a_2x2_r
      u8x8.setCursor( 0 , 0 ); // It will start printing from (0,0) location
      u8x8.print(batteryString);
      u8x8.setCursor( 0 , 1); // (columns, row)
      u8x8.print("                    ");
      u8x8.setCursor( 0 , 2 );
      u8x8.print("Sys: OFF");
      u8x8.setCursor( 0 , 3 );
      u8x8.print("                    ");
      u8x8.setCursor( 0 , 4 );
      u8x8.print(timeString);
    } else {
      u8x8.setFont(u8x8_font_chroma48medium8_r); //try u8x8_font_px437wyse700a_2x2_r
      u8x8.setCursor( 0 , 0 ); // It will start printing from (0,0) location
      u8x8.print(batteryString);
      u8x8.setCursor( 0 , 1); // (columns, row)
      u8x8.print("                    ");
      u8x8.setCursor( 0 , 2 );
      u8x8.print("Sys: ON ");
      u8x8.setCursor( 0 , 3 );
      u8x8.print("                    ");
      u8x8.setCursor( 0 , 4 );
      u8x8.print(timeString);
    }
  } else {
    SYSTEM_ON = false;
    u8x8.setFont(u8x8_font_chroma48medium8_r); //try u8x8_font_px437wyse700a_2x2_r
    u8x8.setCursor( 0 , 0 ); // It will start printing from (0,0) location
    u8x8.print("Low Battery    ");
    u8x8.setCursor( 0 , 1); // (columns, row)
    u8x8.print("Logging Stopped");
    u8x8.setCursor( 0 , 2 );
    u8x8.print("                    ");
    u8x8.setCursor( 0 , 3 );
    u8x8.print("                    ");
    u8x8.setCursor( 0 , 4 );
    u8x8.print(timeString);
    Serial.println("Logging Stopped");
  }
  
}

void setupClock() {
  
  pcf.init();//initialize the clock

  pcf.stopClock();//stop the clock

  // //set time to to 31/3/2018 17:33:0

  //Run only first time. Then comment out.
  // pcf.setYear(24);//set year
  // pcf.setMonth(2);//set month
  // pcf.setDay(5);//set date
  // pcf.setHour(22);//set hour
  // pcf.setMinut(37);//set minut
  // pcf.setSecond(13);//set second

  pcf.startClock();//start the clock
}

void setupBluetooth() {
  

  //pinMode(ledPin, OUTPUT); // use the LED as an output
  pinMode(buttonPinBLE, INPUT); // use button pin as an input

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
  char buttonValue = digitalRead(buttonPinBLE);


  // has the value changed since the last read
  bool buttonChanged = (buttonCharacteristic.value() != buttonValue);
 

  if (buttonChanged) {
    Serial.println("buttonChanged");
    // button state changed, update characteristics
    ledCharacteristic.writeValue(buttonValue);
    buttonCharacteristic.writeValue(buttonValue);
  }

  if (ledCharacteristic.written()) {
    Serial.println("ledCharacteristic.written()");
    Serial.println(ledCharacteristic.value());
    //update LED, either central has written to characteristic or button state has changed
    if (ledCharacteristic.value()) {
      Serial.println("SYSTEM ON");
      SYSTEM_ON = true;
    } else {
      Serial.println("SYSTEM OFF");
      SYSTEM_ON = false;
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

void setupSDCard() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  // wait for Serial Monitor to connect. Needed for native USB port boards only:
  while (!Serial);

  Serial.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    Serial.println("initialization failed. Things to check:");
    Serial.println("1. is a card inserted?");
    Serial.println("2. is your wiring correct?");
    Serial.println("3. did you change the chipSelect pin to match your shield or module?");
    Serial.println("Note: press reset button on the board and reopen this Serial Monitor after fixing your issue!");
    while (true);
  }

  Serial.println("initialization done.");
}

void writeDatainSDCard() {
  delay(500);
  // make a string for assembling the data to log:
  
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("DataFile.txt", FILE_WRITE);
  // {"Accelerometer": {"X1": -0.0537, "Y1": -0.0644, "Z1": 1.0121}, "Gyroscope": {"X1": 1.26, "Y1": -0.07, "Z1": -1.68}, 
  // "Thermometer": {"Degrees C1": 26.8984, "Degrees F1": 80.4172}, "Timestamp": "5/2/24 23:25:21"}
  // if the file is available, write to it:
  if (dataFile) {
    Serial.println("Write in SD Card: Started");
    dataFile.print("{");
    dataFile.print("\"Accelerometer\"");
    dataFile.print(": ");
    dataFile.print("{\"X1\"");
    dataFile.print(": ");
    dataFile.print(myIMU.readFloatAccelX(), 4);
    dataFile.print(", ");
    dataFile.print("\"Y1\"");
    dataFile.print(": ");
    dataFile.print(myIMU.readFloatAccelY(), 4);
    dataFile.print(", ");
    dataFile.print("\"Z1\"");
    dataFile.print(": ");
    dataFile.print(myIMU.readFloatAccelZ(), 4);
    dataFile.print("}, ");

    //Gyroscope
    dataFile.print("\"Gyroscope\"");
    dataFile.print(": ");
    dataFile.print("{\"X1\"");
    dataFile.print(": ");
    dataFile.println(myIMU.readFloatGyroX(), 4);
    dataFile.print(", ");
    dataFile.print("\"Y1\"");
    dataFile.print(": ");
    dataFile.print(myIMU.readFloatGyroY(), 4);
    dataFile.print(", ");
    dataFile.print("\"Z1\"");
    dataFile.print(": ");
    dataFile.print(myIMU.readFloatGyroZ(), 4);
    dataFile.print("}, ");

    //Thermometer
    dataFile.print("\"Thermometer\"");
    dataFile.print(": ");
    dataFile.print("{\"C1\"");
    dataFile.print(": ");
    dataFile.print(myIMU.readTempC(), 4);
    dataFile.print(", ");
    dataFile.print("\"F1\"");
    dataFile.print(": ");
    dataFile.print(myIMU.readTempF(), 4);
    dataFile.print("}, ");

    Time now = pcf.getTime();//get current time
    //print current time
    dataFile.print("\"Timestamp\"");
    dataFile.print(": \"");
    dataFile.print(now.day);
    dataFile.print("/");
    dataFile.print(now.month);
    dataFile.print("/");
    dataFile.print(now.year);
    dataFile.print(" ");
    dataFile.print(now.hour);
    dataFile.print(":");
    dataFile.print(now.minute);
    dataFile.print(":");
    dataFile.print(now.second);
    dataFile.print("\"}\n");
    // close after writing
    dataFile.close();
    Serial.println("Write in SD Card: Ended");
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}
