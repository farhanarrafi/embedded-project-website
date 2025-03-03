#include "LSM6DS3.h"
#include "Wire.h"
#include <Arduino.h>
#include <U8x8lib.h>  //Display library
#include <PCF8563.h>
#include <SPI.h>
#include <SD.h>

PCF8563 pcf;

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/PIN_WIRE_SCL, /*data=*/PIN_WIRE_SDA, /* reset=*/U8X8_PIN_NONE);  // OLEDs without Reset of the Display

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);  //I2C device address 0x6A


const int chipSelect = 2;
unsigned long startTime;
unsigned long endTime;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  u8x8.begin();

  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  u8x8.setFlipMode(1);  // set number from 1 to 3, the screen word will rotary 180
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  // while (!Serial);
  //Call .begin() to configure the IMUs
  if (myIMU.begin() != 0) {
    //Serial.println("Device error");
    u8x8.print("Device error");
  } else {
    //Serial.println("Device OK!");
    u8x8.print("Device OK!");
  }

  Wire.begin();

  pcf.init();        //initialize the clock
  pcf.startClock();  //start the clock

  if (!SD.begin(chipSelect)) {
    //Serial.println("Card failed, or not present");
    u8x8.clear();
    u8x8.print("Insert SD Card!");
    // don't do anything more:
    while (1)
      ;
  }

  // Setting up the headers for the stored data!
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  dataFile.println("\n");
  dataFile.print("Time");
  dataFile.print(", ");
  dataFile.print("Accel_x");
  dataFile.print(", ");
  dataFile.print("Accel_y");
  dataFile.print(", ");
  dataFile.print("Accel_z");
  dataFile.print(", ");
  dataFile.print("Gyr_x");
  dataFile.print(", ");
  dataFile.print("Gyr_y");
  dataFile.print(", ");
  dataFile.print("Gyr_z");
  dataFile.print(", ");
  dataFile.println("Label");
  // You can add the label here just make sure to only use "println" for the last thing you want to print 
  dataFile.close();
}

void loop() {
  Time nowTime = pcf.getTime();  //get current time

  u8x8.clear();

  // u8x8.setCursor(0, 0);
  // u8x8.print("Data Collection");
  digitalWrite(LED_RED, HIGH);
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_GREEN, HIGH);

  float accelxDataArray[1000];
  float accelyDataArray[1000];
  float accelzDataArray[1000];

  float gyrxDataArray[1000];
  float gyryDataArray[1000];
  float gyrzDataArray[1000];


  // Serial.print(millis());
  digitalWrite(LED_BLUE, LOW);
  u8x8.print("Recording data");
  for (int i = 0; i < 1000; i++) {
    // Serial.println(millis());
    accelxDataArray[i] = float(myIMU.readFloatAccelX());
    accelyDataArray[i] = float(myIMU.readFloatAccelY());
    accelzDataArray[i] = float(myIMU.readFloatAccelZ());

    gyrxDataArray[i] = float(myIMU.readFloatGyroX());
    gyryDataArray[i] = float(myIMU.readFloatGyroY());
    gyrzDataArray[i] = float(myIMU.readFloatGyroZ());

    delay(7);  // To ensure 10 seconds of data in each batch
  }
  digitalWrite(LED_BLUE, HIGH);
  

  // Serial.print(",");
  // Serial.print(millis());
  // Serial.print(",");
  u8x8.clear();
  File dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
    digitalWrite(LED_GREEN, LOW);
    u8x8.print("Writing on SD");
    for (int j = 0; j < 1000; j++) {
      dataFile.print(nowTime.hour);
      dataFile.print(":");
      dataFile.print(nowTime.minute);
      dataFile.print(":");
      dataFile.print(nowTime.second);
      dataFile.print(", ");
      dataFile.print(accelxDataArray[j]);
      dataFile.print(", ");
      dataFile.print(accelyDataArray[j]);
      dataFile.print(", ");
      dataFile.print(accelzDataArray[j]);
      dataFile.print(", ");
      dataFile.print(gyrxDataArray[j]);
      dataFile.print(", ");
      dataFile.print(gyryDataArray[j]);
      dataFile.print(", ");
      dataFile.println(gyrzDataArray[j]);
    }
    digitalWrite(LED_GREEN, HIGH);
    u8x8.setCursor(0, 0);
    u8x8.clear();
    // LED would blink (red) after each successful file write
    digitalWrite(LED_RED, LOW);
    // Serial.println(millis());
  }
  dataFile.close();
}
