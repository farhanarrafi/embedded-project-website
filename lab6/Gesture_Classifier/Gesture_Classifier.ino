#include <LSM6DS3.h>
#include <Wire.h>
#include <U8x8lib.h>  //Display library
#include <PCF8563.h>
#include <SPI.h>
#include <SD.h>

#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
//#include <tensorflow/lite/version.h>

#include "model.h"

PCF8563 pcf;

U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* clock=*/PIN_WIRE_SCL, /*data=*/PIN_WIRE_SDA, /* reset=*/U8X8_PIN_NONE);  // OLEDs without Reset of the Display

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);  //I2C device address 0x6A

const PROGMEM float accelerationThreshold = 2;  // threshold of significant in G's
//const int numSamples = 119;
const PROGMEM int numSamples = 119;
int samplesRead = numSamples;

const PROGMEM int chipSelect = 2;

// global variables used for TensorFlow Lite (Micro)
tflite::MicroErrorReporter tflErrorReporter;

// pull in all the TFLM ops, you can remove this line and
// only pull in the TFLM ops you need, if would like to reduce
// the compiled size of the sketch.
tflite::AllOpsResolver tflOpsResolver;

const tflite::Model* tflModel = nullptr;
tflite::MicroInterpreter* tflInterpreter = nullptr;
TfLiteTensor* tflInputTensor = nullptr;
TfLiteTensor* tflOutputTensor = nullptr;

// Create a static memory buffer for TFLM, the size may need to
// be adjusted based on the model you are using
constexpr int tensorArenaSize = 8 * 1024;
byte tensorArena[tensorArenaSize] __attribute__((aligned(16)));

// array to map gesture index to a name
const char* GESTURES[] = {
  "Sitting",
  "Walking",
  "Busing",
  "Climbing"
};

#define NUM_GESTURES (sizeof(GESTURES) / sizeof(GESTURES[0]))

void setup() {
  Serial.begin(9600);
  u8x8.begin();
  u8x8.setFlipMode(1);  // set number from 1 to 3, the screen word will rotary 180
  u8x8.setFont(u8x8_font_chroma48medium8_r);

  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_BLUE, LOW);

  if (myIMU.begin() != 0) {
    u8x8.print("Device error");
  } else {
    u8x8.print("Device OK!");
  }

  // get the TFL representation of the model byte array
  tflModel = tflite::GetModel(model);
  // if (tflModel->version() != TFLITE_SCHEMA_VERSION) {
  //   Serial.println("Model schema mismatch!");
  //   while (1);
  // }

  // Create an interpreter to run the model
  tflInterpreter = new tflite::MicroInterpreter(tflModel, tflOpsResolver, tensorArena, tensorArenaSize, &tflErrorReporter);

  // Allocate memory for the model's input and output tensors
  tflInterpreter->AllocateTensors();

  // Get pointers for the model's input and output tensors
  tflInputTensor = tflInterpreter->input(0);
  tflOutputTensor = tflInterpreter->output(0);

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
  dataFile.print(",");
  dataFile.println("Activity");
  // You can add the label here just make sure to only use "println" for the last thing you want to print
  dataFile.close();
  digitalWrite(LED_BLUE, HIGH);
}

void loop() {

  Time nowTime = pcf.getTime();  //get current time

  float aX, aY, aZ, gX, gY, gZ;

  u8x8.clear();
  digitalWrite(LED_BLUE, HIGH);
  digitalWrite(LED_GREEN, HIGH);


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
    // check if new acceleration AND gyroscope data is available
    // read the acceleration and gyroscope data
    aX = myIMU.readFloatAccelX();
    aY = myIMU.readFloatAccelY();
    aZ = myIMU.readFloatAccelZ();

    gX = myIMU.readFloatGyroX();
    gY = myIMU.readFloatGyroY();
    gZ = myIMU.readFloatGyroZ();

    // normalize the IMU data between 0 to 1 and store in the model's
    // input tensor
    tflInputTensor->data.f[samplesRead * 6 + 0] = aX;
    tflInputTensor->data.f[samplesRead * 6 + 1] = aY;
    tflInputTensor->data.f[samplesRead * 6 + 2] = aZ;
    tflInputTensor->data.f[samplesRead * 6 + 3] = (gX + 2000.0) / 4000.0;
    tflInputTensor->data.f[samplesRead * 6 + 4] = (gY + 2000.0) / 4000.0;
    tflInputTensor->data.f[samplesRead * 6 + 5] = (gZ + 2000.0) / 4000.0;

    samplesRead++;

    if (samplesRead == numSamples) {
      // Run inferencing
      TfLiteStatus invokeStatus = tflInterpreter->Invoke();
      if (invokeStatus != kTfLiteOk) {
        //Serial.println("Invoke failed!");
        u8x8.print("Invoke failed!");
        while (1)
          ;
        return;
      }


      digitalWrite(LED_GREEN, LOW);
      u8x8.print("Writing in SD");
      File dataFile = SD.open("datalog.txt", FILE_WRITE);
      // Loop through the output tensor values from the model
      for (int i = 0; i < NUM_GESTURES; i++) {
        float maxProb = 0.0;
        for (int j = 0; j < 1000; j++) {
          if (tflOutputTensor->data.f[i] > maxProb) {
            maxProb = tflOutputTensor->data.f[i];
          }
        }
        if (dataFile && maxProb > 0.6) {
            dataFile.print(nowTime.hour);
            dataFile.print(":");
            dataFile.print(nowTime.minute);
            dataFile.print(":");
            dataFile.print(nowTime.second);
            dataFile.print(", ");
            dataFile.print(GESTURES[i]);
            dataFile.print(" : ");
            dataFile.println(maxProb, 6);
          }
      }
      dataFile.close();
      digitalWrite(LED_GREEN, HIGH);
    }
  }
}
