#include "LSM6DS3.h"
LSM6DS3 myIMU(I2C_MODE, 0x6A);
const float accelerationThreshold = 2;  // threshold of significant in G's
const int numSamples = 119;
int samplesRead = numSamples;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial)
    ;
  //Call .begin() to configure the IMUs
  if (myIMU.begin() != 0) {
    Serial.println("Device error");
  } else {
    Serial.println("Device OK!");
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  float x, y, z;

  while (samplesRead == numSamples) {
    x = myIMU.readFloatAccelX();
    y = myIMU.readFloatAccelY();
    z = myIMU.readFloatAccelZ();
    float aSum = fabs(x) + fabs(y) + fabs(z);
    // check if it's above the threshold
    if (aSum >= accelerationThreshold) {
      // reset the sample read count
      samplesRead = 0;
      break;
    }
  }

  
  while (samplesRead < numSamples) {
    x = myIMU.readFloatAccelX();
    y = myIMU.readFloatAccelY();
    z = myIMU.readFloatAccelZ();
    samplesRead++;
    Serial.print(x, 3);
    Serial.print(',');
    Serial.print(y, 3);
    Serial.print(',');
    Serial.print(z, 3);
    Serial.println();
    if (samplesRead == numSamples) {
      // Serial.println();
    }
  }
}
