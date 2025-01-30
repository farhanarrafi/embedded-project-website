#include <PCF8563.h>
PCF8563 pcf;
void setup() {
  Serial.begin(9600);
  pcf.init(); //initialize the clock

  


  pcf.stopClock(); //stop the clock
  pcf.startClock(); //start the clock
}
void loop() {
  Time nowTime = pcf.getTime();//get current time
  //print the current time through the serial monitor
  Serial.print(nowTime.day);
  Serial.print("/");
  Serial.print(nowTime.month);
  Serial.print("/");
  Serial.print(nowTime.year);
  Serial.print(" ");
  Serial.print(nowTime.hour);
  Serial.print(":");
  Serial.print(nowTime.minute);
  Serial.print(":");
  Serial.println(nowTime.second);
  delay(1000);
}