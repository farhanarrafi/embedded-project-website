#define Motor D6 //LED_BUILTIN
#define button1 D7
#define button2 D8

int brightness = 0;  // how bright the LED is
int fadeAmount = 25;  // how many points to fade the LED by

//variables to keep track of the timing of recent interrupts
unsigned long button_time1 = 0, button_time2 = 0;  
unsigned long last_button_time1 = 0, last_button_time2 = 0; 
// unsigned long H = 800, L = 200;

void IRAM_ATTR isr1() {
    button_time1 = millis();
    if (button_time1 - last_button_time1 > 250)
    {
      last_button_time1 = button_time1;
      if (brightness <= (255-fadeAmount)) {
        brightness += fadeAmount;
      }
      analogWrite(Motor, brightness);
    }
}

void IRAM_ATTR isr2() {
    button_time2 = millis();
    if (button_time2 - last_button_time2 > 250)
    {
      last_button_time2 = button_time2;
      if(brightness >= fadeAmount){
        brightness -= fadeAmount;
      }
      analogWrite(Motor, brightness);
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(button1, INPUT);
    pinMode(button2, INPUT);
    pinMode(Motor, OUTPUT);

    attachInterrupt(button1, isr1, RISING);
    attachInterrupt(button2, isr2, RISING);
}

void loop() {
  Serial.println(brightness);
}
