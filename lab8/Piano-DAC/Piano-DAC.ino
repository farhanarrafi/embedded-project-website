#include "esp_timer.h"

const uint8_t SineWave[32] = {
  8,  9, 10, 12, 13, 14, 14, 15,
 15, 15, 14, 14, 13, 12, 10,  9,
  8,  6,  5,  3,  2,  1,  1,  0,
  0,  0,  1,  1,  2,  3,  5,  6
};

volatile uint8_t Index = 0;
volatile bool waveformActive = false;

// === DAC output pins (simulate 3-bit DAC on GPIO 0, 1, 2) ===
const int DAC_PINS[4] = {0, 1, 2, 3};

// === Switch pin (active-high when pressed) ===
const int KEY_PINS[4] = {D7, D8, D9, D10};  // Updated from D6 to D3

const int timerPeriods[4] = {120, 106, 95, 80}; // Approximate values {238, 213, 189, 159}

// === For edge detection ===
bool lastSwitchState = LOW;

// === Timer handle ===
esp_timer_handle_t periodic_timer;

// === Initialize DAC Pins ===
void DAC_Init() {
  for (int i = 0; i < 4; i++) {
    pinMode(DAC_PINS[i], OUTPUT);
    digitalWrite(DAC_PINS[i], LOW);
  }
}

// === Output 3-bit Value to DAC ===
void DAC_Out(uint8_t value) {
  for (int i = 0; i < 4; i++) {
    digitalWrite(DAC_PINS[i], (value >> i) & 0x01);
  }
}

// === Timer callback function ===
void onTimer(void* arg) {
  if (waveformActive) {
    Index = (Index + 1) & 0x1F;
    DAC_Out(SineWave[Index]);
  } else {
    DAC_Out(0);
  }
}

// === Setup Function ===
void setup() {
  Serial.begin(115200);
  DAC_Init();

  for (int i = 0; i < 4; i++) {
    pinMode(KEY_PINS[i], INPUT);  // Define KEY PINS as input
  }
  

  // Create the esp_timer
  const esp_timer_create_args_t timer_args = {
    .callback = &onTimer,
    .arg = NULL,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "sine_timer"
  };

  esp_timer_create(&timer_args, &periodic_timer);
  //esp_timer_start_periodic(periodic_timer, 625); // 625 µs for ~100Hz sine wave
}

// === Main Loop ===
void loop() {
  bool currentSwitchState = LOW;

  for (int i=0; i<4; i++) {
    if (digitalRead(KEY_PINS[i]) == HIGH) {
      currentSwitchState = digitalRead(KEY_PINS[i]);
      waveformActive = true;
      esp_timer_start_periodic(periodic_timer, timerPeriods[i]); // 625 µs for ~100Hz sine wave timerPeriods[i]          // Start the timer
    } else {
      waveformActive = false;
      DAC_Out(0); // Silence when button is released
    }
  }

  lastSwitchState = currentSwitchState;
}