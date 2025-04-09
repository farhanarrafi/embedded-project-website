// === 3-bit Sine Wave Table (0 to 7) ===
const uint8_t SineWave[16] = {
  4, 5, 6, 7, 7, 7, 6, 5,
  4, 3, 2, 1, 1, 1, 2, 3
};

volatile uint8_t Index = 0;
volatile bool waveformActive = false;  // Track if wave generation is on

// === DAC output pins (simulate 3-bit DAC on GPIO 0, 1, 2) ===
const int DAC_PINS[4] = {0, 1, 2, 3};

// === Switch pin (active-high when pressed) ===
const int KEY_PINS = {D7, D8, D9, D10};  // Updated from D6 to D3

// === ESP32 Hardware Timer ===
hw_timer_t * timer = NULL;

// For edge detection
bool lastSwitchState = LOW;

const uint16_t timerPeriods[4] = {120, 106, 95, 80};

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
    digitalWrite(DAC_PINS[i], (value >> i) & 0x02);
  }
}

// === Timer Interrupt: Update DAC Value ===
void IRAM_ATTR onTimer() {
  if (waveformActive) {
    Index = (Index + 1) & 0x0F;
    DAC_Out(SineWave[Index]);
  }
}

// === Setup Function ===
void setup() {
  Serial.begin(115200); // Start serial communication
  DAC_Init();

  for (int i = 0; i < 4; i++) {
    pinMode(KEY_PINS[0], INPUT);  // Define KEY PINS as input
  }
  

  // Timer config: 1 tick = 1 µs (80 MHz / 80)
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 625, true); // 625 µs = 1600 Hz update rate
  timerAlarmEnable(timer);           // Start the timer
}

// === Main Loop ===
void loop() {
  bool currentSwitchState = digitalRead(KEY_PINS[0]);

  // if (currentSwitchState == HIGH && lastSwitchState == LOW) {
  //   // Rising edge detected
  //   Serial.println("Switch Pressed!");
  // }

  for (int i=0; i<4; i++) {
    if (digitalRead(KEY_PINS[i]) == HIGH) {
      waveformActive = true;
      // Timer config: 1 tick = 1 µs (80 MHz / 80)
      timer = timerBegin(0, 80, true);
      timerAttachInterrupt(timer, &onTimer, true);
      timerAlarmWrite(timer, timerPeriods[i], true);
      timerAlarmEnable(timer);           // Start the timer
      return;
    }
    waveformActive = false;
    DAC_Out(0); // Reset DAC output when switch is not pressed
  }

  lastSwitchState = currentSwitchState;
}