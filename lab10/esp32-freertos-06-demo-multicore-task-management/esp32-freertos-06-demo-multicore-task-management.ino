/**
* ESP32 Dual-Core LED Toggle with Serial Monitor Status
*
* Two LEDs are toggled at different rates, each handled by a separate core.
*/
// Core assignments
static const BaseType_t core_0 = 0;  // PRO_CPU
static const BaseType_t core_1 = 1;  // APP_CPU
// LED Pins (set to valid GPIOs on your board)
const int led1_pin = LED_BUILTIN;  // Core 0
const int led2_pin = D6;            // Core 1
// Blink rates
const int led1_rate = 500;  // ms
const int led2_rate = 250;  // ms
// LED states
volatile bool led1_state = false;
volatile bool led2_state = false;
//*****************************************************************************
// Tasks
void blinkLED_Core0(void *parameter) {
  while (1) {
    digitalWrite(led1_pin, HIGH);
    led1_state = true;
    vTaskDelay(led1_rate / portTICK_PERIOD_MS);
    digitalWrite(led1_pin, LOW);
    led1_state = false;
    vTaskDelay(led1_rate / portTICK_PERIOD_MS);
  }
}
void blinkLED_Core1(void *parameter) {
  while (1) {
    digitalWrite(led2_pin, HIGH);
    led2_state = true;
    vTaskDelay(led2_rate / portTICK_PERIOD_MS);
    digitalWrite(led2_pin, LOW);
    led2_state = false;
    vTaskDelay(led2_rate / portTICK_PERIOD_MS);
  }
}
//*****************************************************************************
// Setup
void setup() {
  Serial.begin(115200);
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  Serial.println("--- Dual-Core LED Toggle with Status Monitor ---");
  pinMode(led1_pin, OUTPUT);
  pinMode(led2_pin, OUTPUT);
  // Task for LED 1 on Core 0
  xTaskCreatePinnedToCore(
    blinkLED_Core0,
    "Blink LED 1",
    1024,
    NULL,
    1,
    NULL,
    core_0);
  // Task for LED 2 on Core 1
  xTaskCreatePinnedToCore(
    blinkLED_Core1,
    "Blink LED 2",
    1024,
    NULL,
    1,
    NULL,
    core_1);
}
//*****************************************************************************
// Main Loop
void loop() {
}