/**
* Toggles the LED on and off in its own task/thread.
*/
// Use only core 1 for demo purposes
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

// LED rates
static const int rate_1 = 500; // ms
static const int rate_2 = 323; // ms

// Pins
static const int led_pin = LED_BUILTIN;
// Our task: blink an LED

void toggleLED_1(void *parameter) {
  while (1) {
    digitalWrite(led_pin, HIGH);
    vTaskDelay(rate_1 / portTICK_PERIOD_MS);
    digitalWrite(led_pin, LOW);
    vTaskDelay(rate_1 / portTICK_PERIOD_MS);
  }
}

void toggleLED_2(void *parameter) {
  while (1) {
    digitalWrite(led_pin, HIGH);
    vTaskDelay(rate_2 / portTICK_PERIOD_MS);
    digitalWrite(led_pin, LOW);
    vTaskDelay(rate_2 / portTICK_PERIOD_MS);
  }
}


void setup() {
  // Configure pin
  pinMode(led_pin, OUTPUT);
  // Task to run forever
  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
    toggleLED_1,              // Function to be called
    "Toggle LED 2",           // Name of task
    1024,                   // Stack size (bytes in ESP32, words in FreeRTOS), 768
    NULL,                   // Parameter to pass to function, pointer!
    1,                      // Task priority (0 to configMAX_PRIORITIES - 1) 0-24
    NULL,                   // Task handle, managing (status, memory usage, terminate)
    app_cpu);               // Run on one core for demo purposes (ESP32 only)
  // If this was vanilla FreeRTOS, you'd want to call vTaskStartScheduler() in
  // main after setting up your tasks.
  xTaskCreatePinnedToCore(  // Use xTaskCreate() in vanilla FreeRTOS
    toggleLED_2,              // Function to be called
    "Toggle LED 2",           // Name of task
    1024,                   // Stack size (bytes in ESP32, words in FreeRTOS), 768
    NULL,                   // Parameter to pass to function, pointer!
    1,                      // Task priority (0 to configMAX_PRIORITIES - 1) 0-24
    NULL,                   // Task handle, managing (status, memory usage, terminate)
    app_cpu);               // Run on one core for demo purposes (ESP32 only)
  // If this was vanilla FreeRTOS, you'd want to call vTaskStartScheduler() in
  // main after setting up your tasks.
}
void loop() {
  // Do nothing
  // setup() and loop() run in their own task with priority 1 in core 1
  // on ESP32
}