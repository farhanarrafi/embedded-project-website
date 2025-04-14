/* Edge Impulse Arduino examples
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
/*

Model paparms: 20 epochs learning rate: 0.001
*/

/* Includes ---------------------------------------------------------------- */
#include <Vehicle_Detection_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

#include "esp_camera.h"

// Select camera model - find more camera models in camera_pins.h file here
// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer/camera_pins.h

#define CAMERA_MODEL_XIAO_ESP32S3  // Has PSRAM

#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 10
#define SIOD_GPIO_NUM 40
#define SIOC_GPIO_NUM 39

#define Y9_GPIO_NUM 48
#define Y8_GPIO_NUM 11
#define Y7_GPIO_NUM 12
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 16
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 17
#define Y2_GPIO_NUM 15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM 47
#define PCLK_GPIO_NUM 13

#define LED_GPIO_NUM 21


/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240
#define EI_CAMERA_FRAME_BYTE_SIZE 3



/* Private variables ------------------------------------------------------- */
static bool debug_nn = false;  // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
uint8_t *snapshot_buf;  //points to the output of the capture

static camera_config_t camera_config = {
  .pin_pwdn = PWDN_GPIO_NUM,
  .pin_reset = RESET_GPIO_NUM,
  .pin_xclk = XCLK_GPIO_NUM,
  .pin_sscb_sda = SIOD_GPIO_NUM,
  .pin_sscb_scl = SIOC_GPIO_NUM,

  .pin_d7 = Y9_GPIO_NUM,
  .pin_d6 = Y8_GPIO_NUM,
  .pin_d5 = Y7_GPIO_NUM,
  .pin_d4 = Y6_GPIO_NUM,
  .pin_d3 = Y5_GPIO_NUM,
  .pin_d2 = Y4_GPIO_NUM,
  .pin_d1 = Y3_GPIO_NUM,
  .pin_d0 = Y2_GPIO_NUM,
  .pin_vsync = VSYNC_GPIO_NUM,
  .pin_href = HREF_GPIO_NUM,
  .pin_pclk = PCLK_GPIO_NUM,

  //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
  .xclk_freq_hz = 20000000,
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,

  .pixel_format = PIXFORMAT_JPEG,  //YUV422,GRAYSCALE,RGB565,JPEG
  .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

  .jpeg_quality = 12,  //0-63 lower number means higher quality
  .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
  .fb_location = CAMERA_FB_IN_PSRAM,
  .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf);


/* Switch and LED definitions -------------------------------------------------------- */
#define RED_LED_PIN D9  // Red Light
#define ORA_LED_PIN D8  // Orange Light
#define GRE_LED_PIN D7  // Green Light
//#define LED_BUILTIN   // Pedestrian GO Light
#define PED_SWITCH_PIN D6  // Pedestrian Switch

#define G 0  // Green Light
#define Y 1  // Yellow Light
#define R 2  // Red Light
#define OFF 0
#define ON 1
#define CG 0  // Cars Go
#define CW 1  // Cars Wait
#define PG 2  // Pedestrians Go
#define PW 3  // Pedestrians Wait


struct state {
  unsigned char trafficLight;
  unsigned char pedLight;
  unsigned long delay;
  unsigned char next[4];  //index of the state
};
typedef struct state SType;
SType FSM[4] = {
  { G, OFF, 3000, { CG, CW, CW, CW } },  // Cars Go
  { Y, OFF, 2000, { PG, PG, CW, PG } },  // Cars Wait
  { R, ON, 5000, { PG, PG, PW, PG } },   // Pedestrians Go
  { G, OFF, 2000, { CG, CG, CG, CG } }   // Pedestrians Wait
};

unsigned long startTime = 0;

bool carDetected = false;
bool pedestrianPressed = false;

unsigned long blinkInterval = 1000;
unsigned long blinkStart = 0;
bool pedLedOn = false;

int STATE = 0;

/**
* @brief      Arduino setup function
*/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);


  //comment out the below line to start inference immediately after upload
  while (!Serial)
    ;
  Serial.println("Edge Impulse Inferencing Demo");
  if (ei_camera_init() == false) {
    ei_printf("Failed to initialize Camera!\r\n");
  } else {
    ei_printf("Camera initialized\r\n");
  }

  ei_printf("\nStarting continious inference in 2 seconds...\n");
  ei_sleep(2000);
  pinMode(RED_LED_PIN, OUTPUT);           // Set Red LED as output
  pinMode(ORA_LED_PIN, OUTPUT);           // Set Yellow LED as output
  pinMode(GRE_LED_PIN, OUTPUT);           // Set Green LED as output
  pinMode(LED_BUILTIN, OUTPUT);           // Set PedestrainLED as output
  pinMode(PED_SWITCH_PIN, INPUT_PULLUP);  // Set pedestrain switch as input with internal pull-up
  
  digitalWrite(RED_LED_PIN, LOW);
  digitalWrite(ORA_LED_PIN, LOW);
  digitalWrite(GRE_LED_PIN, LOW);
  digitalWrite(LED_BUILTIN, HIGH); // HIGH is OFF for Builtin LED

  startTime = millis();

  blinkStart = startTime;
  
  // set initial state
  int STATE = CG;
  // Update traffic light based on initial state
  //updateTrafficLights(STATE);
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/
void loop() {

  // if (digitalRead(PED_SWITCH_PIN) == HIGH) {  // If switch is pressed (LOW because of pull-up)
  //   digitalWrite(RED_LED_PIN, HIGH);          // Turn on LED
  // } else {
  //   digitalWrite(RED_LED_PIN, LOW);  // Turn off LED
  // }

  
  if (digitalRead(PED_SWITCH_PIN) == HIGH) {  // If switch is pressed (LOW because of pull-up)
    pedestrianPressed = true;
    ei_printf("pedestrianPressed: true\n");
    ei_printf("FSM[STATE].next[1]: %lu\n", FSM[STATE].next[1]);
  }
  unsigned long currentTime = millis();
  if (currentTime - startTime >= FSM[STATE].delay) {
    switch (STATE) {
      case CG:
        if (carDetected || pedestrianPressed) {
          STATE = FSM[STATE].next[1];
          carDetected = false;
        } else {
          STATE = FSM[STATE].next[0];
        }
        pedestrianPressed = false;
        ei_printf("STATE: CG\n");
        break;
      case CW:
        STATE = FSM[STATE].next[0];
        ei_printf("STATE: CW\n");
        break;
      case PG:
        if (pedestrianPressed) {
          STATE = FSM[STATE].next[0];
        } else {
          STATE = FSM[STATE].next[2];
        }
        ei_printf("STATE: PG\n");
        break;
      case PW:
        STATE = FSM[STATE].next[0];
        ei_printf("STATE: PW\n");
        break;
      default:
        break;
    }
    updateTrafficLights(STATE);
  }
  

  // while (true) {
  //   PORTB = FSM[S].out;
  //   wait(FSM[S], FSM[S].delay);
  //   Input = PORTE = &0x03;
  //   S = FSM[S].next[Input];
  // }

  // if (digitalRead(PED_SWITCH_PIN) == HIGH) {  // If switch is pressed (LOW because of pull-up)
  //   digitalWrite(RED_LED_PIN, HIGH);          // Turn on LED
  // } else {
  //   digitalWrite(RED_LED_PIN, LOW);  // Turn off LED
  // }

  // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
  if (ei_sleep(5) != EI_IMPULSE_OK) {
    return;
  }

  snapshot_buf = (uint8_t *)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

  // check if allocation was successful
  if (snapshot_buf == nullptr) {
    ei_printf("ERR: Failed to allocate snapshot buffer!\n");
    return;
  }

  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &ei_camera_get_data;

  if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
    ei_printf("Failed to capture image\r\n");
    free(snapshot_buf);
    return;
  }

  // Run the classifier
  ei_impulse_result_t result = { 0 };

  EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
  if (err != EI_IMPULSE_OK) {
    ei_printf("ERR: Failed to run classifier (%d)\n", err);
    return;
  }

  // print the predictions
  //ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
  //          result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
  bool bb_found = result.bounding_boxes[0].value > 0;
  for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
    auto bb = result.bounding_boxes[ix];
    if (bb.value == 0) {
      continue;
    }
    ei_printf("    %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\n", bb.label, bb.value, bb.x, bb.y, bb.width, bb.height);
    carDetected = true;
  }
  if (!bb_found) {
    //ei_printf("    No objects found\n");
  }
#else
  for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
    ei_printf("    %s: %.5f\n", result.classification[ix].label,
              result.classification[ix].value);
  }
#endif

#if EI_CLASSIFIER_HAS_ANOMALY == 1
  ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif


  free(snapshot_buf);
}



void blinkPedLight() {
  unsigned long currentMillis = millis();
  if (currentMillis - blinkStart >= blinkInterval) {
    blinkStart = currentMillis;     // Save the last time the LED toggled
    pedLedOn = !pedLedOn;
    digitalWrite(LED_BUILTIN, pedLedOn ? LOW : HIGH);    // Apply state
  }
}

void updateTrafficLights(int state) {
  ei_printf("inside updateTrafficLights()\n");
  //digitalWrite(LED_BUILTIN, FSM[STATE].pedLight == ON ? LOW : HIGH);
  if(FSM[STATE].pedLight == ON) {
    blinkPedLight();
  } else {
    //blinkStart = 0;
    pedLedOn = false;
    digitalWrite(LED_BUILTIN, HIGH);
  }
  switch (FSM[STATE].trafficLight) {
    case R:
      digitalWrite(RED_LED_PIN, HIGH);
      digitalWrite(ORA_LED_PIN, LOW);
      digitalWrite(GRE_LED_PIN, LOW);
      break;
    case G:
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(ORA_LED_PIN, LOW);
      digitalWrite(GRE_LED_PIN, HIGH);
      break;
    case Y:
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(ORA_LED_PIN, HIGH);
      digitalWrite(GRE_LED_PIN, LOW);
      break;
    default:
      digitalWrite(RED_LED_PIN, LOW);
      digitalWrite(ORA_LED_PIN, LOW);
      digitalWrite(GRE_LED_PIN, LOW);
      break;
  }
  startTime = millis();
}

/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void) {

  if (is_initialised) return true;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  //initialize the camera
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);       // flip it back
    s->set_brightness(s, 1);  // up the brightness just a bit
    s->set_saturation(s, 0);  // lower the saturation
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#elif defined(CAMERA_MODEL_ESP_EYE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
  s->set_awb_gain(s, 1);
#endif

  is_initialised = true;
  return true;
}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void) {

  //deinitialize the camera
  esp_err_t err = esp_camera_deinit();

  if (err != ESP_OK) {
    ei_printf("Camera deinit failed\n");
    return;
  }

  is_initialised = false;
  return;
}


/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           if ei_camera_frame_buffer is to be used for capture and resize/cropping.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
  bool do_resize = false;

  if (!is_initialised) {
    ei_printf("ERR: Camera is not initialized\r\n");
    return false;
  }

  camera_fb_t *fb = esp_camera_fb_get();

  if (!fb) {
    ei_printf("Camera capture failed\n");
    return false;
  }

  bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

  esp_camera_fb_return(fb);

  if (!converted) {
    ei_printf("Conversion failed\n");
    return false;
  }

  if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
      || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
    do_resize = true;
  }

  if (do_resize) {
    ei::image::processing::crop_and_interpolate_rgb888(
      out_buf,
      EI_CAMERA_RAW_FRAME_BUFFER_COLS,
      EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
      out_buf,
      img_width,
      img_height);
  }


  return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr) {
  // we already have a RGB888 buffer, so recalculate offset into pixel index
  size_t pixel_ix = offset * 3;
  size_t pixels_left = length;
  size_t out_ptr_ix = 0;

  while (pixels_left != 0) {
    out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix + 2];

    // go to the next pixel
    out_ptr_ix++;
    pixel_ix += 3;
    pixels_left--;
  }
  // and done!
  return 0;
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif