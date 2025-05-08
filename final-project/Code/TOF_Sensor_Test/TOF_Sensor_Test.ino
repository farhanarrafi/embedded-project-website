#include <vl53l8cx.h>


#ifdef ARDUINO_SAM_DUE
  #define DEV_I2C Wire1
#else
  #define DEV_I2C Wire
#endif
#define SerialPort Serial

#define LPN_PIN A3
#define PWREN_PIN 11

void print_result(VL53L8CX_ResultsData *Result);
void clear_screen(void);
void handle_cmd(uint8_t cmd);
void display_commands_banner(void);

// Components.
VL53L8CX tof_sensor(&DEV_I2C, LPN_PIN);

bool EnableAmbient = false;
bool EnableSignal = false;
uint8_t res = VL53L8CX_RESOLUTION_4X4;
char report[256];
uint8_t status;

/* Setup ---------------------------------------------------------------------*/
void setup()
{

  // Enable PWREN pin if present
  if (PWREN_PIN >= 0) {
    pinMode(PWREN_PIN, OUTPUT);
    digitalWrite(PWREN_PIN, HIGH);
    delay(10);
  }

  // Initialize serial for output.
  SerialPort.begin(460800);

  // Initialize I2C bus.
  DEV_I2C.begin();

  // Configure VL53L8CX component.
  tof_sensor.begin();
  status = tof_sensor.init();

  // Start Measurements
  status = tof_sensor.start_ranging();
}

void loop()
{
  VL53L8CX_ResultsData Results;
  uint8_t NewDataReady = 0;

  do {
    status = tof_sensor.check_data_ready(&NewDataReady);
  } while (!NewDataReady);

  if ((!status) && (NewDataReady != 0)) {
    status = tof_sensor.get_ranging_data(&Results);
    print_result(&Results);
  }

  if (Serial.available() > 0) {
    handle_cmd(Serial.read());
  }
  delay(100);
}

void print_result(VL53L8CX_ResultsData *Result)
{
  int8_t i, j, k, l;
  uint8_t zones_per_line;
  uint8_t number_of_zones = res;

  zones_per_line = (number_of_zones == 16) ? 4 : 8;

  display_commands_banner();

  SerialPort.print("Cell Format :\n\n");

  for (l = 0; l < VL53L8CX_NB_TARGET_PER_ZONE; l++) {
    snprintf(report, sizeof(report), " \033[38;5;10m%20s\033[0m : %20s\n", "Distance [mm]", "Status");
    SerialPort.print(report);

    if (EnableAmbient || EnableSignal) {
      snprintf(report, sizeof(report), " %20s : %20s\n", "Signal [kcps/spad]", "Ambient [kcps/spad]");
      SerialPort.print(report);
    }
  }

  SerialPort.print("\n\n");

  for (j = 0; j < number_of_zones; j += zones_per_line) {
    // for (i = 0; i < zones_per_line; i++) {
    //   SerialPort.print(" ---------------- ");
    // }
    // SerialPort.print("\n");

    // for (i = 0; i < zones_per_line; i++) {
    //   SerialPort.print("|                ");
    // }
    // SerialPort.print("|\n");

    for (l = 0; l < VL53L8CX_NB_TARGET_PER_ZONE; l++) {
      // Print distance and status
      for (k = (zones_per_line - 1); k >= 0; k--) {
        if (Result->nb_target_detected[j + k] > 0) {
          // snprintf(report, sizeof(report), "| \033[%3ld\033[0m : %3ld ",
          //          (long)Result->distance_mm[(VL53L8CX_NB_TARGET_PER_ZONE * (j + k)) + l],
          //          (long)Result->target_status[(VL53L8CX_NB_TARGET_PER_ZONE * (j + k)) + l]);
          // SerialPort.print(report);
          int zone_index = j + k;
          int data_index = (VL53L8CX_NB_TARGET_PER_ZONE * zone_index) + l;
          long distance = (long)Result->distance_mm[data_index];
          long status = (long)Result->target_status[data_index];
          snprintf(report, sizeof(report), "%5ld,%5ld,", distance, status);
          SerialPort.print(report);
        } else {
          //snprintf(report, sizeof(report), "| %3s  :  %3s ", "X", "X");
          snprintf(report, sizeof(report), "%ld,%ld,", "X", "X");
          SerialPort.print(report);
        }
      }
      SerialPort.print("\n");

      if (EnableAmbient || EnableSignal) {
        // Print Signal and Ambient
        for (k = (zones_per_line - 1); k >= 0; k--) {
          if (Result->nb_target_detected[j + k] > 0) {
            if (EnableSignal) {
              snprintf(report, sizeof(report), "| %3ld : ", (long)Result->signal_per_spad[(VL53L8CX_NB_TARGET_PER_ZONE * (j + k)) + l]);
              SerialPort.print(report);
            } else {
              snprintf(report, sizeof(report), "| %3s  :  ", "X");
              SerialPort.print(report);
            }
            if (EnableAmbient) {
              snprintf(report, sizeof(report), "%3ld ", (long)Result->ambient_per_spad[j + k]);
              SerialPort.print(report);
            } else {
              snprintf(report, sizeof(report), "%3s ", "X");
              SerialPort.print(report);
            }
          } else {
            snprintf(report, sizeof(report), "| %3s  :  %3s ", "X", "X");
            SerialPort.print(report);
          }
        }
        SerialPort.print("|\n");
      }
    }
  }
  for (i = 0; i < zones_per_line; i++) {
    SerialPort.print(" -----------------");
  }
  SerialPort.print("\n");
}

void toggle_resolution(void)
{
  status = tof_sensor.stop_ranging();

  switch (res) {
    case VL53L8CX_RESOLUTION_4X4:
      res = VL53L8CX_RESOLUTION_8X8;
      break;

    case VL53L8CX_RESOLUTION_8X8:
      res = VL53L8CX_RESOLUTION_4X4;
      break;

    default:
      break;
  }
  status = tof_sensor.set_resolution(res);
  status = tof_sensor.start_ranging();
}

void toggle_signal_and_ambient(void)
{
  EnableAmbient = (EnableAmbient) ? false : true;
  EnableSignal = (EnableSignal) ? false : true;
}

void clear_screen(void)
{
  snprintf(report, sizeof(report), "%c[2J", 27); /* 27 is ESC command */
  SerialPort.print(report);
}

void display_commands_banner(void)
{
  snprintf(report, sizeof(report), "%c[2H", 27); /* 27 is ESC command */
  SerialPort.print(report);

  Serial.print("53L8A1 Simple Ranging demo application\n");
  Serial.print("--------------------------------------\n\n");

  Serial.print("Use the following keys to control application\n");
  Serial.print(" 'r' : change resolution\n");
  Serial.print(" 's' : enable signal and ambient\n");
  Serial.print(" 'c' : clear screen\n");
  Serial.print("\n");
}

void handle_cmd(uint8_t cmd)
{
  switch (cmd) {
    case 'r':
      toggle_resolution();
      clear_screen();
      break;

    case 's':
      toggle_signal_and_ambient();
      clear_screen();
      break;

    case 'c':
      clear_screen();
      break;

    default:
      break;
  }
}
