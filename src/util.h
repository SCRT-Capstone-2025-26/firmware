#ifndef UTIL_H
#define UTIL_H

#include <Arduino.h>
#include <cmath>
#include <ArduinoEigen.h>

#include "led.h"

typedef unsigned long Millis;
typedef unsigned long Micros;

// Booting = The board is initializing components in the setup function (which could be run on a power failure or watchdog reboot)
// Unknown = Right after booting to determine if the board is flying right now or not
// Unarmed = The board demos the servo and then idles
// Armed   = The board is waiting for launch
// Flying  = The board is in activate control and trying to aerobrake to the target altitude
// Done    = The board is done active controls
// Failure = The board has reached an unrecoverable state

// Booting -> Unknown = After the boot sequence is done
// Unknown -> Unarmed = After enough time has elasped to determine if flight is happening and it hasn't been detected
// Unknown -> Flying  = After enough time has elasped to determine if flight is happening and it has been detected
// Unarmed -> Armed   = The arming pin has been detected
// Armed   -> Flight  = Flight has been detected
// FLight  -> Done    = The board is done with active controls (either because apogee was reached or competition rules require shutoff)
// *       -> Failure = An error that cannot be recovered from was detected

enum BoardMode {
  BOOTING,
  UNKNOWN,
  UNARMED,
  ARMED,
  FLYING,
  DONE,
  FAILURE
};

static String MODE_TO_NAME[] = {
  "Booting",
  "Unknown",
  "Unarmed",
  "Armed",
  "Flying",
  "Done",
  "Failure",
};

// The LEDS are visible with just a value of 1 out of 255, but not completly blinding

// The color scheme is roughly
// Red: Close to flight/in flight
// Green: In stable idle state
// Blue: Close to boot/in boot
static RGB MODE_TO_COLOR[] = {
  RGB(0, 0, 1),
  RGB(1, 0, 1),
  RGB(0, 1, 1),
  RGB(1, 1, 0),
  RGB(1, 0, 0),
  RGB(0, 1, 0),
  RGB(1, 1, 1),
};

// The component that failed with the special
// case of a fail now error and do nothing err
enum FailComp {
  FAIL_NOW_ERR,
  DO_NOTHING_ERR,
  BARO_ERR,
  IMU_ERR
};

#define LED_POSITIVE RGB(0, 1, 0)
#define LED_NEGATIVE RGB(1, 0, 0)
#define LED_DISABLE  RGB(0, 0, 0)
#define LED_NEUTRAL  RGB(0, 0, 1)

// Actually 1024ms for optmization
#define BARO_ERR_LIM_PER_SECOND 10
#define IMU_ERR_LIM_PER_SECOND  30

#define GYRO_TO_RADPS     (0.001f * DEG_TO_RAD)

#define ARM_ON  LOW
#define ARM_OFF HIGH

// Can't be lower due to barometer bug yet
#define WATCHDOG_MS             20
// Clearing flash is quite slow (the core does feed the watchdog while clearing)
//  but the minimum sector clear can be like 100ms I think at worst case it 
//  doesn't really matter since the flash is only cleared on the ground when booting
//  so we can have it be extra long
#define WATCHDOG_MS_CLEAR_FLASH 200

// The expected time beavs is useful in a flight
#define USEFUL_FLIGHT_TIME_MS 20 * 1000
#define FLASH_SAMPLE_RATE     (USEFUL_FLIGHT_TIME_MS / FLASH_BUF_ELEMS)

#define DEBUG_BOOT_DELAY 3000

extern BoardMode board_mode;
extern Millis last_mode_change;

extern uint32_t baro_errors;
extern uint32_t imu_errors;

// These functions are based on the arduino delay, but feed the watchdog
void sleep(Millis target_time);
bool sleep_to(Millis target_time);

void push_mode(BoardMode mode);
void note_error(String &&message, FailComp failure_comp);
Millis millis_in_mode();

#endif

