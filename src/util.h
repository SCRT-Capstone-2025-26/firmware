#ifndef UTIL_H
#define UTIL_H

#include <Arduino.h>
#include <cmath>
#include <ArduinoEigen.h>

#include "led.h"

typedef unsigned long Millis;

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

#define LED_POSITIVE RGB(0, 1, 0)
#define LED_NEGATIVE RGB(1, 0, 0)
#define LED_DISABLE  RGB(0, 0, 0)
#define LED_NEUTRAL  RGB(0, 0, 1)

#define SERVO_CHARGE_MILLIS 2000
#define UNKNOWN_WAIT        2000
#define DEBUG_BOOT_DELAY    3000

#define SECONDS_TO_MILLIS 1000.0
#define GYRO_TO_RADPS     (0.001f * DEG_TO_RAD)

#define SERVO_FREQ  300.0f
#define SERVO_MIN   0.0f
#define SERVO_MAX   1.0f
// Flush extension is 6.35mm and there are 100.53fmm per rotation
// TODO: Check this
#define SERVO_FLUSH 6.35f / 100.53f

// I don't know why these aren't provided as constants from the library
// It basically uses a if statement chain on a bunch of floats to figure out
//  the setting which is strange https://github.com/stm32duino/ISM6HG256X/blob/main/src/ISM6HG256XSensor.cpp#L2705
#define GYRO_RATE 960.0f
#define ACC_RATE  960.0f

// TODO: Update these
#define GRAVITY_ACC       9.80665f
// This is low for testing
// The amount of acc from normal gravity required to consider
//  it a launch
#define LAUNCH_ACC        0.3f
// The number of samples that fit the launch criteria
//  to actually transition to launch
#define LAUNCH_SAMPLE_REQ 30
// A big history can easily take up a lot of the kinda limited ram
// The seconds of imu data to have in a rolling buffer so that after
//  launch is detected the first few moments of launch
// If this is big enough that it takes a while to compute
//  the code will have to change
#define LAUNCH_HIST_S     0.4f
// We need to determine the rotation before launch from
//  some accelerometer data so we put that in the circular buffer as well
#define ROT_HIST_SAMPLES  30

const Eigen::Vector3f LOCAL_UP(0.0f, 0.0f, -1.0f);

bool delay_to(Millis target_time);

#endif

