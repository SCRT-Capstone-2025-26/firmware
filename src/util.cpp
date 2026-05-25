#include "util.h"

#include <cstdint>
#include <hardware/watchdog.h>

#include "logging.h"

#define HALF_WATCHDOG_MS (WATCHDOG_MS / 2)

BoardMode board_mode = BOOTING;
Millis last_mode_change = 0;

// We count errors and the last time recieved
// this allows us to get how many erros have happend in the last second
Millis last_err_push = 0;
uint32_t imu_errors = 0;
uint32_t baro_errors = 0;

// Not super precise
// Feeds the watchdog while sleeping
void sleep(Millis target_time) {
  watchdog_update();

  // We enter a loop to feed the watchdog while sleeping
  while (target_time > HALF_WATCHDOG_MS) {
    target_time -= HALF_WATCHDOG_MS;
    delay(HALF_WATCHDOG_MS);
    watchdog_update();
  }

  delay(target_time);

  watchdog_update();
}

// Returns false if the time is in the past
// Feeds the watchdog while sleeping
bool sleep_to(Millis target_time) {
  Millis curr = millis();

  // Since millis is unsigned we have to check for overflow
  bool in_future = curr <= target_time; // In the future or now
  if (in_future) { sleep(target_time - curr); }

  return in_future;
}

void push_mode(BoardMode mode) {
  log_message(ModeChange{board_mode, mode});

  leds[LED_STATUS] = MODE_TO_COLOR[mode];
  led_show();

  board_mode = mode;
  last_mode_change = millis();
}

// Pushing LED_STATUS gets overwritten immediatly so is equivalent to a failure with no origin
void note_error(String &&message, FailComp failure_comp) {
  log_message(Error{message});

  // We remove errors one error per second
  Millis curr_time = millis();
  // DIV is cringe so we save a bit by using 1024
  uint32_t seconds = (last_err_push / 1024) - (curr_time / 1024);
  last_err_push = curr_time;

  // Remove the number of seconds from the error count
  if (baro_errors < seconds) { baro_errors = 0; } else { baro_errors -= seconds; }
  if (imu_errors < seconds) { imu_errors = 0; } else { imu_errors -= seconds; }

  switch (failure_comp) {
    case BARO_ERR:
      baro_errors++;
      leds[LED_BARO] = LED_NEGATIVE;
      led_show();
      break;

    case IMU_ERR:
      imu_errors++;
      leds[LED_IMU] = LED_NEGATIVE;
      led_show();
      break;

    default:
      break;
  }

  // If we are DONE we don't go to FAILURE since we don't need to
  //  and reboot could cause it to detect flight again when it shouldn't
  if (board_mode != DONE && (baro_errors >= BARO_ERR_LIM_PER_SECOND ||
                             imu_errors >= IMU_ERR_LIM_PER_SECOND ||
                             failure_comp == FAIL_NOW_ERR)) {
    push_mode(FAILURE);
  }
}

Millis millis_in_mode() {
  // This should never happen
  if (last_mode_change > millis()) {
    note_error("Mode changed marked in future", DO_NOTHING_ERR);
    return 0;
  }

  return millis() - last_mode_change;
}

bool is_after(Micros a, Micros b) {
  Micros delta = b - a;

  // This only works since Micros is unsigned and therefore implements modulo arithmetic
  // If the highest bit is set then delta must have underflowed so a > b
  //  this if a has overflowed and b hasn't because of modular arithmetic
  //  it will have the same result
  Micros high_bit = delta >> ((sizeof(Micros) * 8) - 1);
  return high_bit == 1;
}

// Creates a line from the x0, x1, y0, y1 and then finds the y for the given x on that line
// x0 should not be equal to x1
float linear_interp(float x, float x0, float x1, float y0, float y1) {
  float dx = x0 - x1;
  float dy = y0 - y1;

  float m = dy / dx;
  return (m * (x - x0)) + y0;
}

