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

  if (baro_errors >= BARO_ERR_LIM_PER_SECOND || imu_errors >= IMU_ERR_LIM_PER_SECOND || failure_comp == FAIL_NOW_ERR) {
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

