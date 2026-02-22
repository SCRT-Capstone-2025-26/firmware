#include "util.h"

#include <hardware/watchdog.h>

#define HALF_WATCHDOG_MS (WATCHDOG_MS / 2)

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

