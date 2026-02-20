#include "util.h"

#include <hardware/watchdog.h>

bool sleep(Millis target_time) {
  watchdog_update();

  while (target_time > (WATCHDOG_MS / 2)) {
    delay(WATCHDOG_MS / 2);
    watchdog_update();
  }

  delay(target_time);

  watchdog_update();
  return in_future;
}

// Returns false if the time is in the past
bool sleep_to(Millis target_time) {
  Millis curr = millis();

  // Since millis is unsigned we have to check for overflow
  bool in_future = curr <= target_time; // In the future or now
  if (in_future) { sleep(target_time - curr); }

  return in_future;
}

