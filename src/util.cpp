#include "util.h"

// Returns false if the time is in the past
bool delay_to(Millis target_time) {
  Millis curr = millis();

  // Since millis is unsigned we have to check for overflow
  bool in_future = curr <= target_time; // In the future or now
  if (in_future) { delay(target_time - curr); }

  return in_future;
}
