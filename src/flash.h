#ifndef FLASH_H
#define FLASH_H

#include <hardware/flash.h>
#include <cstdint>

// NOTE: I think we could use the uinitialized RAM macro for a watchdog reboot

// This may need to be compressed
struct State {
  uint8_t valid;
  float h;
  float v;
  // We save a spot on the covariance matrix because it is symmetric
  float h_cov;
  float v_cov;
  float hv_cov;
} __attribute__((packed));

// Finds where the flash buffer was last updated and inits in the flash
// index to that. Also returns that last_state or flash if 0
bool flash_reinit(State *last_state);

// NOTE: This will be quite slow
bool clear_flash_buf();

// NOTE: This can only be called on the main CPU and forces the other CPU into lockout mode
//  it also disables interrupts while running
// NOTE: This will return false if called before clear_flash_buf or flash_reinit
bool flash_push_state(State &&state);

#endif

