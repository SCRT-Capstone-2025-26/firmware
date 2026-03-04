#ifndef FLASH_H
#define FLASH_H

#include <hardware/flash.h>
#include <pico/flash.h>
#include <cstdint>

// NOTE: I think we could use the uinitialized RAM macro for a watchdog reboot

// This may need to be compressed
struct State {
  uint8_t valid = 0;
  float h;
  float v;
  // We save a spot on the covariance matrix because it is symmetric
  float h_cov;
  float v_cov;
  float hv_cov;

  State(float h, float v, float h_cov, float v_cov, float hv_cov) : h(h), v(v), h_cov(h_cov), v_cov(v_cov), hv_cov(hv_cov) {
  }

  State() : h(0.0f), v(0.0f), h_cov(0.0f), v_cov(0.0f), hv_cov(0.0f) {
  }
} __attribute__((packed));

// This is the buffer size (it is 2 MiB)
// TODO: Use a script to inject these constants into the compilation
// NOTE: This comes from the filesystem size in platformio.ini if that changes this could cause
//  nasty problems
#define FS_SIZE         2097152
#define BUF_MEM         FS_SIZE
// NOTE: This comes from the maximum_size in beavs.json in upload if that changes this could cause
//  nasty problems
#define FLASH_SIZE      4194304
// This is the number of elements we can actually fit in the buffer
#define FLASH_BUF_ELEMS (BUF_MEM / sizeof(State))

static_assert(BUF_MEM % FLASH_SECTOR_SIZE == 0, "Data buffer must be divisible by FLASH_SECTOR_SIZE");
// Technically this follows from the first condition
static_assert(BUF_MEM % FLASH_PAGE_SIZE == 0, "Data buffer must be divisible by FLASH_PAGE_SIZE");
static_assert(BUF_MEM <= FS_SIZE, "Data buffer must fit in the FS");

// Finds where the flash buffer was last updated and inits in the flash
// index to that. Also returns that last_state or flash if 0
bool flash_reinit(State *last_state);

// NOTE: This will be quite slow
// NOTE: This does feed to watchdog and let the other core run a bit, but it can't feed faster than it takes to
//  clear a sector
bool clear_flash_buf();

// NOTE: This can only be called on the main CPU and forces the other CPU into lockout mode
//  it also disables interrupts while running
// NOTE: This will return false if called before clear_flash_buf or flash_reinit
bool flash_push_state(State &&state);

// This overrides the weak version from the standard library and this is compatible the arduino earlephilower stuff
flash_safety_helper_t *get_flash_safety_helper(void);

#endif

