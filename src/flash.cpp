#include "flash.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <pico.h>
#include <stdalign.h>

#include "logging.h"

#define INVALID_FLASH_INDEX -1

// This must include a zero
#define VALID_FLASH_ENTRY 0

// This allows the other core to do a bit of work while the flash is being cleared
// We are not concerned about time sicne clearing is already slow
#define CLEAR_SLEEP_MS     2

struct WriteArgs {
  // In flash space (ie & - XIP_BASE)
  void *memory;
  uint8_t *page;
  size_t size;
};

// See https://arduino-pico.readthedocs.io/en/latest/platformio.html#flash-size for this
FlashState *flash_buf = (FlashState *)(FLASH_SIZE - FS_SIZE - 4096);
size_t flash_index = INVALID_FLASH_INDEX;

// Binary searches the array for the last valid entry
// TODO: Verify
bool flash_reinit(FlashState *state) {
  size_t low = -1;
  size_t high = FLASH_BUF_ELEMS;

  // Basic binary search
  while (low + 1 != high) {
    size_t index = (low + high) / 2;

    if (flash_buf[index].valid == VALID_FLASH_ENTRY) {
      low = index;
    } else {
      high = index;
    }
  }

  flash_index = low + 1;

  // Handles the edge case of no valid entries
  if (low == -1) {
    return false;
  }

  *state = flash_buf[low];
  return true;
}

void _clear_flash_buf(void *addr) {
  // A potentially slow call
  // It has no return so we just have to trust the function
  flash_range_erase((uint32_t)addr - XIP_BASE, FLASH_SECTOR_SIZE);
}

bool clear_flash_buf() {
  // Check the logging core is ready for a flash write
  if (!flash_ready) {
    return false;
  }

  // Since BUF_MEM is a multiple of FLASH_SECTOR_SIZE and _clear_flash_buf clears a SECTOR
  //  every call this is a valid loop
  for (size_t i = 0; i < BUF_MEM; i += FLASH_SECTOR_SIZE) {
    void *erase_mem = (void *)((size_t)&flash_buf + i);

    // We can check if the flash is already cleared and avoid wasting time
    bool cleared = true;
    // This could be faster with bigger blocks than bytes
    for (size_t j = 0; j < FLASH_SECTOR_SIZE; j++) {
      if (((uint8_t *)erase_mem)[j] != 0xFF) {
        cleared = false;
        break;
      }
    }

    if (!cleared) {
      flash_safe_execute(_clear_flash_buf, erase_mem, 0 /*The timeout_ms is not implemented anyway*/);
    }

    // This lets the other core run a bit and feeds the watchdog
    sleep(CLEAR_SLEEP_MS);
  }

  flash_index = 0;
  return true;
}

void _flash_write(void *_args) {
  WriteArgs args = *(WriteArgs *)_args;

  // It has no return so we just have to trust the function
  flash_range_program((uint32_t)args.memory, args.page, args.size);
}

bool flash_push_state(FlashState &&state) {
  if (flash_index == INVALID_FLASH_INDEX || flash_index >= FLASH_BUF_ELEMS) {
    return false;
  }

  // Check the logging core is ready for a flash write
  if (!flash_ready) {
    return false;
  }

  // This code assumes this is true
  static_assert(FLASH_PAGE_SIZE >= sizeof(FlashState), "Flash state must fit in a flash page");

  WriteArgs args;

  // We could write twice instead of having a double page but that creates issues if a write
  //  fails halfway this could of course still fail halfway, but there is nothing we
  //  can do about that at least
  uint8_t page[FLASH_PAGE_SIZE * 2];
  // Pointer arithmetic is cringe if it can be avoided
  void *addr_to_write = &flash_buf[flash_index];
  // This finds the page of the start of the new state
  //  to be added
  void *page_addr = (void *)((size_t)addr_to_write & ~(FLASH_PAGE_SIZE - 1));
  // The offset of the state in the page
  size_t in_page_offset = (size_t)page_addr - (size_t)addr_to_write;

  // Check if we can fit everything in one page if not we use two pages
  bool one_page = in_page_offset + sizeof(FlashState) <= FLASH_PAGE_SIZE;
  size_t write_size = one_page ? FLASH_PAGE_SIZE : (FLASH_PAGE_SIZE * 2);

  // Copy into ram to modify the flash only where we want to
  memcpy(page, page_addr, write_size);
  // Add to our new flash copy
  memcpy(page + in_page_offset, &state, write_size);

  args.memory = page_addr;
  args.page   = page;
  args.size   = write_size;
  return flash_safe_execute(_flash_write, page, 0 /*The timeout_ms is not implemented anyway*/) == PICO_OK;
}

// The overriden flash safety functions

uint32_t irq_state[NUM_CORES];

bool earle_core_init_deinit(bool init) {
  return true;
}

// NOTE: These functions do not implement the timeout_ms

int earle_enter_safe_zone_timeout_ms(uint32_t timeout_ms) {
  rp2040.idleOtherCore();

  irq_state[get_core_num()] = save_and_disable_interrupts();

  return PICO_OK;
}

int earle_exit_safe_zone_timeout_ms(uint32_t timeout_ms) {
  restore_interrupts_from_disabled(irq_state[get_core_num()]);

  rp2040.resumeOtherCore();

  return PICO_OK;
}

flash_safety_helper_t earle_flash_safety_helper = flash_safety_helper_t{
  .core_init_deinit = earle_core_init_deinit,
  .enter_safe_zone_timeout_ms = earle_enter_safe_zone_timeout_ms,
  .exit_safe_zone_timeout_ms = earle_exit_safe_zone_timeout_ms
};

flash_safety_helper_t *get_flash_safety_helper(void) {
  return &earle_flash_safety_helper;
}

