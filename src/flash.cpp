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

#define FLASH_SAFE_TIMEOUT 3

// This is the desired buffer size
#define BUF_MAX_MEM    (1024 * 1024 * 2)
// This will align to FLASH_PAGE_SIZE since FLASH_SECTOR_SIZE is a multiple of page size
#define FLASH_ALIGN    FLASH_SECTOR_SIZE
// This is the buffer size truncated to make sure that it is aligned to the flash pages
#define BUF_MEM        (BUF_MAX_MEM & ~(FLASH_ALIGN - 1))
// This is the number of elements we can actually fit in the buffer
#define BUF_ELEMS      (BUF_MEM / sizeof(State))

struct WriteArgs {
  // In flash space (ie & - XIP_BASE)
  void *memory;
  uint8_t *page;
  size_t size;
};

alignas(FLASH_ALIGN) __in_flash("flash_buf") State flash_buf[BUF_ELEMS];
size_t flash_index = INVALID_FLASH_INDEX;

// Binary searches the array for the last valid entry
bool flash_reinit(State *state) {
  size_t low = -1;
  size_t high = BUF_ELEMS;

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

void _clear_flash_buf(void *_) {
  // A potentially slow call
  // It has no return so we just have to trust the function
  // BUF_MEM is correctly size see above
  flash_range_erase((uint32_t)&flash_buf - XIP_BASE, BUF_MEM);
}

bool clear_flash_buf() {
  // Check the logging core is ready for a flash write
  if (!flash_ready) {
    return false;
  }

  flash_safe_execute(_clear_flash_buf, NULL, FLASH_SAFE_TIMEOUT);

  flash_index = 0;
  return true;
}

void _flash_write(void *_args) {
  WriteArgs args = *(WriteArgs *)_args;

  // It has no return so we just have to trust the function
  flash_range_program((uint32_t)args.memory, args.page, args.size);
}

bool flash_push_state(State &&state) {
  if (flash_index == INVALID_FLASH_INDEX || flash_index == BUF_ELEMS) {
    return false;
  }

  // Check the logging core is ready for a flash write
  if (!flash_ready) {
    return false;
  }

  // This code assumes this is true
  static_assert(FLASH_PAGE_SIZE >= sizeof(State), "State must fit in a flash page");

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
  bool one_page = in_page_offset + sizeof(State) <= FLASH_PAGE_SIZE;
  size_t write_size = one_page ? FLASH_PAGE_SIZE : (FLASH_PAGE_SIZE * 2);

  // Copy into ram to modify the flash only where we want to
  memcpy(page, page_addr, write_size);
  // Add to our new flash copy
  memcpy(page + in_page_offset, &state, write_size);

  args.memory = page_addr;
  args.page   = page;
  args.size   = write_size;
  return flash_safe_execute(_flash_write, page, FLASH_SAFE_TIMEOUT) == PICO_OK;
}


