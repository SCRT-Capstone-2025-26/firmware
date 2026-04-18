#include "table.h"
#include "table_data.h"

#include <cstddef>
#include <sys/types.h>

// See table.h for an explanation of the table

// Creates a line from the x0, x1, y0, y1 and then finds the y for the given x on that line
float linear_interp(float x, float x0, float x1, float y0, float y1) {
  float dx = x0 - x1;
  float dy = y0 - y1;

  float m = dy / dx;
  return (m * (x - x0)) + y0;
}

// It is quite possible this is slower than just normal iteration
// However, it won't matter (and it will be faster on the height table)
// The array must be of size greater than two and monotonic
// Which is checked by the build script
// This will always return values in bounds
size_t bin_search(const float arr[], float target, size_t size) {
  size_t low = 0;
  size_t high = size - 1;

  // Binary search for the bounds around the current target
  while (low + 1 != high) {
    size_t index = (low + high) / 2;

    if (target < arr[index]) {
      high = index;
    } else {
      low = index;
    }
  }

  // Return the value below target with low + 1 being above
  // The below and above may not be true if the value is outside
  //  the range of the array
  return low;
}

// This could be optimized a bit
// If the height or velocity is outside this code
//  will use the two closest indices instead of indices
//  around so the interpolation continues linearly
float index_table(float height, float velocity) {
  // Find the two height indices that our height is between
  size_t h0 = bin_search(HEIGHTS, height, HEIGHTS_SIZE);
  size_t h1 = h0 + 1;

  // Since the table is HEIGHTS_SIZE rows of EXTENSIONS_SIZE
  // We want to index the row representing the given height
  //  which is what these index
  size_t th0 = h0 * EXTENSIONS_SIZE;
  size_t th1 = h1 * EXTENSIONS_SIZE;

  // Now using the rows in the table we find the velocity
  //  they think we should be at and store these indices
  //  which represent indices in the EXTENSIONS array
  size_t e00 = bin_search(&LOOKUP[th0], velocity, EXTENSIONS_SIZE);
  size_t e01 = e00 + 1;
  size_t e10 = bin_search(&LOOKUP[th1], velocity, EXTENSIONS_SIZE);
  size_t e11 = e10 + 1;

  // Now we interpolate all four points together first the two extensions at the same height
  // Then across the heights
  float e0 = linear_interp(velocity, LOOKUP[th0 + e00], LOOKUP[th0 + e01], EXTENSIONS[e00], EXTENSIONS[e01]);
  float e1 = linear_interp(velocity, LOOKUP[th1 + e10], LOOKUP[th1 + e11], EXTENSIONS[e10], EXTENSIONS[e11]);
  return linear_interp(height, HEIGHTS[h0], HEIGHTS[h1], e0, e1);
}

