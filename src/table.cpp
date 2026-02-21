#include "table.h"
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
// Binary searches a velocity row and then interpolates the given extension
// Since this uses linear_interp if the value is out of the bounds
//  it just becomes a linear_interp of the two values closest to that edge
float bin_search(float row[], float velocity) {
  size_t low = 0;
  size_t high = ROW_SIZE - 1;

  // Binrary search for the bounds around the current velocity
  while (low + 1 != high) {
    size_t index = (low + high) / 2;

    if (velocity < row[index]) {
      high = index;
    } else {
      low = index;
    }
  }

  // Interpolate using the found bounds
  // velocity could be lower than row[low] or higher than row[high]
  //  in which case row[low] and row[high] are not bounds around velocity and
  //  this becomes a linear extrapolation from the edge cases
  return linear_interp(velocity, row[low], row[high], ROW_WEIGHTS[low], ROW_WEIGHTS[high]);
}

// This could be optimized a bit
float index_table(float height, float velocity) {
  // The * (1.0f / x) is for gcc to optimize the div
  // Find bounds of the table clamped to be valid
  ssize_t raw_low_index = (ssize_t)((height - HEIGHT_START) * (1.0f / HEIGHT_STEP));
  size_t low_index = min(max(raw_low_index, 0), TABLE_SIZE - 1);
  size_t high_index = low_index + 1;

  // Find the table entries at each row
  float lower = bin_search(TABLE[low_index * ROW_SIZE], velocity);
  float upper = bin_search(TABLE[low_index * ROW_SIZE], velocity);

  // Find the lower and upper height bound
  float low_height = (low_index * HEIGHT_STEP) + HEIGHT_START;
  float high_height = (high_index * HEIGHT_STEP) + HEIGHT_START;

  // Interpolate the result
  // As mentioned in bin_search this will linearly extrapolate if height is not
  //  in the table and bounded by low_index and high_index
  return linear_interp(height, low_height, high_height, lower, upper);
}

