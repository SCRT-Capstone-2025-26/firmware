#include "table.h"
#include "table_data.h"

#include "util.h"

#include <cstddef>
#include <sys/types.h>

// See table.h for an explanation of the table

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

// Index the table by heigth and angle this give the start
//  of a continous row of velocities mapped to the extensions
size_t index_row(size_t height, size_t angle) {
  return ((height * ANGLES_SIZE) + angle) * EXTENSIONS_SIZE;
}

// Given the index of a row this returns the extension for the given
//  velocity on that row
// This will linearly extend if the velocity is out of bounds
float search_vel(size_t row_index, float velocity) {
  // Find the extension indices
  size_t vi0 = bin_search(&LOOKUP[row_index], velocity, EXTENSIONS_SIZE);
  size_t vi1 = vi0 + 1;

  // Interpolate between the indices the value is between
  // NOTE: This assumes there is are no velocities on the row with the same values for different indices
  //  this is check in the table_gen.py
  return linear_interp(velocity, LOOKUP[row_index + vi0], LOOKUP[row_index + vi1], EXTENSIONS[vi0], EXTENSIONS[vi1]);
}

// This finds the extension for a give height, angle, and velocity
//  in the table interpolating all those values
// The amount of interpolations makes this somewhat expensive
// This will linearly extend if any of the values is out of bounds
//  for the lookup
float index_table(float height, float angle, float velocity) {
  // Finds the height indices closest
  size_t hi0 = bin_search(HEIGHTS, height, HEIGHTS_SIZE);
  size_t hi1 = hi0 + 1;

  // Finds the angle indices closest
  size_t ai0 = bin_search(ANGLES, angle, ANGLES_SIZE);
  size_t ai1 = ai0 + 1;

  // Finds the extension corresponding to all 4 closest points
  float e00 = search_vel(index_row(hi0, ai0), velocity);
  float e10 = search_vel(index_row(hi1, ai0), velocity);
  float e01 = search_vel(index_row(hi0, ai1), velocity);
  float e11 = search_vel(index_row(hi1, ai1), velocity);

  // Interpolates between the four first interpolating the heights then angle
  // NOTE: This assumes there is are no heights or angles that are the same for different indices
  //  this is check in the table_gen.py
  float e0 = linear_interp(height, HEIGHTS[hi0], HEIGHTS[hi1], e00, e10);
  float e1 = linear_interp(height, HEIGHTS[hi0], HEIGHTS[hi1], e01, e11);
  return linear_interp(angle, ANGLES[ai0], ANGLES[ai1], e0, e1);
}

