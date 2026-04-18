#ifndef TABLE_H
#define TABLE_H

#include <cstddef>

// The lookup table is "compressed" by having the table
//  be indexed by (heights, extensions) and is populated
//  by the velocity at the given height and extension that
//  will reach the target. So we can index the table by 
//  height then find the velocity that is closest to our
//  current velocity and then we know the extension that
//  will work. The code actually finds the 4 closest
//  values and interpolates.

// NOTE: If height or velocity is outside the table the code extrapolates linearly from the table edges
// NOTE: This means it can return an value and should be clamped after indexing

float index_table(float height, float velocity);

#endif
