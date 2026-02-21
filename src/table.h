#ifndef TABLE_H
#define TABLE_H

#include <cstddef>

// TODO: Give these better names

// The is table is a lookup table for the areobrake extension given the
//  height and velocity. It takes advantage of the fact that the extension
//  to velocity is monotonic for constant height. We it only stores an array
//  of rows index by height where each row contains an array of floats
//  these floats are the velocities that correspond to a given extension
//  store in the ROW_WEIGHTS table. So TABLE[height][i] has a velocity
//  the corresponds to ROW_WEIGHTS[i]. This allows for the table
//  to be small and still store data. Accessing the table
//  just interpolates between the two height values and for each of those
//  interpolates the two closest velocity values.

// NOTE: If height or velocity is outside the table the code extrapolates linearly from the table edges
// NOTE: This means it can return an value and should be clamped after indexing

const extern float TABLE[];
const extern float ROW_WEIGHTS[];

const extern size_t TABLE_SIZE;
const extern size_t ROW_SIZE;

const extern float HEIGHT_STEP;
const extern float HEIGHT_START;

float index_table(float height, float velocity);

#endif
