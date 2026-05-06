#ifndef TEST_H
#define TEST_H

#include "util.h"

#include <ArduinoEigen.h>

// NOTE: These use micros and so cannot handle times greater than about ~70 minutes

// Uses Serial
void init_debug();
void read_debug();

void get_acc(Eigen::Vector3f *data, Micros time);
void get_hg_acc(Eigen::Vector3f *data, Micros time);

void get_gyro(Eigen::Vector3f *data, Micros time);

void get_baro(float *pressure, float *temperature);

void get_reboot();

#endif
