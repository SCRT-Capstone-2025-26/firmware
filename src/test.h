#ifndef TEST_H
#define TEST_H

#include <ArduinoEigen.h>

// NOTE: These use micros and so cannot handle times greater than about ~70 minutes

void get_acc(Eigen::Vector3f *data);
void get_hg_acc(Eigen::Vector3f *data);

void get_gyro(Eigen::Vector3f *data);

void get_baro(float *pressure, float *temperature);

bool get_reboot();

#endif
