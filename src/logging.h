#ifndef LOGGING_H
#define LOGGING_H

#include <variant>
#include <ISM6HG256XSensor.h>

#include "util.h"

struct ModeChange {
  BoardMode old;
  BoardMode next;
};

typedef std::variant<String, ModeChange> Message;

struct __attribute__((packed)) AccCalib {
  float x;
  float y;
  float z;

  AccCalib(ISM6HG256X_Axes_t &acc) : x(acc.x), y(acc.y), z(acc.z) {
  }
};

struct __attribute__((packed)) GyroCalib {
  float x;
  float y;
  float z;

  GyroCalib(ISM6HG256X_Axes_t &gyro) : x(gyro.x), y(gyro.y), z(gyro.z) {
  }
};

// The data should be packed as it it written directly to a buffer
typedef std::variant<AccCalib, GyroCalib> CalibData;

void log_message(Message &&content);

void write_calib(CalibData &&data);

bool wait_log_boot();

#endif

