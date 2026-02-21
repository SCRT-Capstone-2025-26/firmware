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

struct AccCalib {
  ISM6HG256X_Axes_t acc_axis;
};

struct GyroCalib {
  ISM6HG256X_Axes_t gyro_axis;
};

typedef std::variant<AccCalib, GyroCalib> CalibData;

void log_message(Message &&content);

void write_calib(CalibData &&data);

bool wait_log_boot();

#endif

