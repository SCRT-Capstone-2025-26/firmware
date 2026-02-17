#ifndef LOGGING_H
#define LOGGING_H

#include <variant>
#include <ISM6HG256XSensor.h>

#include "util.h"

struct ModeChange {
  BoardMode old;
  BoardMode next;
};

struct Reading {
  ISM6HG256X_Axes_t acc_axis;
  ISM6HG256X_Axes_t gyro_axis;
};

typedef std::variant<String, ModeChange> Message;

void log_message(Message &&content);

void write_readings(Reading &&readings);

bool wait_log_boot();

#endif

