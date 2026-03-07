#ifndef LOGGING_H
#define LOGGING_H

#include <variant>
#include <ISM6HG256XSensor.h>

#include "util.h"

struct ModeChange {
  BoardMode old;
  BoardMode next;
};

struct Error {
  String content;
};

// Arduino doesn't have a simple 64 bit hex to string so it is a tiny bit complex ot generate the string
struct BoardID {
  uint64_t id;
};

typedef std::variant<String, Error, ModeChange, BoardID> Message;

struct __attribute__((packed)) Acc {
  float x;
  float y;
  float z;
};

struct __attribute__((packed)) Gyro {
  float x;
  float y;
  float z;
};

struct __attribute__((packed)) Baro {
  float pressure;
  float temperature;
};

struct __attribute__((packed)) Servo {
  float percent;
};

struct __attribute__((packed)) Current {
  uint16_t voltage;
  int32_t temp;
  int32_t current;
  uint32_t power;
};

// The data should be packed as it it written directly to a buffer
typedef std::variant<Acc, Gyro, Baro, Servo, Current> Data;

extern std::atomic<bool> flash_ready;

void log_message(Message &&content);

void write_data(Data &&data);

bool wait_log_boot();

#endif

