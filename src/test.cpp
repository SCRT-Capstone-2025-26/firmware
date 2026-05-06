#include "test.h"

#include "util.h"
#include "logging.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <variant>
#include <CircularBuffer.hpp>

#define COMMAND_BUF_SIZE 256

// This is maybe too big and slow
// NOTE: It would be faster to have a buffer for each type of data
//  like acc, hg_acc, baro ... This would make communication faster
//  if it is needed
struct __attribute__((packed)) StateCommand {
  Micros time;

  float acc_x;
  float acc_y;
  float acc_z;

  float acc_x_noise;
  float acc_y_noise;
  float acc_z_noise;

  float hg_acc_x;
  float hg_acc_y;
  float hg_acc_z;

  float hg_acc_x_noise;
  float hg_acc_y_noise;
  float hg_acc_z_noise;

  float gyro_x;
  float gyro_y;
  float gyro_z;

  float gyro_x_noise;
  float gyro_y_noise;
  float gyro_z_noise;

  float pressure;
  float temperature;

  float pressure_noise;
  float temperature_noise;
};

struct __attribute__((packed)) DoneCommand {
  Micros time;
};

// The data should be packed as it it written directly to a buffer
typedef std::variant<StateCommand, DoneCommand> Command;

typedef struct IndexedState {
  StateCommand c1;
  StateCommand c2;
} IndexedState;

#define INTERP_INDEX(name) linear_interp(time, index.c1.time, index.c2.time, index.c1.name, index.c2.name) + \
  random(linear_interp(time, index.c1.time, index.c2.time, index.c1.name##_noise, index.c2.name##_noise))

std::atomic_bool done = false;
std::atomic<Micros> done_time = 0;
// Using a non-locking circular buffer may be better
semaphore_t buf_sem;
CircularBuffer<StateCommand, COMMAND_BUF_SIZE> buf;

// buf must not be empty
IndexedState time_index(Micros time) {
  IndexedState index;

  sem_acquire_blocking(&buf_sem);
  // Binary search is now cringe mostly cause this buffer contains way more
  //  non-important states since it only gets cleared when full
  for (size_t i = 0; i < buf.size(); i++) {
    if (buf[i].time >= time) {
      index.c1 = buf[i];
      index.c2 = buf[max(i, 1) - 1];
    }
  }
  sem_release(&buf_sem);

  return index;
}

// More advanced random should be used in the future
float random(float scale) {
  // Apparently the first sample can be time consumeing
  uint64_t rand = get_rand_64();
  // Apparently divsion by large ints causes "bias"
  //  so I did this. I don't know if it is better
  uint32_t f = 0;
  // It generates a random mantissa
  f |= rand & ((1 << 23) - 1);
  // Then generates a power that is 2^1
  f |= 128 << 23;
  // Now f is random number from 2 to 4

  // Then cast (avoiding undefined behaviour)
  float ret;
  memcpy(&ret, &f, sizeof(float));
  // Zero it and scale
  return (ret - 3.0f) * scale;
}

void get_acc(Eigen::Vector3f *data, Micros time) {
  IndexedState index = time_index(time);

  data->x() = INTERP_INDEX(acc_x);
  data->y() = INTERP_INDEX(acc_y);
  data->z() = INTERP_INDEX(acc_z);
}

void get_hg_acc(Eigen::Vector3f *data, Micros time) {
  IndexedState index = time_index(time);

  data->x() = INTERP_INDEX(hg_acc_x);
  data->y() = INTERP_INDEX(hg_acc_y);
  data->z() = INTERP_INDEX(hg_acc_z);
}

void get_gyro(Eigen::Vector3f *data, Micros time) {
  IndexedState index = time_index(time);

  data->x() = INTERP_INDEX(gyro_x);
  data->y() = INTERP_INDEX(gyro_y);
  data->z() = INTERP_INDEX(gyro_z);
}

// The temperature will be slightly off using this since they are actually sampled at different times
void get_baro(float *pressure, float *temperature) {
  Micros time = micros();
  IndexedState index = time_index(time);

  *pressure = INTERP_INDEX(pressure);
  *temperature = INTERP_INDEX(temperature);
}

bool get_reboot() {
  return done && micros() >= done_time;
}

void init_debug() {
  sem_init(&buf_sem, 1, 1);

  // Get the start state
  while (buf.size() == 0) {
    read_debug();
  }
}

void read_debug() {
  char desc = Serial.peek();
  switch (desc) {
    case 'S':
      StateCommand state_command;
      // The serial buffer is expanded in TEST mode so state_command can fit
      if (Serial.available() < sizeof(state_command) + 1) {
        break;
      }

      Serial.read();
      Serial.readBytes((char *)&state_command, sizeof(state_command));

      sem_acquire_blocking(&buf_sem);
      buf.unshift(state_command);
      sem_release(&buf_sem);

      break;
    case 'D':
      DoneCommand done_command;
      if (Serial.available() < sizeof(done_command) + 1) {
        break;
      }

      Serial.read();
      Serial.readBytes((char *)&done_command, sizeof(done_command));

      done_time = done_command.time;
      done = true;

      break;
    default:
      log_message("Debug read failure");
      break;
  }
}

