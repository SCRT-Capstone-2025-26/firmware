#ifdef TEST
#include "test.h"
#include "test_data.h"

#include "util.h"

#include <cstddef>
#include <cstdint>
#include <cstring>

typedef struct Index {
  size_t low;
  float raw;
} Index;

Index time_index(Micros time) {
  Index index;
  index.raw = (float)time / micros_per_step;
  index.low = min(time / micros_per_step, max_steps - 2);

  return index;
}

float interp(Index index, const float *buf) {
  return linear_interp(index.raw, index.low, index.low + 1, buf[index.low], buf[index.low + 1]);
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
  Index index = time_index(time);

  data->x() = interp(index, acc_x_data) + random(acc_noise);
  data->y() = interp(index, acc_y_data) + random(acc_noise);
  data->z() = interp(index, acc_z_data) + random(acc_noise);
}

void get_hg_acc(Eigen::Vector3f *data, Micros time) {
  Index index = time_index(time);

  data->x() = interp(index, hg_acc_x_data) + random(hg_acc_noise);
  data->y() = interp(index, hg_acc_y_data) + random(hg_acc_noise);
  data->z() = interp(index, hg_acc_z_data) + random(hg_acc_noise);
}

void get_gyro(Eigen::Vector3f *data, Micros time) {
  Index index = time_index(time);

  data->x() = interp(index, gyro_x_data) + random(gyro_noise);
  data->y() = interp(index, gyro_y_data) + random(gyro_noise);
  data->z() = interp(index, gyro_z_data) + random(gyro_noise);
}

// The temperature will be slightly off using this since they are actually sampled at different times
void get_baro(float *pressure, float *temperature) {
  Index index = time_index(micros());

  *pressure = interp(index, pres_data) + random(baro_noise);
  // Currently no temp noise
  *temperature = interp(index, temp_data);
}

bool get_reboot() {
  return micros() >= max_steps * micros_per_step;
}

bool init_debug() {
  // Get the start state
  read_debug();

  // If we are already rebooting then the init failed
  return !get_reboot();
}

void read_debug() {
  char desc = Serial.read();
  switch (desc) {
    case 'S':
      break;
    case 'D':
      break;
    default:
      log_message("Debug read failure");
      break;
  }
}
#endif

