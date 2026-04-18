#ifdef TEST
#include "test.h"
#include "test_data.h"

#include <cstddef>

// TODO: Maybe allow nonlinear index to save space

size_t time_index(Micros time) {
  return min(time / micros_per_step, max_steps - 1);
}

void get_acc(Eigen::Vector3f *data, Micros time) {
  size_t index = time_index(time);

  data->x() = acc_x_data[index];
  data->y() = acc_y_data[index];
  data->z() = acc_z_data[index];
}

void get_hg_acc(Eigen::Vector3f *data, Micros time) {
  size_t index = time_index(time);

  data->x() = hg_acc_x_data[index];
  data->y() = hg_acc_y_data[index];
  data->z() = hg_acc_z_data[index];
}

void get_gyro(Eigen::Vector3f *data, Micros time) {
  size_t index = time_index(time);

  data->x() = gyro_x_data[index];
  data->y() = gyro_y_data[index];
  data->z() = gyro_z_data[index];
}

// The temperature will be slightly off using this since they are actually sampled at different times
void get_baro(float *pressure, float *temperature) {
  size_t index = time_index(micros());

  *pressure = pres_data[index];
  *temperature = temp_data[index];
}

bool get_reboot() {
  return micros() >= max_steps * micros_per_step;
}
#endif

