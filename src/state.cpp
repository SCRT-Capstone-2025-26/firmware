#include "state.h"
#include "util.h"

#include "logging.h"
#include <cmath>

// NOTE: These operations use the eigen math library and could be manually optimized in some cases,
//  however, the compiler should handle a lot with inlining and if they need to be optimized later they can
// NOTE: We could calibrate the sensors in RestState, instead of pre calibrating them (this neither calibration
//  version has been implmented yet)

void FlightState::push_baro(float pressure, float temperature, float sample_rate) {
}

void FlightState::push_acc(ISM6HG256X_Axes_t &acc, float sample_rate) {
  // This is just simple integration (not even like trapezoidal)
  // This will be changed

  pos += vel * sample_rate;

  Eigen::Vector3f acc_vec(acc.x, acc.y, acc.z);
  vel += (rot * acc_vec) * sample_rate;
}

void FlightState::push_gyro(ISM6HG256X_Axes_t &gyro, float sample_rate) {
  // See https://stackoverflow.com/questions/23503151/how-to-update-quaternion-based-on-3d-gyro-data
  // I think this is based on the approximation sin(x) == x
  Eigen::Quaternionf w(0, gyro.x * GYRO_TO_RADPS, gyro.y * GYRO_TO_RADPS, gyro.z * GYRO_TO_RADPS);
  rot.coeffs() += 0.5 * sample_rate * (rot * w).coeffs();
  rot.normalize();
}

float FlightState::get_servo() {
  return 0.0f;
}

bool FlightState::done() {
  // I believe IREC requires no flight controls at 30 degrees
  // We know that (0.0f, 0.0f -1.0f is up from the local frame
  // So transforming our local up to the global frame
  // So we see if the angle between true up and our local up is more than 30 degrees
  Eigen::Vector3f local_up(0.0f, 0.0f, -1.0f);
  Eigen::Vector3f up(0.0f, 1.0f, 0.0f);
  Eigen::Vector3f local_up_in_global = rot * local_up;

  // Since both are unit vectors we can use dot product to compute the cosine between them
  // Hopefully cos gets optimized
  if (up.dot(local_up_in_global) < std::cos(30.0f * DEG_TO_RAD)) {
    return true;
  }

  return false;
}

void RestState::push_baro(float pressure, float temperature, float sample_rate) {
}

void RestState::push_acc(ISM6HG256X_Axes_t &acc, float sample_rate) {
  // We want to find that q such that when we rotate an accelerometer reading by q
  // it gets transformed into the coordinate frame where y is up
  // We don't care about what the gyro says in the rest state sense we have a method to determine the absolute rotation
  // TODO: Take multiple samples to determine rotation (this depends on how we read the imu)

  Eigen::Vector3f acc_vec(acc.x, acc.y, acc.z);
  // This is the vector that gravity actually points when the board is facing up (the accelerometer is mounted at an angle)
  // We don't care about magnitude since it is a direction
  Eigen::Vector3f down(0.0f, -1.0f, 0.0f);

  // The rotation that takes acc and turns it into down
  rot = Eigen::Quaternionf::FromTwoVectors(acc_vec, down);

  acceleration = acc_vec.norm();

  inited = true;
}

void RestState::push_gyro(ISM6HG256X_Axes_t &gyro, float sample_rate) {
}

bool RestState::try_init_flying(FlightState &state) {
  if (!inited) {
    return false;
  }

  if (abs(acceleration - GRAVITY_ACC) > 60) {
    state.rot = rot;
    state.vel = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
    state.pos = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

    return true;
  }

  return false;
}

bool RestState::try_init_flying_boot(FlightState &state) {
  return try_init_flying(state);
}

