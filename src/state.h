#ifndef STATE_H
#define STATE_H

#include "util.h"

#include <ISM6HG256XSensor.h>
#include <ArduinoEigen.h>
#include <CircularBuffer.hpp>
#include <cstdint>
#include <tuple>

// These classes define code that handles the rocket's physical state when in rest and in flight
// The rest state also detects the transisition to flying

// The flight state uses a Kalman filter to determine the rockets position
// Our lookup table currently only has two values world height and rocket forward velocity
// This make up our state [h, v] and it is important to note these are in seperate frames of reference
// We also track the angle with the gyro (for this and other conditions)
// To simplify the math we assume that the angle is relativitly stable. (This is also an assume of the lookup table)
// When the angle starts to change significantly it is already to late for beavs to have much control
// The rocket does launch at a bit of an angle so we do have to include the angle
// Since the accelerometer makes many observations and the barometer makes fewer, but is absolute we have
//  the accelerometer be a control input and the barometer be an observation
// The transformation matrix is [[1, cos(zenith) * dt], [0, 1]] (since we assume the angle is constant)
//  we don't change v based on an angle input (this could change in the future) and h is just v * cos(zenith) * dt
//  by simple trig. We use zenith, because it is convient to calculate from our rotation without a trig call.
// The control matrix is [0.5 * cos(zenith) * dt^2, dt] this is just trivial calc and trig
// Trivially the observation matrix is [1, 0] and the observation noise is just a matrix with only a value in the
//  upper right. This is set depending on some factors as there is a error spike when hitting around a mach.
//  Alsso currently this is not implemented, but there could be correlation in recently taken samples so
//  the error could be based on time

struct FlightState {
  Eigen::Quaternionf rot;

  // 0 is height in world frame, 1 is velocity in rocket frame
  Eigen::Vector2f state;
  // Noise of the state
  Eigen::Matrix2f cov;
  // The observation vector for the baro
  Eigen::RowVector2f obser = Eigen::Vector2f(1.0f, 0.0f);
  // We treat acceleration as a control
  // The first entry has to be updated when using since it depends on zenith
  Eigen::Vector2f control = Eigen::Vector2f(0.0f, 1.0f / ACC_RATE);
  // State transisition matrix (1, cos(zenith) * dt, 0, 1)
  // The top right value has to be updated when using since it depends on zenith
  Eigen::Matrix2f trans = Eigen::Matrix2f::Identity();
  // Noise from transisition
  Eigen::Matrix2f trans_noise = Eigen::Matrix2f::Identity();

  FlightState() {}

  void push_baro(float pressure, float temperature);
  void push_acc(Eigen::Vector3f &&acc);
  void push_gyro(Eigen::Vector3f &&gyro);

  float get_servo();

  bool done();
};

struct Measurement {
  Eigen::Vector3f data;
  bool is_acc;
};

struct RestState {
  const static uint16_t BUF_SIZE = (uint16_t)(ACC_RATE * LAUNCH_HIST_S) + (uint16_t)(GYRO_RATE * LAUNCH_HIST_S);
  // These hold the data collected by the imu because when the launch
  //  happens it will be detected a bit late due to filtering out false
  //  positives
  // This is a huge ram sink, but should be fine as we have enough and don't use dynamic allocation (except a few strings)
  CircularBuffer<Measurement, BUF_SIZE> buf;
  CircularBuffer<Eigen::Vector3f, ROT_HIST_SAMPLES> rot_calib_buf;

  // The number of samples in a row that have registered a launch
  int launch_samples = 0;

  RestState() {}

  void push_buf(Measurement &&meas);

  void push_baro(float pressure, float temperature);
  void push_acc(Eigen::Vector3f &&acc);
  void push_gyro(Eigen::Vector3f &&gyro);

  // Returns true if the rocket is flying and inits the flight state to that
  bool try_init_flying(FlightState &state);

  // Returns true if the rocket is flying and inits the flight state to that (called for some time when booting)
  // Is Currently the same as try_init_flying
  bool try_init_flying_boot(FlightState &state);
};

#endif

