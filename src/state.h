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

// Position is has y perpendicular to the ground

// TODO: Explain filter somewhere

struct FlightState {
  Eigen::Quaternionf rot;

  // 0 is height in world frame, 1 is velocity in rocket frame
  Eigen::Vector2f state;
  // Noise of the state
  Eigen::Matrix2f cov;
  // The observation vector for the baro
  Eigen::Vector2f obser = Eigen::Vector2f(1.0f, 0.0f);
  // Noise from baro observations
  float obser_noise = 1.0f;
  // We treat acceleration as a control
  // The first entry has to be updated when using since it depends on zenith
  Eigen::Vector2f control = Eigen::Vector2f(0.0f, 1.0f / ACC_RATE);
  // Noise from acceleration
  Eigen::Matrix2f control_noise;
  // State transisition matrix (1, cos(zenith) * dt, 0, 1)
  // The top right value has to be updated when using since it depends on zenith
  Eigen::Matrix2f trans = Eigen::Matrix2f(1.0f, 0.0f, 0.0f, 1.0f);


  FlightState() {}

  void push_baro(float pressure, float temperature);
  void push_acc(Eigen::Vector3f &&acc);
  void push_gyro(Eigen::Vector3f &&gyro);

  float get_servo();

  bool done();
};

struct RestState {
  // These hold the data collected by the imu because when the launch
  //  happens it will be detected a bit late due to filtering out false
  //  positives
  // This is a huge ram sink, but should be fine
  CircularBuffer<Eigen::Vector3f, (uint16_t)(ACC_RATE * LAUNCH_HIST_S) + ROT_HIST_SAMPLES> acc_buf;
  CircularBuffer<Eigen::Vector3f, (uint16_t)(GYRO_RATE * LAUNCH_HIST_S)> gyro_buf;

  // The number of samples in a row that have registered a launch
  int launch_samples = 0;

  RestState() {}

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

