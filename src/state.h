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

// TODO: Currently the units are really dumb and should be converted

struct FlightState {
  Eigen::Quaternionf rot;
  // Units in mg * ms
  Eigen::Vector3f vel;
  // Units in mg * ms * ms
  Eigen::Vector3f pos;

  FlightState() {}

  void push_baro(float pressure, float temperature, float sample_rate);
  void push_acc(ISM6HG256X_Axes_t &acc);
  void push_gyro(ISM6HG256X_Axes_t &gyro);

  float get_servo();

  bool done();
};

struct RestState {
  // TODO: Maybe have a barometer buffer

  // These hold the data collected by the imu because when the launch
  //  happens it will be detected a bit late due to filtering out false
  //  positives
  // This is a huge ram sink, but should be fine
  CircularBuffer<Vector3f, (uint16_t)(ACC_RATE * LAUNCH_HIST_S) + ROT_HIST_SAMPLES> acc_buf;
  CircularBuffer<Vector3f, (uint16_t)(GYRO_RATE * LAUNCH_HIST_S)> gyro_buf;

  // The number of samples in a row that have registered a launch
  int launch_samples = 0;

  RestState() {}

  void push_baro(float pressure, float temperature, float sample_rate);
  void push_acc(ISM6HG256X_Axes_t &acc);
  void push_gyro(ISM6HG256X_Axes_t &gyro);

  // Returns true if the rocket is flying and inits the flight state to that
  bool try_init_flying(FlightState &state);

  // Returns true if the rocket is flying and inits the flight state to that (called for some time when booting)
  // Is Currently the same as try_init_flying
  bool try_init_flying_boot(FlightState &state);
};

#endif

