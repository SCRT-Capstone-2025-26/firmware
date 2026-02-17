#ifndef STATE_H
#define STATE_H

#include "util.h"

#include <ISM6HG256XSensor.h>
#include <ArduinoEigen.h>

// These classes define code that handles the rocket's physical state when in rest and in flight
// The rest state also detects the transisition to flying

// Position is has y perpendicular to the ground

// TODO: Currently the units are really dumb and should be converted

struct FlightState {
  Eigen::Quaterniond rot;
  // Units in mg * ms
  Eigen::Vector3d vel;
  // Units in mg * ms * ms
  Eigen::Vector3d pos;

  FlightState() {}

  void push_baro(double pressure, double temperature, double sample_rate);
  void push_acc(ISM6HG256X_Axes_t &acc, double sample_rate);
  void push_gyro(ISM6HG256X_Axes_t &gyro, double sample_rate);

  double get_servo();

  bool done();
};

struct RestState {
  // There is a degree of freedom (roll I believe) since this is based
  // On the accelerometer originally so y is perpendicular to the ground
  // After applying this rotation to a sampled accelerometer reading
  Eigen::Quaterniond rot = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
  double acceleration;

  bool inited = false;

  RestState() {}

  void push_baro(double pressure, double temperature, double sample_rate);
  void push_acc(ISM6HG256X_Axes_t &acc, double sample_rate);
  void push_gyro(ISM6HG256X_Axes_t &gyro, double sample_rate);

  // Returns true if the rocket is flying and inits the flight state to that
  bool try_init_flying(FlightState &state);

  // Returns true if the rocket is flying and inits the flight state to that (called for some time when booting)
  // Is Currently the same as try_init_flying
  bool try_init_flying_boot(FlightState &state);
};

#endif

