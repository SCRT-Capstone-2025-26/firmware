#ifndef STATE_H
#define STATE_H

#include "util.h"
#include "flash.h"

#include <ISM6HG256XSensor.h>
#include <ArduinoEigen.h>
#include <CircularBuffer.hpp>
#include <cstdint>
#include <tuple>

// These classes define code that handles the rocket's physical state when in rest and in flight
// The rest state also detects the transisition to flying

// The flight state uses a Kalman filter (I think technically EKF) to determine the rockets position
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

// I don't know why these aren't provided as constants from the library
// It basically uses a if statement chain on a bunch of floats to figure out
//  the setting which is strange https://github.com/stm32duino/ISM6HG256X/blob/main/src/ISM6HG256XSensor.cpp#L2705
#define GYRO_RATE 960.0f
#define ACC_RATE  960.0f

// 9.80665f is the ISO value of little g
// TODO: Update these
#define GRAVITY_ACC       9.80665f
// The amount of acc from normal gravity required to consider
//  it a launch
#define LAUNCH_ACC        2.0f
// A big history can easily take up a lot of the kinda limited ram
// The seconds of imu data to have in a rolling buffer so that after
//  launch is detected the first few moments of launch
// If this is big enough that it takes a while to compute
//  the code will have to change
// This must capture the whole launch so data used in the ROT_HIST_SAMPLES
//  is not launch data
// This can't be too big or the playback will be too long and we will miss
//  samples
#define LAUNCH_HIST_S     0.75f
// We use an exponential filter on the boolean value of whether
//  the rocket has launch with the decay. When the value passes
//  req a launch is detected
// As ACC_RATE goes to inf if the decay is (1 / ACC_RATE) the
//  curve looks more and more like e^-x (at 960 is is very like e^-x)
//  so we can pick a constant that will give the same values
//  as long as ACC_RATE remains high 1.5 seems good (it will detect flight
//  after about 0.46s)
#define LAUNCH_SAMPLE_DECAY (1.5f / ACC_RATE)
#define LAUNCH_SAMPLE_REQ   0.5f
// We need to determine the rotation before launch from
//  some accelerometer data so we put that in the circular buffer as well
#define ROT_HIST_SAMPLES  ((uint32_t)(0.5f * ACC_RATE))

// This depends on how the lookup table is made
// TODO: Determine value
#define START_HEIGHT 0.0f

// The errors the rocket starts with
// The cross error is 0
// TODO: Determine values
#define START_H_ERROR 1.0f
#define START_V_ERROR 1.0f

// This is the values that the rocket sets the estimated values to if it is booted during flight and there is no flash history
// TODO: Determine values
#define UNK_START_HEIGHT 1000.0f
#define UNK_START_VEL    100.0f

// The errors the rocket starts with if it is booted during flight and there is no flash history
// TODO: Determine values
#define UNK_START_H_ERROR 100.0f
#define UNK_START_V_ERROR 100.0f
#define UNK_START_VH_CORR 100.0f

// This is the values that the rocket sets the estimated values to if it is booted during flight
// TODO: Determine values
#define BOOT_INC_HEIGHT 100.0f
#define BOOT_INC_VEL    -10.0f

// The errors the rocket starts with if it is booted during flight
// TODO: Determine values
#define BOOT_INC_H_ERROR 10.0f
#define BOOT_INC_V_ERROR 10.0f
#define BOOT_INC_VH_CORR 10.0f

// The upward acceleration below which beavs can extend
// TODO: Determine value
#define BEAVS_EXT_ACC (GRAVITY_ACC * 0.5f)

// Launch rail angle (4 degrees off straight up)
// TODO: Check
#define RAIL_ANGLE (4.0f * DEG_TO_RAD)

// This is roughly accurate
#define MACH_TO_MPS 343.0f
// The speed range where the barometer is ignored due to areodynamic effects
// TODO: Determine these
#define MIN_BARO_CUTOFF (0.8f * MACH_TO_MPS)
#define MAX_BARO_CUTOFF (1.2f * MACH_TO_MPS)

// 0 percent is 0mm
#define SERVO_MM_TO_PERCENT (1.0f / 25.0f)

// https://en.wikipedia.org/wiki/Brothers,_Oregon
#ifdef TEST
#define HEIGHT_ABOVE_SEA_LEVEL 0.0f
#else
#define HEIGHT_ABOVE_SEA_LEVEL 1414.0f
#endif

// NOTE: Changing this affects the following line and load_flash() in state.cpp
const Eigen::Vector3f LOCAL_UP(0.0f, 0.0f, 1.0f);
// This is based on LOCAL_UP (this init should be changed to be dependent on LOCAL_UP)
// TODO: Check this math see load_flash
const Eigen::Vector3f RAIL_VEC(0.0f, std::sin(RAIL_ANGLE), std::cos(RAIL_ANGLE));

struct FlightState {
  Eigen::Quaternionf rot;
  float cosZenith;
  // The square magnititude of the accelerometer with the gravity acceleration included
  // This shuts off the BEAVS extension if it is to high to be safe
  // NOTE: Safety critical
  float forward_acc = BEAVS_EXT_ACC;

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
  void push_acc(Eigen::Vector3f acc, bool high_g);
  void push_gyro(Eigen::Vector3f gyro);

  // Converts the current state into a flash saveable struct
  // Or converts from
  // When it converts from it adds noise and time associated with a reboot
  void load_flash(FlashState &&flash_state);
  FlashState get_flash();

  void set_rot(Eigen::Quaternionf newRot);

  float get_servo();

  bool done();
};

struct Measurement {
  Eigen::Vector3f data;
  bool is_high_g;
  bool is_acc;
};

struct RestState {
  const static uint16_t BUF_SIZE = (uint16_t)(ACC_RATE * LAUNCH_HIST_S) + (uint16_t)(GYRO_RATE * LAUNCH_HIST_S);
  // These hold the data collected by the imu because when the launch
  //  happens it will be detected a bit late due to filtering out false
  //  positives
  // This is a huge ram sink, but should be fine as we have enough and don't use dynamic allocation (except a few strings)
  CircularBuffer<Measurement, BUF_SIZE> buf;
  // This is pushed after a acc measurement is ejected from the buf so it contains data
  // older than the buf
  CircularBuffer<Eigen::Vector3f, ROT_HIST_SAMPLES> rot_calib_buf;

  // A exponential weigth average of the boolean value
  // of whether a launch acceleration was detected for that
  // given timestep
  float launchiness = 0;

  RestState() {}

  void push_buf(Measurement &&meas);

  void push_acc(Eigen::Vector3f acc, bool high_g);
  void push_gyro(Eigen::Vector3f gyro);

  // Returns true if the rocket is flying and inits the flight state to that
  bool try_init_flying(FlightState &state);

  // Returns true if the rocket is flying and inits the flight state to that (called for some time when booting)
  // Is Currently the same as try_init_flying
  bool try_init_flying_boot(FlightState &state);
};

#endif

