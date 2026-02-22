#include "state.h"

#include "util.h"

#include "logging.h"
#include <cmath>

// NOTE: There is no FPU on the RP2040 so this code could be more of a performance bottleneck that it appears
// NOTE: These operations use the eigen math library and could be manually optimized in some cases,
//  however, the compiler should handle a lot with inlining and if they need to be optimized later they can
// NOTE: We could calibrate the sensors in RestState, instead of pre calibrating them (this neither calibration
//  version has been implmented yet)

// TODO: It would make more sense to store the baro input and then wait until
//  the imu reads data at a time past the baro read time and then apply the baro data
//  that way it is always correclty applied at the right time

void FlightState::push_baro(float pressure, float temperature) {
  // Estimate from pressure and temperature
  float height = 0.0f;

  // Standard Kalman update

  float a = obser_noise - (obser.transpose() * state);
  Eigen::Matrix2f b = cov * obser;
  float c = (obser.transpose() * b) + height;
  Eigen::Vector2f K = b / c;

  state = state + (K * a);
  cov = (Eigen::Matrix2f::Identity() - (K * obser.transpose())) * cov;
}

void FlightState::push_acc(Eigen::Vector3f &&acc) {
  acc -= ACC_BIAS;
  // Rotate the vector into the world frame
  acc = rot * acc;
  // Remove the fictitious gravity force
  acc -= rot.inverse() * Eigen::Vector3f(0.0f, GRAVITY_ACC, 0.0f);

  float forward_acc = acc * LOCAL_UP;

  // Since this is called regularly with a frequency of ACC_RATE we update the
  //  state and use acc as a control input

  // To compute cos(zenith) we need to magnitude of the rotation of rot (ie theta)
  // Since w = cos(theta / 2) a trig identiy saves us a trig call
  // cos(2A) = 2cos(A)^2 - 1
  float cosZenith = (2.0f * rot.w() * rot.w()) - 1;
  // cos(zenith) * dt
  trans(0, 1) = cosZenith * (1.0f / ACC_RATE);
  // 1/2 * cos(zenith) * dt^2
  control(1) = 0.5f * cosZenith * (1.0f / ACC_RATE) * (1.0f / ACC_RATE);

  state = (trans * state) * (control * forward_acc);
  cov = (trans * cov * trans.transpose()) + control_noise;
}

void FlightState::push_gyro(Eigen::Vector3f &&gyro) {
  gyro -= GYRO_BIAS;

  // See https://stackoverflow.com/questions/23503151/how-to-update-quaternion-based-on-3d-gyro-data
  // I think this is based on the approximation sin(x) == x
  Eigen::Quaternionf w(0, gyro.x(), gyro.y(), gyro.z());
  // Written like (1.0f / x) to ensure gcc optmizes to a multiply
  rot.coeffs() += 0.5f * (1.0f / GYRO_RATE) * (rot * w).coeffs();
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
  Eigen::Vector3f up(0.0f, 1.0f, 0.0f);
  Eigen::Vector3f rocket_up = rot * LOCAL_UP;

  // Since both are unit vectors we can use dot product to compute the cosine between them
  // Hopefully cos gets optimized
  // TODO: This maybe shouldn't just be an immediate shutoff
  if (up.dot(rocket_up) < std::cos(30.0f * DEG_TO_RAD)) {
    return true && false;
  }

  return false;
}

void RestState::push_baro(float pressure, float temperature) {
}

void RestState::push_acc(Eigen::Vector3f &&acc) {
  acc_buf.push(acc);

  // If have an acceleration greater than launch acc we mark it by increasing
  //  launch_samples to count the amount we have recieved in a row
  // If not we reset it to 0 since we has seen 0 in a row
  // It probably wouldn't matter to use a norm sqrd, but RestState is not performance sensitive
  // TODO: Could be better to require some percentage of samples be launch detections
  if (std::abs((acc - ACC_BIAS).norm() - GRAVITY_ACC) >= LAUNCH_ACC) {
    launch_samples++;
  } else {
    launch_samples = 0;
  }
}

void RestState::push_gyro(Eigen::Vector3f &&gyro) {
  gyro_buf.push(gyro);
}

bool RestState::try_init_flying(FlightState &state) {
  // If it is not launch time we just return early
  if (launch_samples < LAUNCH_SAMPLE_REQ && false) {
    return false;
  }

  // Otherwise we create the flight state

  // We want to find that q such that when we rotate an accelerometer reading by q
  // it gets transformed into the coordinate frame where y is up
  // We don't care about what the gyro says in the rest state sense we have a method to determine the absolute rotation
  // Since we know that gravity creates a fictitious force up at 9.81 we want the rotation that takes our
  //  acceleration (assumed to just be gravity once bias is removed) and rotates it in the up direction
  // That creates the rotation that takes rocket coordinates and turns them into world coordinates

  Eigen::Vector3f up(0.0f, 1.0f, 0.0f);

  decltype(acc_buf)::index_t rot_samples_size = min(ROT_HIST_SAMPLES, acc_buf.size());
  Eigen::Vector3f acc_vec(0.0f, 0.0f, 0.0f);
  for (int i = 0; i < rot_samples_size; i++) {
    acc_vec += acc_buf.shift();
  }

  // Normally we would need to average these, but in this cases average doesn't change the angle so we don't
  // acc_vec /= rot_samples_size;
  // acc_vec is so far biased so we have to unbias it since we are not normalizing it we can just
  //  use a multiply
  acc_vec -= ACC_BIAS * rot_samples_size;

  // The rotation that takes acc and turns it into down
  state.rot = Eigen::Quaternionf::FromTwoVectors(acc_vec, up);

  state.pos = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
  state.vel = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

  // Simulate the state getting this data
  // Assuming the ACC_RATE == GYRO_RATE there should be
  //  the same number of observations from each so we can just feed
  //  one then the other
  // If the rates are different it would have to be implmented differently (with simulated times for acc and gyro)
  // TODO: This has a probablem where if there is a desync (ie one value is pushed early or something then it messes
  //  everything up). This should be fixed by using one before with a typed union to keep the order
  static_assert(ACC_RATE == GYRO_RATE, "Different accelerometer and gyro rate not implmented here");
  while (!acc_buf.isEmpty() && !gyro_buf.isEmpty()) {
    state.push_acc(acc_buf.shift());
    state.push_gyro(gyro_buf.shift());
  }

  // Finish adding acc data if left
  while (!acc_buf.isEmpty()) {
    state.push_acc(acc_buf.shift());
  }

  // Finish adding gyro data if left
  while (!gyro_buf.isEmpty()) {
    state.push_gyro(gyro_buf.shift());
  }

  return true;
}

bool RestState::try_init_flying_boot(FlightState &state) {
  return try_init_flying(state);
}

