#include "state.h"

#include <cmath>

#include "flash.h"
#include "table.h"

// NOTE: There is no FPU on the RP2040 so this code could be more of a performance bottleneck that it appears
// NOTE: These operations use the eigen math library and could be manually optimized in some cases,
//  however, the compiler should handle a lot with inlining and if they need to be optimized later they can.
//  The assembly doesn't look bery optmized (there are a lot of 0s in the matrix that can be propogated) unless
//  called with O3. Of course performance tests are better than looking at the asm.
// NOTE: We could calibrate the sensors in RestState, instead of pre calibrating them (this neither calibration
//  version has been implmented yet)
// NOTE: It is worth considering what parts of this could be fixed point

// NOTE: Pressure is in milibars we plan on converting it
void FlightState::push_baro(float pressure, float temperature) {
  // We ignore the barometer at speeds where the pressure is know to be unreliable
  //  due to areodynamic stuff around 1 mach
  if (MIN_BARO_CUTOFF <= state(1) && MAX_BARO_CUTOFF >= state(1)) {
    return;
  }

  // Estimate from pressure and temperature
  // This is the formula used by https://github.com/RobTillaart/MS5611
  // TODO: Update this math
  float height = 44307.694 * (1 - pow(pressure / 1013.25, 0.190284));
  height -= HEIGHT_ABOVE_SEA_LEVEL;
  // Using the state like this is kinda not allowed in a true Kalman filter
  float noise = 50.0f;

  // Standard Kalman update

  float inno = height - (obser * state);
  float inno_noise = (obser * cov * obser.transpose()) + noise;

  // FYI lol https://math.stackexchange.com/questions/2336473/what-is-the-inverse-of-a-1-times-1-matrix
  Eigen::Vector2f gain = cov * obser.transpose() * (1.0f / inno_noise);

  state += gain * inno;
  cov = (Eigen::Matrix2f::Identity() - (gain * obser)) * cov;
}

void FlightState::push_acc(Eigen::Vector3f acc, bool is_high_g) {
  // Remove the fictitious gravity force
  acc -= rot.inverse() * Eigen::Vector3f(0.0f, GRAVITY_ACC, 0.0f);

  // Forward acc determines if beavs can extend
  forward_acc = acc.dot(LOCAL_UP);
  // TODO: Determine
  float noise = 20.0f;

  // Since this is called regularly with a frequency of ACC_RATE we update the
  //  state and use acc as a control input

  // Standard Kalman predict
  // See https://stats.stackexchange.com/questions/134920/kalman-filter-with-input-control-noise for the control noise

  // cos(zenith) * dt
  trans(0, 1) = cos_zenith * (1.0f / ACC_RATE);
  // 1/2 * cos(zenith) * dt^2
  control(0) = 0.5f * cos_zenith * (1.0f / ACC_RATE) * (1.0f / ACC_RATE);

  state = (trans * state) + (control * forward_acc);
  // See https://stats.stackexchange.com/questions/134920/kalman-filter-with-input-control-noise for the control noise
  cov = (trans * cov * trans.transpose()) + (control * noise * control.transpose()) + trans_noise;
}

void FlightState::push_gyro(Eigen::Vector3f gyro) {
  // See https://stackoverflow.com/questions/23503151/how-to-update-quaternion-based-on-3d-gyro-data
  // I think this is based on the approximation sin(x) == x
  Eigen::Quaternionf w(0, gyro.x(), gyro.y(), gyro.z());
  // Written like (1.0f / x) to ensure gcc optmizes to a multiply
  rot.coeffs() += 0.5f * (1.0f / GYRO_RATE) * (rot * w).coeffs();
  rot.normalize();

  // This updates cos_zenith
  set_rot(rot);
}

void FlightState::load_flash(FlashState &&flash_state) {
  state(0) = flash_state.h;
  state(1) = flash_state.v;

  cov(0, 0) = flash_state.h_cov;
  // The matrix is symmetric
  cov(1, 0) = flash_state.hv_cov;
  cov(0, 1) = flash_state.hv_cov;
  cov(1, 1) = flash_state.v_cov;

  // Since one axis is arbitrary we can just pick a vector such that its cos(angle) from vertical is cos_zenith
  // This is based on LOCAL_UP (this init should be changed to be dependent on LOCAL_UP)
  // NOTE: This depends on LOCAL_UP in state.h
  // TODO: Check this math see LAUNCH_VEC
  Eigen::Vector3f up(0.0f, 1.0f, 0.0f);
  Eigen::Vector3f flight_vec(0.0f, 1.0f - flash_state.cos_zenith, flash_state.cos_zenith);
  set_rot(Eigen::Quaternionf::FromTwoVectors(flight_vec, up));
}

// NOTE: The code assumes that this function doesn't read rot
void FlightState::set_rot(Eigen::Quaternionf new_rot) {
  rot = new_rot;

  // We know that (0.0f, 0.0f -1.0f is up from the local frame
  // So transforming our local up to the global frame
  // So we see if the angle between true up and our local up is more than 30 degrees
  Eigen::Vector3f up(0.0f, 1.0f, 0.0f);
  Eigen::Vector3f rocket_up = rot * LOCAL_UP;
  cos_zenith = up.dot(rocket_up);
}

FlashState FlightState::get_flash() {
  return FlashState(
    state(0),
    state(1),
    cov(0, 0),
    cov(1, 0), // Since the covariance is symmetric we only need one
    cov(1, 1),
    cos_zenith
  );
}

// 0 percent servo is flush with the hull
float FlightState::get_servo() {
  // We can't extend beavs while until we are not accelerating aka the raw (gravity included) accelerometer reading is small
  if (forward_acc >= BEAVS_EXT_ACC) {
    return 0.0f;
  }

  return index_table(state(0), 1.0, state(1)) * SERVO_MM_TO_PERCENT;
}

bool FlightState::done() {
  // I believe IREC requires no flight controls at 30 degrees
  // Hopefully cos gets optimized
  // NOTE: This maybe shouldn't just be an immediate shutoff (although if we calculate 30 deg may be cooked anyway)
  if (cos_zenith < std::cos(30.0f * DEG_TO_RAD)) {
    return true;
  }

  return false;
}

void RestState::push_buf(Measurement &&meas) {
  // If buf is full then the acceleration data being read is moved to the calibration buffer
  if (buf.isFull()) {
    Measurement old_meas = buf.shift();
    if (old_meas.is_acc) {
      rot_calib_buf.push(old_meas.data);
    }
  }

  buf.push(meas);
}

void RestState::push_acc(Eigen::Vector3f acc, bool high_g) {
  push_buf(Measurement{acc, high_g, true});

  // If have an acceleration greater than launch acc we mark it by increasing
  //  launch_samples to count the amount we have recieved in a row
  // It probably wouldn't matter to use a norm sqrd, but RestState is not performance sensitive
  // Exponential moving average of the boolean values
  launchiness *= (1.0f - LAUNCH_SAMPLE_DECAY);
  launchiness_boot *= (1.0f - LAUNCH_SAMPLE_DECAY);

  if (std::abs(acc.norm() - GRAVITY_ACC) >= LAUNCH_ACC) {
    launchiness += LAUNCH_SAMPLE_DECAY;
  }

  if (std::abs(acc.norm() - GRAVITY_ACC) >= LAUNCH_ACC_BOOT) {
    launchiness_boot += LAUNCH_SAMPLE_DECAY;
  }
}

void RestState::push_gyro(Eigen::Vector3f gyro) {
  push_buf(Measurement{gyro, false, false});
}

bool RestState::try_init_flying(FlightState &state) {
  // If it is not launch time we just return early
  if (launchiness <= LAUNCH_SAMPLE_REQ) {
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

  // If there is no calibration readings (which shouldn't happen then we default to the launch rail angle)
  Eigen::Vector3f acc_vec(0.0f, 0.0f, 0.0f);
  decltype(rot_calib_buf)::index_t rot_samples_size = rot_calib_buf.size();
  if (rot_samples_size > 0) {
    while (!rot_calib_buf.isEmpty()) {
      acc_vec += rot_calib_buf.shift();
    }
  } else {
    acc_vec = RAIL_VEC;
  }

  // The rotation that takes acc and turns it into down
  state.set_rot(Eigen::Quaternionf::FromTwoVectors(acc_vec, up));

  state.state = Eigen::Vector2f(START_HEIGHT, 0.0f);
  
  state.cov(0, 0) = START_H_ERROR;
  state.cov(0, 1) = 0.0f;
  state.cov(1, 0) = 0.0f;
  state.cov(1, 1) = START_V_ERROR;

  // Before launch we are experiencing the fictitious gravity acceleration
  state.forward_acc = GRAVITY_ACC;

  // Simulate the state getting this data
  // This is quite expensive
  while (!buf.isEmpty()) {
    Measurement meas = buf.shift();
    if (meas.is_acc) {
      state.push_acc(meas.data, meas.is_high_g);
    } else {
      state.push_gyro(meas.data);
    }

    // This kinda violates the design principles
    watchdog_update();
  }

  return true;
}

// This runs in UNKOWN mode and if a flight is detected it means we have just booted
bool RestState::try_init_flying_boot(FlightState &state) {
  // If it is not launch time we just return early
  if (launchiness <= LAUNCH_SAMPLE_REQ_BOOT) {
    return false;
  }

  Eigen::Vector3f up(0.0f, 1.0f, 0.0f);

  state.set_rot(Eigen::Quaternionf::FromTwoVectors(RAIL_VEC, up));
  state.state = Eigen::Vector2f(UNK_START_HEIGHT, UNK_START_VEL);

  state.cov(0, 0) = UNK_START_H_ERROR;
  state.cov(0, 1) = UNK_START_VH_CORR;
  state.cov(1, 0) = UNK_START_VH_CORR;
  state.cov(1, 1) = UNK_START_V_ERROR;

  // We just set this to a value that will not allow beavs to extend immediately
  state.forward_acc = BEAVS_EXT_ACC;

  return true;
}

