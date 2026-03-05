#include <ISM6HG256XSensor.h>
#include <SoftwareSPI.h>
#include <MS5611_SPI.h>
#include <SPI.h>
#include <RP2040_PWM.h>
#include <cmath>
#include <hardware/watchdog.h>
#include <cstdint>
#include <cmath>

#include "flash.h"
#include "pins.h"
#include "state.h"
#include "logging.h"
#include "led.h"
#include "util.h"
#include "ina745.h"

// NOTE: This code uses millis() extensively and assumes it will not overflow (it will overflow in >40 days and that is not intended usage)
// TODO: Look into pressure drop when hitting around mach numbers

#define SERVO_CHARGE_MILLIS 2000
#define UNKNOWN_WAIT        2000

#define SERVO_FREQ  300.0f
#define SERVO_MIN   0.0f
#define SERVO_MAX   1.0f
// TODO: Check this
#define SERVO_DUTY_MIN 0.75f
#define SERVO_DUTY_MAX 0.15f
#define SERVO_FLUSH    0.0f

// The exponential decay for the servo during flight
//  p = (SERVO_SMOOTH * p) + ((SERVO_SMOOTH - 1) * new_p)
//  This formula assumes a sample every second the real formula does not
// To prevent jittery servo
// TODO: Determine
#define SERVO_SMOOTH 0.3
// Get servo smooth into more favourable units
// This should be compile time const
#define SERVO_SMOOTH_LN_MS std::log(SERVO_SMOOTH * 0.001f)

// TODO: Determine these
// NOTE: Changing these requires recalibration
#define GYRO_FS 4000
#define ACC_FS 4
#define ACC_HIGH_G_FS 64

// We treat very large or small values as errors to avoid hitting an extreme tail in the kalman filter
// TODO: Detrmine these
#define MAX_PRES 0
#define MIN_PRES 0

#define MAX_TEMP 0
#define MIN_TEMP 0

#define MIN_ACC_SQR_MAG 0
#define MIN_ACC_SQR_MAG 0

// The value where the acc switch froms low g to high g
// Currently ACC_FS * GRAVITY_ACC is roughly the max acc reading
//  of the low so when 80% of that is reached it switches
// TODO: Determine value
#define ACC_HIGH_G_SWITCH (ACC_FS * GRAVITY_ACC * 0.8f)

// TODO: Tune the senstivities based on calibration

// I don't know why the constants don't work
// This is just guestimated
#define GYRO_SENS (ISM6HG256X_GYRO_SENSITIVITY_FS_4000DPS * 0.5f)
// I don't know why the constants don't work
// This is just guestimated
#define ACC_SENS (ISM6HG256X_ACC_SENSITIVITY_FS_4G * 3.95)
// This is just guestimated
#define ACC_HIGH_G_SENS (ISM6HG256X_ACC_SENSITIVITY_FS_64G * 0.55f)

enum BaroState {
  IDLE,
  READING_TEMP,
  READING_PRES
};

const Eigen::Vector3f ACC_BIAS(0.008095040980820646f, -0.07066856444586497f, -0.06873988143672187f);
const Eigen::Vector3f ACC_HIGH_G_BIAS(0.0f, 0.0f, 0.0f);
const Eigen::Vector3f GYRO_BIAS(0.0020154851083784846f, 0.0032312920667005307f, -0.002640418776621421f);

FlightState flight_state = FlightState();
RestState rest_state = RestState();

bool servo_powered = false;
float flight_servo_pecent;
Millis flight_servo_last_ms;

Millis baro_read_time;
BaroState baro_state = IDLE;

INA745 current_sensor = INA745(CURRENT_1_ID, &Wire);
bool current_sens_failed = false;

// The mode of the accelerometer
// We stay in high g mode and only switch to low g after launch
bool acc_high_g = true;
// Whether we have observed the readings from the new mode in the fifo
//  once they appear we discard readings from the old mode since we would double
//  count otherwise (there is still a one measurement double possible depending on the
//  the order written, but there is only so much we can do)
bool acc_fifo_switched = true;

Millis next_flash_write;

// The pins aren't correctly assigned for hardware SPI on the board
// I assume it is a mistake (?) so we have to use bit banging
SoftwareSPI softSPI(SPI_SCK, SPI_MISO, SPI_MOSI);

MS5611_SPI baro(BAROMETER_CS, &softSPI);
ISM6HG256XSensor imu(&softSPI, IMU_CS);

// The servo has an operating frequency of 50-300Hz
RP2040_PWM servo(SERVO_1, (float)SERVO_FREQ, 0.0f);

void init_pins() {
  // Disable servo power on startup due to inrush
  // Removing this could damage the board
  pinMode(SERVO_POWER_ENABLE, OUTPUT);
  digitalWrite(SERVO_POWER_ENABLE, LOW);

  // Put the level shifter into the regular mode
  pinMode(LEVELSHIFT_DIR, OUTPUT);
  digitalWrite(LEVELSHIFT_DIR, LOW);

  // We only use servo 1 so that is the only one inited to be a pwm pin
  // The others can just be 0
  servo.setPWM();
  // No floating pins for levelshifter
  pinMode(SERVO_2, OUTPUT);  digitalWrite(SERVO_2, LOW);
  pinMode(SERVO_3, OUTPUT);  digitalWrite(SERVO_3, LOW);
  pinMode(SERVO_4, OUTPUT);  digitalWrite(SERVO_4, LOW);
  pinMode(SERVO_5, OUTPUT);  digitalWrite(SERVO_5, LOW);
  pinMode(SERVO_6, OUTPUT);  digitalWrite(SERVO_6, LOW);
  pinMode(LED_DATA, OUTPUT); digitalWrite(LED_DATA, LOW);

  pinMode(MOSI, OUTPUT);
  pinMode(MISO, INPUT);
  pinMode(SCK, OUTPUT);

  pinMode(ARM_SWITCH, INPUT);
  pinMode(BATTERY_SENSE, INPUT);

  pinMode(BAROMETER_CS, OUTPUT);
  pinMode(IMU_CS, OUTPUT);
  pinMode(RADIO_CS, OUTPUT);

  digitalWrite(BAROMETER_CS, HIGH);
  digitalWrite(IMU_CS, HIGH);
  digitalWrite(RADIO_CS, HIGH);
}

// The servo cannot be enabled before the capacitors charge
bool try_power_servo() {
  if (millis() < SERVO_CHARGE_MILLIS) {
    return false;
  }

  digitalWrite(SERVO_POWER_ENABLE, HIGH);
  servo_powered = true;
  return true;
}

// NOTE: Init values are temporary and will be determined by data later
void setup() {
#ifdef DEBUG
  // Allow some time for the serial monitor to connect
  sleep(DEBUG_BOOT_DELAY);
#endif

  // The 1 means it plays nice with the debugger
  watchdog_enable(WATCHDOG_MS, 1);

  // Initialize the pins
  // This initializes the servo power pins which if improperly initialized can cause
  //  problems with the capacitors when charging
  init_pins();
  log_message("Pins inited");

  // This inits the LEDs it sets them all to powered off
  led_init();
  log_message("LEDs inited");

  // Push mode uses the leds
  push_mode(BOOTING);

  // The radio is not currently used (or installed) so we just set the led to mark that (neutral is blue which is visible)
  leds[LED_RADIO] = LED_NEUTRAL;
  // Whether or not the watchdog has been triggered
  leds[LED_WATCHDOG] = watchdog_caused_reboot() ? LED_NEGATIVE : LED_POSITIVE;
  led_show();

  // Initialize the LED the rp2040 has two SPIs and we init the first one to be able to communicate to the sensors
  softSPI.begin();
  log_message("SPI inited");

  // Initialize the barometer we has an MS5607, but the interface should be the same as the MS5611 which
  // is the library we are using
  // NOTE: This takes 3 ms to read the PROM which could be hardcoded if we want to save boot time
  bool baro_init = baro.begin();
  // Since we are using softwareSPI we have to set the rate in this library
  // This is what the brinup code sets it to so ...
  baro.setSPIspeed(10000000);
  // This sampling will change in the future
  baro.setOversampling(OSR_ULTRA_HIGH);

  // The barometer is not sampling right now
  baro_state = IDLE;

  if (baro_init) { log_message("Barometer inited"); }
  leds[LED_BARO] = baro_init ? LED_POSITIVE : LED_NEGATIVE;
  led_show();

  // The imu returns custom status values instead of booleans
  // So the returns are checked (as shown in their example code you can also or all these values and then check ISM6HG256_OK at the end)
  bool imu_init = imu.begin() == ISM6HG256X_OK;

  // Set the ouput rate
  imu_init &= imu.Set_G_OutputDataRate(GYRO_RATE) == ISM6HG256X_OK;
  imu_init &= imu.Set_X_OutputDataRate(ACC_RATE) == ISM6HG256X_OK;
  imu_init &= imu.Set_HG_X_OutputDataRate(ACC_RATE) == ISM6HG256X_OK;

  // Set the sensor scales
  imu_init &= imu.Set_X_FullScale(GYRO_FS) == ISM6HG256X_OK;
  imu_init &= imu.Set_X_FullScale(ACC_FS) == ISM6HG256X_OK;
  imu_init &= imu.Set_X_FullScale(ACC_HIGH_G_FS) == ISM6HG256X_OK;

  // Set the rate at which data is stored in the fifo (I believe)
  imu_init &= imu.FIFO_G_Set_BDR(GYRO_RATE) == ISM6HG256X_OK;
  imu_init &= imu.FIFO_X_Set_BDR(ACC_RATE) == ISM6HG256X_OK;
  // Allow high readings in the FIFO
  imu_init &= imu.FIFO_Set_HG(true) == ISM6HG256X_OK;

  // Set Set FIFO watermark level
  imu_init &= imu.FIFO_Set_Watermark_Level(199) == ISM6HG256X_OK;
  // Set FIFO stop on watermark level
  imu_init &= imu.FIFO_Set_Stop_On_Fth(1) == ISM6HG256X_OK;

  // Enable the fifo in continious mode (ie it overwrites old samples)
  imu_init &= imu.FIFO_Set_Mode(ISM6HG256X_STREAM_MODE) == ISM6HG256X_OK;

  // Enable all the sensors on the imu. It has two accelerometers one for low g and one for high g.
  // Both can be active at once
  imu_init &= imu.Enable_G() == ISM6HG256X_OK;
  imu_init &= imu.Enable_HG_X() == ISM6HG256X_OK;

  // Mark that we are in high_g mode
  acc_high_g = true;
  acc_fifo_switched = true;

  // TODO: Set up the accelerometer mode currently ISM6HG256X_ACC_HIGH_ACCURACY_ODR_MODE
  //  just immediatly causes the init to do nothing and return error

  if (imu_init) { log_message("IMU inited"); }
  leds[LED_IMU] = imu_init ? LED_POSITIVE : LED_NEGATIVE;
  led_show();

  // We have no led and if it fails no big deal since it just logs
  if (current_sensor.begin() == INA_SUCCESS) {
    log_message("Current sensor inited");
    current_sens_failed = true;
  }

  if (baro_init && imu_init) {
    // The board is now ready
    // If the arm switch is active we could be booting in flight
    //  and so should be in UNKNOWN until we know otherwise we can
    //  just be in unarmed
    if (digitalRead(ARM_SWITCH) == ARM_ON) {
      push_mode(UNKNOWN);
    } else {
      push_mode(UNARMED);
    }
  } else {
    // The board has failed to init
    note_error("Init failed", FAIL_NOW_ERR);
  }

  watchdog_update();
}

// This is called when the board confirms that it has booted an is on the ground waiting to launch
// It could take some time to run (because it waits for the log core), but it shouldn't because the
//  log core should boot fast
void ground_boot() {
  log_message("Clearing flash");

  // We mess with the watchdog here since this could take a long time and we are on the ground and safe
  watchdog_enable(WATCHDOG_MS_CLEAR_FLASH, 1);
  clear_flash_buf();
  watchdog_enable(WATCHDOG_MS, 1);

  log_message("Waiting on log core");
  // Wait for the other core to finish booting
  // This returns when the other core has booted with whether it has created log files
  bool sd_failure = wait_log_boot();
  // If there is an SD failure mark that (it is not critical though).
  // Since the function returned the other core has booted and we can continue
  log_message("Log core booted");
  if (!sd_failure) { log_message("SD inited"); }
  leds[LED_SD] = sd_failure ? LED_NEGATIVE : LED_POSITIVE;
  led_show();
}

// This is called every loop iteration and is responsible for managing the state transitions
// Returns true if the mode changed
void update_mode() {
  Mode old_mode = board_mode;

  switch (board_mode) {
    case UNKNOWN:
      // If booted during flight we should know our before 
      if (rest_state.try_init_flying_boot(flight_state)) {
        FlashState state;
        if (flash_reinit(&state)) {
          flight_state.load_flash(std::move(state));
        }

        push_mode(FLYING);
        pushed_flying = true;
      } else if (millis_in_mode() >= UNKNOWN_WAIT) {
        ground_boot();
        push_mode(UNARMED);
      }

      break;

    case UNARMED:
      if (digitalRead(ARM_SWITCH) == ARM_ON) {
        push_mode(ARMED);
      }

      break;

    case ARMED:
      // If the rest_state can init the flying state then it means that it has detected high acceleration
      //  and we are in flight
      if (rest_state.try_init_flying(flight_state)) {
        push_mode(FLYING);
        pushed_flying = true;

      }

      if (digitalRead(ARM_SWITCH) == ARM_OFF) {
        push_mode(UNARMED);
      }

      break;

    case FLYING:
      // If the flight_state decides that we are done (practically our angle is too high or we have timed out)
      if (flight_state.done()) {
        push_mode(DONE);
      }

      break;

    case DONE:
      break;

    default:
      note_error("Invalid mode", FAIL_NOW_ERR);

      break;
  }

  if (board_mode != old_mode && board_mode == FLYING) {
    Millis next_flash_write = 0;

    flight_servo_percent = 0.0f;
    flight_servo_last_ms = 0;
  }

  watchdog_update();
}

void update_servo() {
  if (!servo_powered) {
    if (try_power_servo()) {
      log_message("Servo powered");
    } else {
      return;
    }
  }

  // We should just let the servo be since we don't know if we are flying and
  //  should leave it
  if (board_mode == UNKNOWN) {
    return;
  }

  float servo_percent = 0.0f;
  if (board_mode == FLYING) {
    // FlightState shouldn't output beavs more than 0.0f if it is dangerous,
    //  but this provides fallback security in case
    if (acc_high_g) {
      servo_percent = 0.0f;
    } else {
      servo_percent = flight_state.get_servo();
    }

    // This interpolates between the two servo values based on the time
    //  elapsed it is has a pretty heavy duty math, but we can afford it
    Millis time = millis_in_mode();
    Millis dt = time - flight_servo_last_ms;

    float interp = std::exp(SERVO_SMOOTH_LN_MS * dt);
    flight_servo_percent = (flight_servo_pecent * interp) + (servo_percent * (1.0f - interp));
    servo_percent = flight_servo_pecent;
  } else if (board_mode == UNARMED) {
    // Just a generic parabola (maxed with 0) to generate the full range of motion over a few seconds
    // It is 0 at 1500 and 4500 millis and peaks at 1 since it is 0 at 1500 millis that gives
    // the servo 1500 (and for servo to be powered after UNKNOWN) to zero since we don't know its position
    // If it enters this mode before the servo is inited the parabola could be messed up
    // The (1.0f / x) is for optimization
    float time = millis_in_mode() * (1.0f / SECONDS_TO_MILLIS);
    servo_percent = max(-(time - 1.5f) * (time - 4.5f) * (1.0f / 2.25f), 0.0f);
  }

  float duty_percent = (servo_percent * (SERVO_DUTY_MAX - SERVO_DUTY_MIN)) + SERVO_DUTY_MIN;
  // This does only returns false configuration errors so we just fail if this returns false
  if (!servo.setPWM(SERVO_1, SERVO_FREQ, duty_percent * 100.0f)) {
    note_error("PWM config error", FAIL_NOW_ERR);
  }

  write_data(Servo{servo_percent});

  watchdog_update();
}

// TODO: Check self heating mentioned for similar product in MS5xxx library docs
void step_sample_baro() {
  switch (baro_state) {
    case IDLE:
      // We only sample when flying
      if (board_mode == FLYING) {
        // We read the pressure first because we care about its accuracy less
        //  so reading it first creates less of a time delay issue
        // Set the baro_read_time to the sample delay
        if (!baro.startReadRawTemp(&baro_read_time)) {
          note_error("Baro temp failure", BARO_ERR);
          // This is not critical we just reset the read
          baro_state = IDLE;
          break;
        }

        // Then add the current time so it is the future time when the delay is done
        baro_read_time += millis();
        baro_state = READING_TEMP;
      }

      break;

    case READING_TEMP:
      // If we have finished the read we switch to the pressure reading
      if (millis() >= baro_read_time) {
        // Set the baro_read_time to the sample delay
        if (!baro.stepReadRawPres(&baro_read_time)) {
          note_error("Baro pres failure", BARO_ERR);
          // This is not critical we just reset the read
          baro_state = IDLE;
          break;
        }

        // Then add the current time so it is the future time when the delay is done
        baro_read_time += millis();
        baro_state = READING_PRES;
      }
      break;

    case READING_PRES:
      // If we have finished the read we switch either clear the sensor
      //  or send the reading to flight state and restart the read
      if (millis() >= baro_read_time) {
        if (!baro.finishReading()) {
          note_error("Baro finish failure", BARO_ERR);
          // This is not critical we just reset the read
          baro_state = IDLE;
          break;
        }

        if (board_mode == FLYING) {
          float pres = baro.getPressure();
          float temp = baro.getTemperature();

          if (pres < MIN_PRES || pres > MAX_PRES || temp < MIN_TEMP || temp > MAX_TEMP) {
            note_error("Suspicious baro reading", BARO_ERR);
            // This is not critical we just reset the read
            baro_state = IDLE;
            break;
          }

          flight_state.push_baro(baro.getPressure(), baro.getTemperature());
          write_data(Baro{baro.getPressure(), baro.getTemperature()});

          // We now restart the sample (we could use a switch fallthrough here)
          // Set the baro_read_time to the sample delay
          if (!baro.startReadRawTemp(&baro_read_time)) {
            note_error("Baro temp after finish failure", BARO_ERR);
            // This is not critical we just reset the read
            baro_state = IDLE;
            break;
          }

          // Then add the current time so it is the future time when the delay is done
          baro_read_time += millis();

          baro_state = READING_TEMP;
        } else {
          // otherwise we return to idle since we should not be sampling the barometer
          baro_state = IDLE;
        }
      }
      break;
  }

  watchdog_update();
}

bool set_acc_mode(bool new_high_g) {
  if (acc_high_g == new_high_g) {
    return true;
  }

  // If we fail to enable the acc we want then this function fails
  // If we fail to disable the other acc this doesn't fail it just
  //  will fill the fifo with more garbage than needed which is fine
  if (acc_high_g) {
    if (imu.Enable_X() != ISM6HG256X_OK) {
      return false;
    }

    if (imu.Disable_HG_X() != ISM6HG256X_OK) {
      note_error("Failed to disable HG", IMU_ERR);
    }
  } else {
    if (imu.Enable_HG_X() != ISM6HG256X_OK) {
      return false;
    }

    if (imu.Disable_X() != ISM6HG256X_OK) {
      note_error("Failed to disable non-HG", IMU_ERR);
    }
  }

  acc_high_g = new_high_g;
  acc_fifo_switched = false;

  return true;
}

// TODO: Maybe account for the accelerometer effects offset from the gyro
// NOTE: We read raw data because not reading raw data reads the senstivity
//  from the sensor making the FIFO reading twice as slow. We also don't use
//  the standard senstivity instead using calibrated senstivities
void sample_imu() {
  bool acc_axis_read = false;
  bool gyro_axis_read = false;

  int16_t reading_data[3];
  // These are biased the bias is not removed
  //  since we use them for changing from high g to low g
  //  and we don't want to remove the bias from that they are scaled
  //  though
  Eigen::Vector3f acc_axis;
  Eigen::Vector3f gyro_axis;

  float sqr_mag;
  // This is after the high and low pass filter
  // If nothing the high and low pass filter we should probably go to high G mode
  float filtered_sqr_mag = ACC_HIGH_G_SWITCH;

  uint16_t samples;
  if (imu.FIFO_Get_Num_Samples(&samples) != ISM6HG256X_OK) {
    note_error("Sample read failed", IMU_ERR);
    return;
  }

  uint8_t tag;
  for (uint16_t i = 0; i < samples; i++) {
    // We could try recovering the read on errors, but I think it is best to just leave the loop
    if (imu.FIFO_Get_Tag(&tag) != ISM6HG256X_OK) {
      note_error("Tag read failed", IMU_ERR);
      break;
    }
    if (imu.FIFO_Get_Data((uint8_t *)reading_data) != ISM6HG256X_OK) {
      note_error("Data read failed", IMU_ERR);
      break;
    }

    switch (tag) {
      case GYRO_TAG:
        gyro_axis_read = true;

        gyro_axis.x() = reading_data[0] * GYRO_SENS;
        gyro_axis.y() = reading_data[1] * GYRO_SENS;
        gyro_axis.z() = reading_data[2] * GYRO_SENS;
        gyro_axis -= GYRO_BIAS;

        if (board_mode == FLYING) {
          flight_state.push_gyro(gyro_axis);
        }

        break;

      case ACC_TAG:
        acc_axis_read = true;

        // If we are in high_g mode and the acc_fifo has started outputing high_g
        //  we ignore our data
        // If we are not in high_g mode then since we are getting low g data
        //  we should mark that
        if (acc_high_g) {
          if (acc_fifo_switched) {
            break;
          }
        } else {
          acc_fifo_switched = true;
        }

        acc_axis.x() = reading_data[0] * ACC_SENS;
        acc_axis.y() = reading_data[1] * ACC_SENS;
        acc_axis.z() = reading_data[2] * ACC_SENS;
        acc_axis -= ACC_BIAS;

        sqr_mag = acc_axis.dot(acc_axis);
        if (sqr_mag < MIN) {
          note_error("Suspicious imu normal g reading", IMU_ERR);
          // This is not critical we just skip the read
          break;
        } else {
          filtered_sqr_mag = sqr_mag;
        }

        if (board_mode == FLYING) {
          flight_state.push_acc(acc_axis, false);
        } else if (board_mode == UNKNOWN || board_mode == UNARMED || board_mode == ARMED) {
          rest_state.push_acc(acc_axis, false);
        }

        break;

      case ACC_HG_TAG:
        // Getting a high g reading from the fifo is the same as getting an normal accelerometer reading
        //  at least a raw reading
        acc_axis_read = true;

        // See case ACC_TAG
        if (!acc_high_g) {
          if (acc_fifo_switched) {
            break;
          }
        } else {
          acc_fifo_switched = true;
        }

        acc_axis.x() = reading_data[0] * ACC_HIGH_G_SENS;
        acc_axis.y() = reading_data[1] * ACC_HIGH_G_SENS;
        acc_axis.z() = reading_data[2] * ACC_HIGH_G_SENS;
        acc_axis -= ACC_HIGH_G_BIAS;

        sqr_mag = acc_axis.dot(acc_axis);
        if (sqr_mag < MIN) {
          note_error("Suspicious imu hg reading", IMU_ERR);
          // This is not critical we just skip the read
          break;
        } else {
          filtered_sqr_mag = sqr_mag;
        }

        if (board_mode == FLYING) {
          flight_state.push_acc(acc_axis , true);
        } else if (board_mode == UNKNOWN || board_mode == UNARMED || board_mode == ARMED) {
          rest_state.push_acc(acc_axis, true);
        }

        break;

      default:
        note_error("Unkown tag read", IMU_ERR);
        break;
    }

    // We only use low-g mode once in flight because when waiting for launch
    //  we want high g mode to get the early few readings of the launch
    if (board_mode == FLYING && acc_axis_read) {
      if (!set_acc_mode(filtered_sqr_mag >= ACC_HIGH_G_SWITCH)) {
        note_error("Mode switch failed", IMU_ERR);
      }
    }

    watchdog_update();
  }

  write_data(Acc{acc_axis.x(), acc_axis.y(), acc_axis.z()});
  write_data(Gyro{gyro_axis.x(), gyro_axis.y(), gyro_axis.z()});
}

void sample_current() {
  if (current_sens_failed) {
    return;
  }

  current_sensor.read();

  write_data(Current{
    current_sensor.bus_millivolts(),
    current_sensor.temperature_millicelsius(),
    current_sensor.current_milliamps(),
    current_sensor.power_microwatts()
  });
}

// This handles what the board should do when it has reached a critical failure
// There is no reason to not just reboot unless we are in debug in which case we can
// disable the watchdog and sleep to show what happened
void do_failure() {
#ifndef DEBUG
  watchdog_reboot(0, 0, 0);
#else
  watchdog_disable();
  delay(1000);
#endif
}

// TODO: We could make the loop schedule in a way that does more barometer readings
//  because we don't really need to run the loop a bunch between barometer readings
//  so we could just run the loop then wait for the next barometer reading to be
//  ready (this would mess with the flash write rate if not done well). The
//  loop may be fast enough it doesn't matter
void loop() {
  // If we have reached critical failure then we return early
  if (board_mode == FAILURE) {
    do_failure();
    return;
  }

  // Sample the sensors (this updates the relevant state object)
  // The barometer only provides samples at the odr sampling rate
  //  so this function just advances the sampling process if possible
  // We sample the imu first since it contains a buffer of all the samples
  //  and we want the samples fed to the flight state roughly in order
  //  so we want the buffer to be empty when reading the pressure sensor
  //  to ensure it is in order roughly
  sample_imu();
  step_sample_baro();
  // This is current only used for logging, but may be used to increase boot speed in UNKNOWN mode,
  //  by sensing a safe time to activate the servo
  sample_current();

  // Update the servo based on the state object
  update_servo();

  // This is slow from UNKNOWN to UNARMED
  //  however we don't care since it is on the ground flight
  //  state and so losing accelerometer data in the fifo
  //  doesn't really matter
  update_mode();

  if (millis_in_mode() >= next_flash_write) {
    flash_push_state(flight_state.get_flash());

    next_flash_write += FLASH_SAMPLE_RATE;
  }

  watchdog_update();
}

