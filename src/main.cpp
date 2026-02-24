#include <ISM6HG256XSensor.h>
#include <SoftwareSPI.h>
#include <MS5611_SPI.h>
#include <SPI.h>
#include <RP2040_PWM.h>
#include <cmath>
#include <cstdint>

#include "pins.h"
#include "state.h"
#include "logging.h"
#include "led.h"
#include "util.h"

// NOTE: This code uses millis() extensively and assumes it will not overflow (it will overflow in >40 days and that is not intended usage)
// TODO: Look into gyro saturation
// TODO: Look into pressure drop when hitting around mach numbers
// TODO: Implement watchdog and look into how to store memory between watchdog reboots (probably write important data to flash on watchdog)
//  this requires important data to be written such that the watchdog is able to read them in a consistent state at anytime
// TODO: Consider checking the arming pin in UNKNOWN status to prevent false positives

BoardMode board_mode = BOOTING;
Millis last_mode_change = 0;

FlightState flight_state = FlightState();
RestState rest_state = RestState();

bool servo_powered = false;

// The pins aren't correctly assigned for hardware SPI on the board
// I assume it is a mistake (?) so we have to use bit banging
SoftwareSPI softSPI(SPI_SCK, SPI_MISO, SPI_MOSI);

MS5611_SPI baro(BAROMETER_CS, &softSPI);
ISM6HG256XSensor imu(&softSPI, IMU_CS);

// The servo has an operating frequency of 50-300Hz
RP2040_PWM servo(SERVO_1, (float)SERVO_FREQ, 0.0f);

Millis next_sample;
const Millis sample_size_ms = 100;
const float sample_size_s = sample_size_ms / SECONDS_TO_MILLIS;

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

void push_mode(BoardMode mode) {
  log_message(ModeChange{board_mode, mode});

  leds[LED_STATUS] = MODE_TO_COLOR[mode];
  led_show();

  board_mode = mode;
  last_mode_change = millis();
}

Millis millis_in_mode() {
  // This should never happen
  if (last_mode_change > millis()) {
    log_message("Mode changed marked in future");
    return 0;
  }

  return millis() - last_mode_change;
}

// Pushing LED_STATUS gets overwritten immediatly so is equivalent to a failure with no origin
void push_failure(LEDs failure_led = LED_STATUS) {
  leds[failure_led] = LED_NEGATIVE;
  // Push mode updates the LEDS so we don't need to call led_show
  led_show();

  push_mode(FAILURE);
}

// NOTE: Init values are temporary and will be determined by data later
void setup() {
#ifdef DEBUG
  // Allow some time for the serial monitor to connect
  delay(DEBUG_BOOT_DELAY);
#endif

  // TODO: Check if this is a watchdog boot

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
  // Same with the magnetometer
  leds[LED_MAGN] = LED_NEUTRAL;
  led_show();

  // Initialize the LED the rp2040 has two SPIs and we init the first one to be able to communicate to the sensors
  softSPI.begin();
  log_message("SPI inited");

  // Initialize the barometer we has an MS5607, but the interface should be the same as the MS5611 which
  // is the library we are using
  bool baro_init = baro.begin();
  // Since we are using softwareSPI we have to set the rate in this library
  // This is what the brinup code sets it to so ...
  baro.setSPIspeed(10000000);
  // This sampling will change in the future
  baro.setOversampling(OSR_ULTRA_HIGH);

  if (baro_init) { log_message("Barometer inited"); }
  leds[LED_BARO] = baro_init ? LED_POSITIVE : LED_NEGATIVE;
  led_show();

  // The imu returns custom status values instead of booleans
  // So the returns are checked (as shown in their example code you can also or all these values and then check ISM6HG256_OK at the end)
  bool imu_init = imu.begin() == ISM6HG256X_OK;

  // TODO: Figure out high G sensor the library doesn't seem to support it in the fifo, but the datasheet does

  // Enable all the sensors on the imu. It has two accelerometers one for low g and one for high g.
  // Both can be active at once
  imu_init &= imu.Enable_G() == ISM6HG256X_OK;
  imu_init &= imu.Enable_X() == ISM6HG256X_OK;

  // Set the ouput rate
  imu_init &= imu.Set_G_OutputDataRate(GYRO_RATE) == ISM6HG256X_OK;
  imu_init &= imu.Set_X_OutputDataRate(ACC_RATE) == ISM6HG256X_OK;

  // Set the rate at which data is stored in the fifo (I believe)
  imu_init &= imu.FIFO_G_Set_BDR(GYRO_RATE) == ISM6HG256X_OK;
  imu_init &= imu.FIFO_X_Set_BDR(ACC_RATE) == ISM6HG256X_OK;

  // Set Set FIFO watermark level
  imu_init &= imu.FIFO_Set_Watermark_Level(199) == ISM6HG256X_OK;
  // Set FIFO stop on watermark level
  imu_init &= imu.FIFO_Set_Stop_On_Fth(1) == ISM6HG256X_OK;

  // Enable the fifo in continious mode (ie it overwrites old samples)
  imu_init &= imu.FIFO_Set_Mode(ISM6HG256X_STREAM_MODE) == ISM6HG256X_OK;

  // TODO: Set up the accelerometer mode currently ISM6HG256X_ACC_HIGH_ACCURACY_ODR_MODE
  //  just immediatly causes the init to do nothing and return error

  if (baro_init) { log_message("IMU inited"); }
  leds[LED_IMU] = imu_init ? LED_POSITIVE : LED_NEGATIVE;
  led_show();

  if (baro_init && imu_init) {
    // The board is now ready to be armed
    push_mode(UNKNOWN);
  } else {
    // The board has failed to init
    push_mode(FAILURE);
  }

  next_sample = millis();
}

// This is called when the board confirms that it has booted an is on the ground waiting to launch
// It could take some time to run (because it waits for the log core), but it shouldn't because the
//  log core should boot fast
void ground_boot() {
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
  switch (board_mode) {
    case UNKNOWN:
      // If booted during flight we should know our before 
      if (rest_state.try_init_flying_boot(flight_state)) {
        push_mode(FLYING);
      } else if (millis_in_mode() >= UNKNOWN_WAIT) {
        ground_boot();
        push_mode(UNARMED);
      }

      break;

    case UNARMED:
      if (digitalRead(ARM_SWITCH) == LOW) {
        push_mode(ARMED);
      }

      break;

    case ARMED:
      // If the rest_state can init the flying state then it means that it has detected high acceleration
      //  and we are in flight
      if (rest_state.try_init_flying(flight_state)) {
        push_mode(FLYING);
      }

      if (digitalRead(ARM_SWITCH) == HIGH) {
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
      push_failure();

      break;
  }
}

// TODO: Handle errors
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

  float duty_percent = SERVO_FLUSH;

  if (board_mode == FLYING) {
    duty_percent = flight_state.get_servo();
  } else if (board_mode == UNARMED) {
    // Just a generic parabola (maxed with 0) to generate the full range of motion over a few seconds
    // It is 0 at 1500 and 4500 millis and peaks at 1 since it is 0 at 1500 millis that gives
    // the servo 1500 to zero since we don't know its position
    float time = millis_in_mode() / SECONDS_TO_MILLIS;
    float duty_percent = max(-(time - 1.5f) * (time - 4.5f) / 2.25f, 0.0f);
  }

  servo.setPWM(SERVO_1, (float)SERVO_FREQ, (float)(duty_percent * SERVO_FREQ));
}

// TODO: Check self heating mentioned for similar product in MS5xxx library docs
// TODO: Add error handling
void sample_baro() {
  baro.read();
  float temp = baro.getPressure();
  float pressure = baro.getTemperature();

  if (board_mode == FLYING) {
    flight_state.push_baro(temp, pressure);
  } else if (board_mode == UNKNOWN || board_mode == UNARMED || board_mode == ARMED) {
    rest_state.push_baro(temp, pressure);
  }
}

// TODO: Add error handling
void sample_imu() {
  ISM6HG256X_Axes_t acc_axis;
  ISM6HG256X_Axes_t gyro_axis;

  uint16_t samples;
  imu.FIFO_Get_Num_Samples(&samples);

  uint8_t tag;
  for (uint16_t i = 0; i < samples; i++) {
    imu.FIFO_Get_Tag(&tag);

    switch (tag) {
      case 1:
        imu.FIFO_G_Get_Axes(&gyro_axis);

        if (board_mode == FLYING) {
          flight_state.push_gyro(Eigen::Vector3f(gyro_axis.x, gyro_axis.y, gyro_axis.z));
        } else if (board_mode == UNKNOWN || board_mode == UNARMED || board_mode == ARMED) {
          rest_state.push_gyro(Eigen::Vector3f(gyro_axis.x, gyro_axis.y, gyro_axis.z));
        }

        break;

      case 2:
        imu.FIFO_X_Get_Axes(&acc_axis);

        if (board_mode == FLYING) {
          flight_state.push_acc(Eigen::Vector3f(acc_axis.x, acc_axis.y, acc_axis.z));
        } else if (board_mode == UNKNOWN || board_mode == UNARMED || board_mode == ARMED) {
          rest_state.push_acc(Eigen::Vector3f(acc_axis.x, acc_axis.y, acc_axis.z));
        }

        break;

      default:
        break;
    }
  }

#ifdef CALIBRATION
  write_calib(AccCalib(acc_axis));
  write_calib(GyroCalib(gyro_axis));
#endif
}

// This handles what the board should do when it has reached a critical failure
void do_failure() {
  delay(1000);
}

void loop() {
  // If we have reached critical failure then we return early
  if (board_mode == FAILURE) {
    do_failure();
    return;
  }

  // Sample the sensors (this updates the relevant state object)
  sample_baro();
  sample_imu();

  // Update the servo based on the state object
  update_servo();

  update_mode();

  next_sample += sample_size_ms;
  if (!delay_to(next_sample)) {
    log_message("Loop overrun");
  }
}

