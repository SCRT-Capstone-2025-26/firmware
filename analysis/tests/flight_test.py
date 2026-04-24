from math import sqrt
from log_check import LogExpectation, LogChecker

# 10,000ft in meters
apogee = 10000 / 3.28084
g = 9.80665
# y = -gt^2 + v_0t
# apogee = -gt_a^2 + v_0t_a
# t_a = v_0 / 2g since the vertex is minus b over 2 a
# apogee = -g(v_0 / 2g)^2 + v_0(v_0 / 2g)
# apogee = -v_0^2 / 4g + v_0^2 / 2g = v_0^2 / 4g
# v_0 = 2sqrt(apogee * g)
v_0 = 2 * sqrt(apogee * g)
t_a = v_0 / (2 * g) # also sqrt(apogee / g)

# Five seconds before the launc starts
t_0 = 4
# Burn out happens a second later
t_b = 1
# We need the thrust that will get us to end in parabola defined above
#  this is a rough estimate
burn_acc = v_0 / t_b

# 500ms steps
# Its fine since it interpolates
micros_per_step = 5000
# This calculates have many steps of 500 micros it takes to reach 1.2 * t_a + 3
# There is are 3 seconds before launch
max_steps = int(((t_a * 1.2) + t_0) / micros_per_step * 1000 * 1000)

# It is somewhat counterintuitive, but we are in free fall for most of the flight
acc_data = [(0.0, 0.0, 0.0)] * max_steps
hg_acc_data = [(0.0, 0.0, 0.0)] * max_steps

gyro_data = [(0.0, 0.0, 0.0)] * max_steps

acc_noise = 0.75
hg_acc_noise = 1.5

gyro_noise = 0.01

baro_noise = 5.0

def height(t):
    return (-g * (t ** 2)) + (v_0 * t)


# This is the formula used by https://github.com/RobTillaart/MS5611
# 44307.694 * (1 - pow(pressure / SEA_LEVEL_PRESURE, 0.190284))
def h_to_pres(height):
    return 1013.5 * ((1 - (height / 44307.694)) ** (1 / 0.190284))


baro_data = []
for i in range(max_steps):
    # Time in seconds 0 being launch
    t = (i * micros_per_step / 1000 / 1000) - t_0
    # Five seconds of pre launch
    if t <= 0.0:
        # Z is up and it is counterintuitive, but resting on the ground
        #  is actually moving up at g
        acc_data[i] = (0.0, 0.0, g)
        hg_acc_data[i] = (0.0, 0.0, g)
        baro_data.append((h_to_pres(0.0), 0.0))
    elif t <= t_b:
        # Just roughly simulate accelerating up to v_0 over t_b
        acc_data[i] = (0.0, 0.0, burn_acc - g)
        hg_acc_data[i] = (0.0, 0.0, burn_acc - g)
        baro_data.append((h_to_pres(height(t)), 0.0))
    else:
        baro_data.append((h_to_pres(height(t)), 0.0))

checker = LogChecker()

serial = LogExpectation('^Serial inited$', core=0, latest_time=10)
pins = LogExpectation('^Pins inited$', core=1, latest_time=10)
leds = LogExpectation('^LEDs inited$', core=1, latest_time=10)

booting = LogExpectation('^Booting -> Booting$', core=1, latest_time=10)
board_id = LogExpectation('^Board ID: 0x[0-9a-f]{16}$', core=1, latest_time=10)
uncalibrated = LogExpectation(r'^NOTE: Using default calibration values \(aka this board is uncalibrated\)$', core=1, latest_time=10)
test_mode = LogExpectation('^Board is in TEST mode$', core=1, latest_time=10)
spi = LogExpectation('^SPI inited$', core=1, latest_time=10)
baro = LogExpectation('^Barometer inited$', core=1, latest_time=20)
imu = LogExpectation('^IMU inited$', core=1, latest_time=20)
unknown = LogExpectation('^Booting -> Unknown$', core=1, latest_time=20)

sd = LogExpectation('^SD inited$', core=0, latest_time=20)
# Finding a file takes longer the more files
file_num = LogExpectation('^File number [0-9]+ found$', core=0, latest_time=500)

flying = LogExpectation('^Unknown -> Flying$', core=1, latest_time=60)
# We expect the flash reinit to fail since flash is disabled
flash_reinit = LogExpectation('^ERROR: Flash reinit failed$', core=1, latest_time=60)

servo = LogExpectation('^Servo powered$', core=1, latest_time=2010)

checker.add_expected(servo)

core_0 = [serial, sd, file_num]
core_1 = [pins, leds, booting, board_id, uncalibrated, test_mode, spi, baro, imu, unknown, flying, flash_reinit]

# Adding follows also adds them as expectations
for previous, next in zip(core_0[:-1], core_0[1:]):
    checker.add_follows(previous, next)

for previous, next in zip(core_1[:-1], core_1[1:]):
    checker.add_follows(previous, next)

# Data and logs are already parsed
def check_sample(log, data):
    # Since we can't import when this is run to build test data
    #  only to validate
    import parse

    for millis, item in data:
        t = (millis / 1000) - t_0
        if t < 0.0:
            continue

        if isinstance(item, parse.FilterState):
            y = height(t)
            error = abs(item.h - y)
            # The t > x cases gives the filter a bit of time to catch up
            # Since the starting velocities are different
            # These should be made tighter
            if (error > 0.5 and t > 10) or (error > 2 and t > 3):
                return False, f'Height estimated to be {y}m at {item.h}m {t}s in flight'

    return checker.check(log)

