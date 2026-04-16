from math import sqrt
from log_check import LogExpectation, LogChecker
import parse

# 10,000ft in meters
apogee = 10000 / 3.28084
g = 9.81
# y = -gt^2 + v_0t
# apogee = -gt_a^2 + v_0t_a
# t_a = v_0 / 2g since the vertex is minus b over 2 a
# apogee = -g(v_0 / 2g)^2 + v_0(v_0 / 2g)
# apogee = -v_0^2 / 4g + v_0^2 / 2g = v_0^2 / 4g
# v_0 = 2sqrt(apogee * g)
v_0 = 2 * sqrt(apogee * g)
t_a = v_0 / (2 * g) # also sqrt(apogee / g)

# 5ms steps
micros_per_step = 500
# This calculates have many steps of 500 micros it takes to reach 1.2 * t_a
max_steps = int(t_a * 1.2 / micros_per_step * 1000 * 1000)

# It is somewhat counterintuitive, but we are in free fall
acc_data = [(0.0, 0.0, 0.0)] * max_steps
hg_acc_data = [(0.0, 0.0, 0.0)] * max_steps

gyro_data = [(0.0, 0.0, 0.0)] * max_steps

def height(t):
    return (-g * (t ** 2)) + (v_0 * t)


# This is the formula used by https://github.com/RobTillaart/MS5611
# 44307.694 * (1 - pow(pressure / SEA_LEVEL_PRESURE, 0.190284))
def h_to_pres(height):
    sea_level_pressure = 1013.15 * 1e2
    return sea_level_pressure * ((1 - (height / 44307.694)) ** (1 / 0.190284))


baro_data = []
for i in range(max_steps):
    # Time in seconds
    t = i * micros_per_step / 1000 / 1000
    baro_data.append((h_to_pres(height(t)), 0.0))

checker = LogChecker()

serial = LogExpectation('^Serial inited$', core=0, latest_time=10)
pins = LogExpectation('^Pins inited$', core=1, latest_time=10)
leds = LogExpectation('^LEDs inited$', core=1, latest_time=10)

booting = LogExpectation('^Booting -> Booting$', core=1, latest_time=10)
test_mode = LogExpectation('^Board is in TEST mode$', core=1, latest_time=10)
spi = LogExpectation('^SPI inited$', core=1, latest_time=10)
baro = LogExpectation('^Barometer inited$', core=1, latest_time=20)
imu = LogExpectation('^IMU inited$', core=1, latest_time=20)
unknown = LogExpectation('^Booting -> Unknown$', core=1, latest_time=20)

sd = LogExpectation('^SD inited$', core=0, latest_time=20)
# Finding a file takes longer the more files
file_num = LogExpectation('^File number [0-9]+ found$', core=0, latest_time=500)

unarmed = LogExpectation('^Unknown -> Unarmed$', core=1, latest_time=2020)
log_wait = LogExpectation('^Waiting on log core$', core=1, latest_time=2020)
log_boot = LogExpectation('^Log core booted$', core=1, latest_time=2020)
sd_init_c1 = LogExpectation('^SD inited$', core=1, latest_time=2020)
# Flash is disabled on the test so this should be fast
flash_clear = LogExpectation('^Clearing flash$', core=1, latest_time=2020)
armed = LogExpectation('^Unarmed -> Armed$', core=1, latest_time=2020)

servo = LogExpectation('^Servo powered$', core=1, latest_time=2010)

checker.add_expected(servo)

core_0 = [serial, sd, file_num]
core_1 = [pins, leds, booting, test_mode, spi, baro, imu, unknown, unarmed, log_wait, log_boot, sd_init_c1, flash_clear, armed]

# Adding follows also adds them as expectations
for previous, next in zip(core_0[:-1], core_0[1:]):
    checker.add_follows(previous, next)

for previous, next in zip(core_1[:-1], core_1[1:]):
    checker.add_follows(previous, next)

# Data and logs are already parsed
def check_sample(log, data):
    for millis, item in data:
        t = millis / 1000
        if isinstance(item, parse.FilterState):
            y = height(t)
            error = abs(item.h - y)
            # The t > x cases gives the filter a bit of time to catch up
            # Since the starting velocities are different
            # These should be made tighter
            if (error > 0.5 and t > 10) or (error > 2 and t > 3):
                return False, f'Height estimated to be {y}m at {item.h}m {t}s in flight'

    return checker.check(log)

