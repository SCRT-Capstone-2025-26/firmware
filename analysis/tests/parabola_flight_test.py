from log_check import LogExpectation, LogChecker

# 10,000ft in meters
apogee = 10000 / 3.28084
g = 9.81
# y = -gt^2 + v_0t
# apogee = -gt_a^2 + v_0t_a
# t_a = g / 2v_0 since the vertex is minus a over 2 b
# apogee = -g(g / 2v_0)^2 + v_0(g / 2v_0)
# apogee = -g^3 / 4v_0^2 + g / 2

max_steps = 1
micros_per_step = 3 * 1000 * 1000

acc_data = [(0.0, 0.0, 9.81)]
hg_acc_data = [(0.0, 0.0, 9.81)]

gyro_data = [(0.0, 0.0, 0.0)]

baro_data = [(0.0, 0.0)]

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
    return checker.check(log)
