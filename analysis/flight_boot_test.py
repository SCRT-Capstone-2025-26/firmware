from log_check import LogExpectation, LogChecker
from tests.run_test import Done, State

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

flying = LogExpectation('^Unknown -> Flying$', core=1, latest_time=60)
# We expect the flash reinit to fail since flash is disabled
flash_reinit = LogExpectation('^ERROR: Flash reinit failed$', core=1, latest_time=60)

servo = LogExpectation('^Servo powered$', core=1, latest_time=2010)

checker.add_expected(servo)

core_0 = [serial, sd, file_num]
core_1 = [pins, leds, booting, test_mode, spi, baro, imu, unknown, flying, flash_reinit]

# Adding follows also adds them as expectations
for previous, next in zip(core_0[:-1], core_0[1:]):
    checker.add_follows(previous, next)

for previous, next in zip(core_1[:-1], core_1[1:]):
    checker.add_follows(previous, next)

# Data and logs are already parsed
def check_sample(log, data):
    return checker.check(log)


def run_test(_):
    acc_data = (0.0, 0.0, 0.0)
    hg_acc_data = (0.0, 0.0, 0.0)

    gyro_data = (0.0, 0.0, 0.0)

    baro_data = (0.0, 0.0)

    # Since the code does linear interpolation this will make it always have the given state
    yield State(0, acc_data, hg_acc_data, gyro_data, baro_data)
    yield State(1, acc_data, hg_acc_data, gyro_data, baro_data)

    yield Done(3)

