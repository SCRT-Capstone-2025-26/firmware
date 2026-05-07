from log_check import LogExpectation, LogChecker
from run_test import Done, State
import sys
import importlib.util
import pathlib
import time

import parse

# TODO: Repackage the sim
test_path = pathlib.Path('../SCRT_Rocket_SIM/simulation/sim.py')
sys.path.append(str(test_path.parent))
spec = importlib.util.spec_from_file_location('sim', test_path)
s = importlib.util.module_from_spec(spec)
spec.loader.exec_module(s)
sys.path.pop()

# This is the formula used by https://github.com/RobTillaart/MS5611
# 44307.694 * (1 - pow(pressure / SEA_LEVEL_PRESURE, 0.190284))
def h_to_pres(height):
    return 1013.5 * ((1 - (height / 44307.694)) ** (1 / 0.190284))

# lol
def get_sensor_data(self):
    h, v, theta = self.u
    t = self.t
    
    current_accel = self.accel(t, self.u, None)
    
    baro_data = (float(h_to_pres(h)), 0.0)
    
    acc_data = (0.0, 0.0, float(current_accel))
    hg_acc_data = acc_data
    
    return {
        "acc": acc_data,
        "acc_noise": (0.0, 0.0, 0.0),
        "hg_acc": hg_acc_data,
        "hg_acc_noise": (0.0, 0.0, 0.0),
        "gyro": (0.0, 0.0, 0.0),
        "gyro_noise": (0.0, 0.0, 0.0),
        "baro": baro_data,
        "baro_noise": (0.0, 0.0)
    }


def run_test(dm):
    sim = s.Sim()

    acc_data = (0.0, 0.0, 9.81)
    acc_noise = (0.0, 0.0, 1.0)
    hg_acc_data = (0.0, 0.0, 9.81)
    hg_acc_noise = (0.0, 0.0, 1.0)

    gyro_data = (0.0, 0.0, 0.0)
    gyro_noise = (0.0, 0.0, 0.0)

    baro_data = (h_to_pres(0.0), 0.0)
    baro_noise = (0.0, 0.0)

    # Since the code does linear interpolation this will make it always have the given state
    dm.send(State(0, acc_data, acc_noise, hg_acc_data, hg_acc_noise, gyro_data, gyro_noise, baro_data, baro_noise))
    dm.send(State(5 * 1000 * 1000, acc_data, acc_noise, hg_acc_data, hg_acc_noise, gyro_data, gyro_noise, baro_data, baro_noise))

    for i in range(50):
        dm.update()
        time.sleep(0.1)

    sim.ext = 0
    t = 0
    st = time.time()
    while True:
        servo = next((x for x in reversed(dm.get_current_data()) if isinstance(x[1], parse.Servo)), None)
        if servo is not None:
            sim.ext = servo[1].percent

        sim.step(None, 0.1)
        t += 0.1
        print(int((5 + t) * 1000 * 1000))
        dm.send(State(int((5 + t) * 1000 * 1000), **get_sensor_data(sim)))
        if t + st - time.time() > 0:
            time.sleep(t + st - time.time())
        dm.update()

