from test_comm import State
from scrt.sim import Sim
import time
import math

import parse

# This is the formula used by https://github.com/RobTillaart/MS5611
# 44307.694 * (1 - pow(pressure / SEA_LEVEL_PRESURE, 0.190284))
def h_to_pres(height):
    return 1013.5 * ((1 - (height / 44307.694)) ** (1 / 0.190284))


def get_sensor_data(sim, ground=False):
    h, _, theta = sim.curr_state

    baro_data = (h_to_pres(h), 0.0)

    if ground:
        current_accel = 9.81
    else:
        current_accel = sim.accel(sim.curr_time, sim.curr_state, sim.ext)

    acc_data = (0.0, current_accel * math.sin(theta), current_accel * math.cos(theta))
    hg_acc_data = acc_data

    return {
        "acc": acc_data,
        "acc_noise": (1.0, 1.0, 1.0),
        "hg_acc": hg_acc_data,
        "hg_acc_noise": (1.0, 1.0, 1.0),
        "gyro": (0.0, 0.0, 0.0),
        "gyro_noise": (1.0, 1.0, 1.0),
        "baro": baro_data,
        "baro_noise": (10.0, 0.0)
    }


def run_test(dm):
    sim = Sim()
    # 261 low, 281 high
    sim.set_state(state=(800, 281, 0), time=10, max_step=0.1)

    start = get_sensor_data(sim, True)
    dm.send(State(0, **start))
    st = time.time()
    dm.send(State(5 * 1000 * 1000, **start))

    t = 0
    for _ in range(50):
        dm.update()

        t += 0.1
        if t + st - time.time() > 0:
            time.sleep(t + st - time.time())

    while time.time() - st < 20:
        dm.update()

        servo = next((x for x in reversed(dm.get_current_data()) if isinstance(x[1], parse.Servo)), None)
        if servo is not None:
            sim.ext = servo[1].percent

        st1 = sim.curr_time
        while st1 + 0.1 >= sim.curr_time:
            sim.step_state()
        t = 5 + sim.curr_time

        dm.send(State(int(t * 1000 * 1000), **get_sensor_data(sim)))

        if t + st - time.time() > 0:
            time.sleep(t + st - time.time())

