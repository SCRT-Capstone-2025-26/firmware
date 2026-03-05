import kalman
import random
from matplotlib import pyplot as plt

# TODO: Better angle randomness sim
def run_sim(
        launch_time, launch_acc,
        launch_acc_bias, launch_acc_noise, launch_acc_filter_noise, launch_theta_noise,
        acc_bias, acc_noise, theta_noise, acc_filter_noise, baro_bias, baro_noise, baro_filter_noise, baro_sample_chance_per_sec,
        filter=None, plot=False):
    if filter is None:
        filter = kalman.KalmanFilter(0.1, 0.3, 0.5)

    filter_sim = kalman.FilterSim(filter)

    true_h = 0
    true_v = 0

    true_hs = [true_h]
    true_vs = [true_v]
    t = [filter_sim.time]

    while filter_sim.time < launch_time:
        filter_sim.step(launch_acc + ((random.random() - 0.5) * launch_acc_noise) + launch_acc_bias, launch_acc_filter_noise, random.random() * launch_theta_noise)

        # I believe swapping the integration is numerically better for some reason
        # I should just use like scipy or somthing
        true_h += true_v / filter_sim.step_rate
        true_v += launch_acc / filter_sim.step_rate

        true_hs.append(true_h)
        true_vs.append(true_v)
        t.append(filter_sim.time)


    while filter_sim.time < 20:
        if random.random() < baro_sample_chance_per_sec / filter_sim.step_rate:
            filter_sim.sample(true_h + ((random.random() - 0.5) * baro_noise) + baro_bias, baro_filter_noise)
        filter_sim.step(-1 + ((random.random() - 0.5) * acc_noise) + acc_bias, acc_filter_noise, random.random() * theta_noise)

        # I believe swapping the integration is numerically better for some reason
        # I should just use like scipy or somthing
        true_h += true_v / filter_sim.step_rate
        true_v += -1 / filter_sim.step_rate

        true_hs.append(true_h)
        true_vs.append(true_v)
        t.append(filter_sim.time)


    hs = list(map(lambda state: state[0], filter_sim.states))
    vs = list(map(lambda state: state[1], filter_sim.states))

    if plot:
        plt.plot(t, hs, t, true_hs)
        plt.show()

        plt.plot(t, vs, t, true_vs)
        plt.show()

    return t, hs, vs, true_hs, true_vs


if __name__ == '__main__':
    run_sim(
        6, 12,
        -3, 6, 4, 0.3,
        0.1, 0.5, 0.1, 1, 0, 25, 100, 2,
        plot=True
    )

    run_sim(
        6, 10,
        0, 4, 5, 0.2,
        0.1, 0.3, 0.05, 1, 0, 100, 100000, 2,
        plot=True
    )

    run_sim(
        6, 10,
        0, 4, 5, 0.2,
        0.1, 0.3, 0.05, 1, 10, 10, 10, 2,
        plot=True
    )

    run_sim(
        6, 10,
        0, 0.1, 0.05, 0.2,
        0.1, 0.3, 0.05, 0.05, 10, 10, 1000, 2,
        plot=True
    )

