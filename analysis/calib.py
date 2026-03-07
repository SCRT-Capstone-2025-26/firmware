import argparse
import data_parse
import scipy.optimize as opt
import matplotlib.pyplot as plt
import math

# NOTE: steady_slices should not overlap
# TODO: It should calibrate sensitivities for each x, y, and z axis as well as the bias since there
#  was 1.02g error after bias was accounted for on one of the hg accs

# NOTE: This should be your local gravity
G = 9.8057

def calib_acc(accs, steady_slices):
    # get all the slices in a list
    acc_slices = [accs[a:b] for a, b in steady_slices]

    # Compute the mean for each slice
    means = [data_parse.Acc(
        sum(acc.x for acc in accs) / len(accs),
        sum(acc.y for acc in accs) / len(accs),
        sum(acc.z for acc in accs) / len(accs)
    ) for accs in acc_slices]

    # Since the rest accelerometer reading is g + bias we have to seperate them
    # That means there should be a scaler g and bias b such that
    #  |mean - bias| = g
    def loss(x):
        g, bias_x, bias_y, bias_z = x

        error = 0
        for mean in means:
            norm_sq = \
                (mean.x - bias_x) ** 2 + \
                (mean.y - bias_y) ** 2 + \
                (mean.z - bias_z) ** 2

            error += (math.sqrt(norm_sq) - g) ** 2

        return error

    g, *bias = opt.minimize(loss, (1, 0, 0, 0)).x
    return float(g) / G, data_parse.Acc(*(float(bias_entry) for bias_entry in bias))


def calib_gyro(gyros, steady_slices):
    # get all the slices in a list
    gyro_slices = [gyros[a:b] for a, b in steady_slices]
    # Combine all the slides together into one mega list
    gyros = sum(gyro_slices, [])

    # Compute the mean which is the bias
    mean = data_parse.Gyro(
        sum(gyro.x for gyro in gyros) / len(gyros),
        sum(gyro.y for gyro in gyros) / len(gyros),
        sum(gyro.z for gyro in gyros) / len(gyros)
    )

    return mean


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("path")
    args = parser.parse_args()

    with open(args.path, 'rb') as file:
        items = data_parse.read_all(file)

    accs = [acc for (_, acc) in items if isinstance(acc, data_parse.Acc)]
    gyros = [gyro for (_, gyro) in items if isinstance(gyro, data_parse.Gyro)]

    plt.plot(accs)
    plt.show(block=False)

    slices = int(input('Steady Regions: '))

    steady_slices = []
    for i in range(slices):
        low = int(input('Low: '))
        high = int(input('High: '))
        steady_slices.append((low, high))

    print(calib_acc(accs, steady_slices))
    print(calib_gyro(gyros, steady_slices))

