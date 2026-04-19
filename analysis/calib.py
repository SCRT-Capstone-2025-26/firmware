import argparse
import parse
import scipy.optimize as opt
import matplotlib.pyplot as plt
import math

# NOTE: steady_slices should not overlap

def calib_acc(g, accs, steady_slices):
    # get all the slices in a list
    acc_slices = [accs[a:b] for a, b in steady_slices]

    # Compute the mean for each slice
    means = [parse.Acc(
        sum(acc.x for acc in accs) / len(accs),
        sum(acc.y for acc in accs) / len(accs),
        sum(acc.z for acc in accs) / len(accs)
    ) for accs in acc_slices]

    # Since the rest accelerometer reading is g + bias we have to seperate them
    # That means there should be a scaler g and bias b such that
    #  |(mean * sens) - bias| = g
    def loss(x):
        sens_x, sens_y, sens_z, bias_x, bias_y, bias_z = x

        error = 0
        for mean in means:
            norm_sq = \
                ((mean.x * sens_x) - bias_x) ** 2 + \
                ((mean.y * sens_y) - bias_y) ** 2 + \
                ((mean.z * sens_z) - bias_z) ** 2

            error += (math.sqrt(norm_sq) - g) ** 2

        return error

    return opt.minimize(loss, (1, 1, 1, 0, 0, 0)).x


def calib_gyro(gyros, steady_slices):
    # get all the slices in a list
    gyro_slices = [gyros[a:b] for a, b in steady_slices]
    # Combine all the slides together into one mega list
    gyros = sum(gyro_slices, [])

    # Compute the mean which is the bias
    mean = parse.Gyro(
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
        items = parse.read_all(file)

    accs = [acc for (_, acc) in items if isinstance(acc, parse.Acc)]
    gyros = [gyro for (_, gyro) in items if isinstance(gyro, parse.Gyro)]

    plt.plot(accs)

    plt.legend(['Accelerometer X Axis', 'Accelerometer Y Axis', 'Accelerometer Z Axis'])
    plt.ylabel('Acceleration')
    plt.xlabel('Index')

    plt.show(block=False)

    slices = int(input('How many steady regions: '))

    steady_slices = []
    for i in range(slices):
        low = int(input(f'Region {i + 1} start index: '))
        high = int(input(f'Region {i + 1} end index: '))
        steady_slices.append((low, high))

    g = float(input('What is g: '))

    acc_sensbias = calib_acc(g, accs, steady_slices)
    gyro_bias = calib_gyro(gyros, steady_slices)
    print(f'Acc sens {acc_sensbias[:3]}')
    print(f'Acc bias {acc_sensbias[3:]}')
    print(f'Gyro bias {gyro_bias}')

