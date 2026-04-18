import struct
from collections import namedtuple
import argparse

Acc = namedtuple('Acc', ('x', 'y', 'z'))
Gyro = namedtuple('Gyro', ('x', 'y', 'z'))
Baro = namedtuple('Baro', ('pressure', 'tempurate'))
Servo = namedtuple('Servo', ('percent'))
# The units here are not the standard SI units
Current = namedtuple('Current', ('voltage', 'temp', 'current', 'power'))
FilterState = namedtuple('FilterState', ('h', 'v', 'h_cov', 'v_cov', 'hv_cov'))
RotState = namedtuple('RotState', ('x', 'y', 'z', 'w'))

Log = namedtuple('Log', ('time', 'core', 'message'))

item_types = {
    b'A': ('fff', Acc),
    b'G': ('fff', Gyro),
    b'B': ('ff', Baro),
    b'S': ('f', Servo),
    b'C': ('HiiI', Current),
    b'F': ('fffff', FilterState),
    b'R': ('ffff', RotState)
}

# Can unpack_from be used?
def read_item(file):
    id = file.read(1)
    if id == b'':
        return None

    packing, item_type = item_types[id]
    packing = '<L' + packing

    data = file.read(struct.calcsize(packing))
    timestamp, *args = struct.unpack(packing, data)

    return timestamp, item_type(*args)


def read_all(file):
    items = []
    while True:
        item = read_item(file)
        if item is None:
            break

        items.append(item)

    return items


def read_log(line):
    header, content = line.split('] ')
    timestamp, core = header.split(', ')

    timestamp = int(timestamp.split(': ')[1].split('ms')[0])
    core = int(core.split(': ')[1])

    return Log(timestamp, core, content)


def read_logs(file):
    lines = file.read().splitlines()
    logs = [read_log(line) for line in lines[:-1]]
    if lines[-1] != '':
        logs.append(read_log(lines[-1]))

    return logs


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("path")
    args = parser.parse_args()

    with open(args.path, 'rb') as file:
        for item in read_all(file):
            print(item)


