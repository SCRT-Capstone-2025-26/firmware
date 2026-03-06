import struct
from collections import namedtuple
import argparse

Acc = namedtuple('Acc', ('x', 'y', 'z'))
Gyro = namedtuple('Gyro', ('x', 'y', 'z'))
Baro = namedtuple('Baro', ('pressure', 'tempurate'))
Servo = namedtuple('Servo', ('percent'))
# The units here are not the standard SI units
Current = namedtuple('Current', ('voltage', 'temp', 'current', 'power'))

item_types = {
    b'A': ('<Lfff', Acc),
    b'G': ('<Lfff', Gyro),
    b'B': ('<Lff', Baro),
    b'S': ('<Lf', Servo),
    b'C': ('<LHiiI', Current),
}

# Can unpack_from be used?
def read_item(file):
    id = file.read(1)
    if id == b'':
        return None

    packing, item_type = item_types[id]

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


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("path")
    args = parser.parse_args()

    with open(args.path, 'rb') as file:
        for item in read_all(file):
            print(item)


