from collections import namedtuple
import struct
from threading import Thread
import argparse as ap
import serial

import parse

State = namedtuple('State', ('time', 'acc', 'hg_acc', 'gyro', 'baro'))
Done = namedtuple('Done', ('time'))
Ping = namedtuple('Ping', tuple())

def flatten(item):
    l = []
    for x in item:
        if isinstance(x, tuple):
            l.extend(x)
        else:
            l.append(x)

    return l


send_types = {
    State: ('<Lfffffffffff', b'S'),
    Done: ('<L', b'D'),
    Ping: (None, b'P'),
}

class DataManager:
    def __init__(self, port):
        self.port = port
        self.running = False


    def get_current_data(self):
        if not self.running:
            raise Exception('Manager not started. Use a "with" statement.')

        # Shallow copy
        return self.data[:]


    def write(self, item):
        layout, id = send_types[type(item)]

        self.ser.write(id)
        if layout is not None:
            self.ser.write(struct.pack(layout, flatten(item)))


    def _run(self):
        for item in parse.read_iter(self.ser):
            self.data.append(item)


    def __enter__(self):
        self.ser = serial.Serial(self.port, 115200)
        self.data = []
        self.running = True

        Thread(target=self._run).start()
        return self


    def __exit__(self, _1, _2, _3):
        self.running = False

        if self.ser.is_open:
            self.ser.close()


parser = ap.ArgumentParser()
parser.add_argument('port')
parser.add_argument('test')
args = parser.parse_args()

import time

with DataManager(args.port) as dm:
    while True:
        dm.ser.write(b'P')
        time.sleep(1)
        print(dm.get_current_data())
