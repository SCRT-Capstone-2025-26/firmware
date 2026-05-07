from collections import namedtuple
import struct
from threading import Thread
import argparse as ap
import serial
import sys
import importlib.util
import pathlib
import time

import parse

State = namedtuple('State', ('time', 'acc', 'acc_noise', 'hg_acc', 'hg_acc_noise', 'gyro', 'gyro_noise', 'baro', 'baro_noise'))
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
    State: ('<Lffffffffffffffffffffff', b'S'),
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


    def send(self, item):
        layout, id = send_types[type(item)]

        self.ser.write(id)
        if layout is not None:
            self.ser.write(struct.pack(layout, *flatten(item)))


    def _run(self):
        try:
            for item in parse.read_iter(self.ser):
                print(item)
                if item[0] is None:
                    print(item[1])

                self.data.append(item)
        except serial.serialutil.SerialException:
            self.running = False


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

# See https://stackoverflow.com/questions/79852343/how-can-i-dynamically-load-and-execute-foo-py-if-it-contains-relative-imports
# I don't want to force the scripts to be rewritten so I do sys path
# It would probably be more correct to pop some stuff from sys path and then re add it
test_path = pathlib.Path(args.test)
sys.path.append(str(test_path.parent))
spec = importlib.util.spec_from_file_location('test_gen', test_path)
test_gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(test_gen)
sys.path.pop()

with DataManager(args.port) as dm:
    dm.ser.write(b'Test 1\0')

    test_gen.run_test(dm)

    while dm.running:
        time.sleep(1)

