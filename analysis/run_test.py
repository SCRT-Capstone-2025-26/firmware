from collections import namedtuple
import struct
from threading import Thread
import argparse as ap
import serial
import sys
import importlib.util
import pathlib
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

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

# TODO: The plot is not great code and should be fixed
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
                if item[0] is None:
                    print(item[1])

                self.data.append(item)
        except serial.serialutil.SerialException:
            # TODO: Fix
            self.running = False


    def _setup_canvas(self):
        count = len(self.types_to_plot)
        self.fig, axes_list = plt.subplots(count, 1, sharex=True)
        if count == 1: axes_list = [axes_list]

        for ax, typ in zip(axes_list, self.types_to_plot):
            self.axes[typ] = ax
            self.lines[typ] = []
            
            # Create a line for every field in the tuple (e.g., x, y, z)
            for field_name in typ._fields:
                line, = ax.plot([], [], label=field_name)
                self.lines[typ].append(line)
            
            ax.set_title(typ.__name__)
            ax.legend(loc='upper right', fontsize='x-small')

        self.fig.show()
        plt.pause(0.1)

    def update(self):
        if not self.data:
            self.fig.canvas.flush_events()
            return

        # Snapshot for thread safety and performance
        # Using a sliding window of the last 400 points
        current_snapshot = self.data[-400:] 

        for typ in self.types_to_plot:
            # Filter items of this type: [(time, datum), ...]
            typ_items = [it for it in current_snapshot if isinstance(it[1], typ)]
            if not typ_items: continue

            times, data_objects = zip(*typ_items)
            
            # Update each line (field) in this plot
            for i, line in enumerate(self.lines[typ]):
                # Extract the i-th value from the data tuple
                y_values = [obj[i] for obj in data_objects]
                line.set_data(times, y_values)
            
            self.axes[typ].relim()
            self.axes[typ].autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


    def __enter__(self):
        self.types_to_plot = [parse.Acc]
        self.running = False
        self.data = []
        self.lines = {}
        self.axes = {}
        plt.ion()
        self._setup_canvas()

        self.ser = serial.Serial(self.port, 115200)
        self.data = []
        self.running = True

        Thread(target=self._run).start()
        return self


    def __exit__(self, _1, _2, _3):
        plt.ioff()

        if not self.running:
            return

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

# TODO: Fix noise
with DataManager(args.port) as dm:
    dm.ser.write(b'Test 1\0')

    test_gen.run_test(dm)

    while dm.running:
        dm.update()
        time.sleep(0.01)

