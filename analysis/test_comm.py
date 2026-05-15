from collections import namedtuple
import struct
from threading import Thread
import serial
import matplotlib.pyplot as plt
import time
import glob
import random

import parse

State = namedtuple(
    "State",
    (
        "time",
        "acc",
        "acc_noise",
        "hg_acc",
        "hg_acc_noise",
        "gyro",
        "gyro_noise",
        "baro",
        "baro_noise",
    ),
)
Done = namedtuple("Done", ("time"))
Ping = namedtuple("Ping", tuple())
Reboot = namedtuple("Reboot", tuple())


def flatten(item):
    l = []
    for x in item:
        if isinstance(x, tuple):
            l.extend(x)
        else:
            l.append(x)

    return l


send_types = {
    State: ("<Lffffffffffffffffffffff", b"S"),
    Done: ("<L", b"D"),
    Ping: (None, b"P"),
    Reboot: (None, b"R"),
}


# TODO: The plot is not great code and should be fixed
class DataManager:
    def __init__(self):
        self.running = False

    def get_current_data(self):
        if not self.running:
            raise Exception("Manager not started or stopped")

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
                if item[0] is None and isinstance(item[1], bytes):
                    print(item[1].decode(), end="")

                self.data.append(item)
        # This is deeply stupid, but this is the error it gets when the connection is closed
        except TypeError:
            pass

    def _setup_canvas(self):
        count = len(self.types_to_plot)
        self.fig, axes_list = plt.subplots(count, 1, sharex=True, figsize=(9, 9))
        if count == 1:
            axes_list = [axes_list]

        for ax, typ in zip(axes_list, self.types_to_plot):
            self.axes[typ] = ax
            self.lines[typ] = []

            # Create a line for every field in the tuple (e.g., x, y, z)
            for field_name in typ._fields:
                (line,) = ax.plot([], [], label=field_name)
                self.lines[typ].append(line)

            ax.set_title(typ.__name__)
            ax.legend(loc="upper right", fontsize="x-large")

        self.fig.show()
        plt.pause(0.1)

    # TODO: auto update
    def update(self):
        if not self.data:
            self.fig.canvas.flush_events()
            return

        current_snapshot = self.data[:]
        for typ in self.types_to_plot:
            typ_items = [it for it in current_snapshot if isinstance(it[1], typ)]
            if not typ_items:
                continue

            times, data_objects = zip(*typ_items)

            for i, line in enumerate(self.lines[typ]):
                y_values = [obj[i] for obj in data_objects]
                line.set_data(times, y_values)

            self.axes[typ].relim()
            self.axes[typ].autoscale_view()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def find_port(self):
        files = glob.glob('/dev/ttyACM[0-9]*')

        if len(files) == 0:
            return None

        # Whatever man
        return random.choice(files)

    def start(self, id):
        self.types_to_plot = [parse.Acc, parse.Servo, parse.FilterState]
        self.data = []
        self.lines = {}
        self.axes = {}
        plt.ion()
        self._setup_canvas()

        while True:
            try:
                port = self.find_port()
                if port is None:
                    time.sleep(0.1)
                    continue

                self.ser = serial.Serial(port, 115200)

                # Reset board
                self.send(Reboot())
            # We know the board is reset when it kills the serial
            except OSError:
                break

        # Ping to make sure we have rebooted
        # The else and continue make it so the loop
        #  repeats when the inner loop is not broken out of
        while True:
            try:
                port = self.find_port()
                if port is None:
                    time.sleep(0.1)
                    continue

                self.ser = serial.Serial(port, 115200)

                self.send(Ping())

                while self.ser.in_waiting > 0:
                    res = parse.read_item(self.ser)
                    # If it is an ACK then break out
                    if res is not None and isinstance(res[1], bool) and res[1]:
                        break
                else:
                    time.sleep(0.1)
                    continue

                break
            # If the board reboots will this is going we just restart
            except OSError:
                time.sleep(0.1)
                pass

        self.ser.write(b'MTest ID: ' + id.encode('utf-8') + b'\0')

        self.running = True
        Thread(target=self._run).start()

    def stop(self, _1, _2, _3):
        if not self.running:
            return

        self.running = False

        if self.ser.is_open:
            self.ser.close()

        plt.ioff()
        plt.show()
