from collections import namedtuple
import struct
from threading import Thread
import serial
import serial.serialutil
import matplotlib.pyplot as plt

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
}


# TODO: The plot is not great code and should be fixed
class DataManager:
    def __init__(self, port):
        self.port = port
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
                    print(item[1].decode(), end='')

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

        # Snapshot for thread safety and performance
        # Using a sliding window of the last 400 points
        current_snapshot = self.data[:]
        for typ in self.types_to_plot:
            # Filter items of this type: [(time, datum), ...]
            typ_items = [it for it in current_snapshot if isinstance(it[1], typ)]
            if not typ_items:
                continue

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


    def start(self):
        self.types_to_plot = [parse.Acc, parse.Servo, parse.FilterState]
        self.data = []
        self.lines = {}
        self.axes = {}
        plt.ion()
        self._setup_canvas()

        self.ser = serial.Serial(self.port, 115200)

        self.running = True
        Thread(target=self._run).start()


    def stop(self, _1, _2, _3):
        self.send(Done(0))

        if not self.running:
            return

        self.running = False

        if self.ser.is_open:
            self.ser.close()

        plt.ioff()
        plt.show()

