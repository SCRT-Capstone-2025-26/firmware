from collections import namedtuple
from threading import Thread

from parse import read_iter

State = namedtuple('State', ('time', 'acc', 'hg_acc', 'gyro', 'baro'))
Done = namedtuple('Done', ('time'))

class DataManager():
    def __init__(self, path):
        self.path = path
        self.running = False


    def get_current_data(self):
        if not self.running:
            raise Exception("Must have started the manager. Please use a with statement")

        # Shallow copy
        return self.data[:]


    def _run(self):
        for item in self.data_iter:
            self.data.append(item)


    def __enter__(self):
        self.file = open(self.path, 'rb')
        self.data_iter = read_iter(self.file)
        self.data = []

        Thread(target=self._run).start()
        self.running = True


    def __exit__(self):
        # Should kill the thread by raising an exception
        self.file.close()



