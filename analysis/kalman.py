import numpy as np

class KalmanFilter():
    def __init__(self, h_noise, v_noise, hv_noise, step_rate=960, h=0, v=0, h_var=None, v_var=None, hv_cov=None):
        if h_var is None:
            h_var = h_noise * step_rate

        if v_var is None:
            v_var = v_noise * step_rate

        if hv_cov is None:
            hv_cov = hv_noise * step_rate

        self.step_rate = step_rate

        self.state = np.array([[h], [v]])
        self.cov = np.array([[h_var, hv_cov], [hv_cov, v_var]])

        self.obser = np.array([[1, 0]])
        self.control = np.array([[0], [1 / step_rate]])

        self.trans = np.identity(2)
        self.trans_noise = np.array([[h_noise, hv_noise], [hv_noise, v_noise]])


    def step(self, acc, acc_noise, theta):
        cos_theta = np.cos(theta)

        self.trans[0, 1] = cos_theta / self.step_rate
        self.control[0, 0] = 0.5 * cos_theta / self.step_rate / self.step_rate

        self.state = (self.trans @ self.state) + (self.control * acc)
        self.cov = (self.trans @ self.cov @ self.trans.transpose()) + (self.control * acc_noise @ self.control.transpose()) + self.trans_noise


    def sample(self, h, noise):
        inno = h - (self.obser @ self.state)
        inno_noise = (self.obser @ self.cov @ self.obser.transpose()) + noise

        # FYI lol https://math.stackexchange.com/questions/2336473/what-is-the-inverse-of-a-1-times-1-matrix
        gain = self.cov @ self.obser.transpose() / inno_noise

        self.state += gain * inno
        self.cov = (np.identity(2) - (gain @ self.obser)) @ self.cov



class FilterSim():
    def __init__(self, filter):
        self.filter = filter

        self.step_rate = filter.step_rate

        self.states = [filter.state]
        self.covs = [filter.cov]
        self.time = 0


    def step(self, *args, **kwargs):
        self.filter.step(*args, **kwargs)

        self.states.append(self.filter.state)
        self.covs.append(self.filter.state)
        self.time += 1 / self.step_rate


    def sample(self, *args, **kwargs):
        self.filter.sample(*args, **kwargs)

