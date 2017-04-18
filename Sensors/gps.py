import sys
import numpy as np
import numpy.random as rand

sys.path.append('..')
import ParamFiles.AerosondeParameters as P
from Sensors.sensorabstract import sensorAbstract


class GpsChannel(sensorAbstract):
    def __init__(self, bias, std_dev):
        super(GpsChannel, self).__init__(gain=1.0, bias=bias, std_dev=std_dev)

        self.nu = 0.0
        self.k_gps = P.gps_k
        self.Ts = P.gps_sample_time

    def sensor_value(self, state):
        pos = state
        nu = np.exp(-self.k_gps * self.Ts) * self.nu + rand.normal(loc=0.0, scale=self.std_dev)
        self.nu = nu
        gps_val = pos + self.nu
        return gps_val


class Gps(sensorAbstract):
    def __init__(self):
        super(Gps, self).__init__()
        self.north_channel = GpsChannel(P.gps_bias_ne, P.gps_std_dev_ne)
        self.east_channel = GpsChannel(P.gps_bias_ne, P.gps_std_dev_ne)
        self.h_channel = GpsChannel(P.gps_bias_alt, P.gps_std_dev_alt)

    def sensor_value(self, state):
        [states, wind, va] = state
        # self.state = np.matrix([[P.Pn0],  # (0)
        #                         [P.Pe0],  # (1)
        #                         [P.Pd0],  # (2)
        #                         [P.u0],  # (3)
        #                         [P.v0],  # (4)
        #                         [P.w0],  # (5)
        #                         [P.phi],  # (6)
        #                         [P.theta],  # (7)
        #                         [P.psi],  # (8)
        #                         [P.p0],  # (9)
        #                         [P.q0],  # (10)
        #                         [P.r0]])  # (11)
        pn = states.item(0)
        pe = states.item(1)
        pd = states.item(2)
        psi = states.item(8)
        wn = wind.item(0)
        we = wind.item(1)
        va_mag = np.linalg.norm(va)

        gps_n = self.north_channel.sensor_value(pn)
        gps_e = self.east_channel.sensor_value(pe)
        gps_h = self.h_channel.sensor_value(-pd)

        vn = va_mag * np.cos(psi) + wn
        ve = va_mag * np.sin(psi) + we

        # std_dev_vg = (vn**2 * P.gps_std_dev_vel**2 + ve**2 * P.gps_std_dev_vel**2) / (vn**2 + ve**2)
        # std_dev_course = (vn**2 * P.gps_std_dev_vel**2 + ve**2 * P.gps_std_dev_vel**2) / (vn**2 + ve**2)**2
        std_dev_vg = P.gps_std_dev_vel
        std_dev_course = P.gps_std_dev_vel / np.sqrt(vn**2 + ve**2)

        gps_vg = np.sqrt((va_mag * np.cos(psi) + wn)**2 +
                         (va_mag * np.sin(psi) + we)**2) + rand.normal(scale=std_dev_vg)
        gps_course = np.arctan2(ve, vn) + rand.normal(scale=std_dev_course, loc=0.0)
        # print(psi, vn, ve, gps_course)
        # gps_loc = [gps_n, gps_e, gps_h]
        # gps_head = [gps_vg, gps_course]

        return np.array([[gps_n], [gps_e], [gps_h], [gps_course], [gps_vg]])

