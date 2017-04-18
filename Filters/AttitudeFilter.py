import numpy as np
import sys
from math import sin, cos, tan

sys.path.append('..')
import ParamFiles.AerosondeParameters as P

class AttitudeFilter(object):
    def __init__(self, xhat):
        """
        Initialize the kalman filter
        :param xhat:
        the state of the filter that we're going to track
        [phihat, thetahat]
        """
        self.xhat = xhat
        # self.P = np.array([[0.000000001, 0],
        #                    [0, 0.000000001]])
        # self.Q = np.array([[0.000000001, 0],
        #                    [0, 0.000000001]])
        self.P = np.array([[0.01, 0.],
                           [0., 0.01]], dtype=np.float64)

        self.Q = np.array([[0.0000000000005, 0.0],
                           [0.0, 0.000000000065]], dtype=np.float64)
        self.Ri = P.accel_std_dev ** 2
        self.count = 0

    def predict(self, ts, sensors):
        # a_x = sensors.item(0)
        # a_y = sensors.item(1)
        # a_z = sensors.item(2)
        p = sensors.item(3)
        q = sensors.item(4)
        r = sensors.item(5)
        # h = sensors.item(6)
        va = sensors.item(7)

        self.xhat += ts * self.calculate_f(p, q, r)
        A = self.calculate_dfdx(q, r)
        self.P = self.P + ts * (np.matmul(A, self.P) + np.matmul(self.P, A.T) + self.Q)
        return self.xhat


    def update(self, ts, sensors):
        a_x = sensors.item(0)
        a_y = sensors.item(1)
        a_z = sensors.item(2)
        Yn = [a_x, a_y, a_z]
        p = sensors.item(3)
        q = sensors.item(4)
        r = sensors.item(5)
        # h = sensors.item(6)
        va = sensors.item(7)

        C = self.calculate_dhdx(p, q, r, va)
        h = self.calculate_h(p, q, r, va)
        for i in range(3):
            Li = np.matmul(self.P, C[i].reshape((-1, 1))) / (self.Ri + np.matmul(C[i].reshape((1,-1)), np.matmul(self.P, C[i].reshape((-1,1)))))
            self.P = np.matmul((np.identity(2) - np.matmul(Li, C[i].reshape((1,-1)))), self.P)
            self.xhat = self.xhat + Li * (Yn[i] - h.item(i))
            self.count += 1
            # if self.count%200 ==0: print(Li)

        return self.xhat

    def calculate_f(self, p, q, r):
        # equation on page 157
        phi = self.xhat.item(0)
        theta = self.xhat.item(1)
        f = np.array([[p + q * sin(phi) * tan(theta) + r * cos(phi) * tan(theta)],
                      [q * cos(phi) - r * sin(phi)]])
        return f

    def calculate_h(self, p, q, r, va):
        # equation on page 157
        phi = self.xhat.item(0)
        theta = self.xhat.item(1)
        h = np.array([[q * va * sin(theta) + P.g * sin(theta)],
                      [r * va * cos(theta) - p * va * sin(theta) - P.g * cos(theta) * sin(phi)],
                      [-q * va * cos(theta) - P.g * cos(theta) * cos(phi)]])
        return h

    def calculate_dfdx(self, q, r):
        # equation on page 158
        phi = self.xhat.item(0)
        theta = self.xhat.item(1)
        dfdx = np.array(
            [[q * cos(phi) * tan(theta) - r * sin(phi) * tan(theta), (q * sin(phi) + r * cos(phi)) / (cos(theta) ** 2)],
             [-q * sin(phi) - r * cos(phi), 0]])
        return dfdx

    def calculate_dhdx(self, p, q, r, va):
        phi = self.xhat.item(0)
        theta = self.xhat.item(1)
        dhdx = np.array([[0, q * va * cos(theta) + P.g * cos(theta)],
                         [-P.g * cos(phi) * cos(theta),
                          -r * va * sin(theta) - p * va * cos(theta) + P.g * sin(phi) * sin(theta)],
                         [P.g * sin(phi) * cos(theta), (q * va + P.g * cos(phi)) * sin(theta)]])
        return dhdx