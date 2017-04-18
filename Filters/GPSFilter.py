import numpy as np
import sys
from math import sin, cos, tan

sys.path.append('..')
import ParamFiles.AerosondeParameters as P

class GPSFilter(object):
    def __init__(self, xhat):
        """
        Initialize the kalman filter
        :param xhat:
        the state of the filter that we're going to track
        """
        self.xhat = xhat

        self.P = np.identity(7) - 0.95
        self.Q = np.array([[0.00030, 0, 0, 0, 0, 0, 0],  # north posistion
                           [0, 0.00030, 0, 0, 0, 0, 0],  # east position
                           [0, 0, 0.00000030, 0, 0, 0, 0],  # ground velocity
                           [0, 0, 0, 0.000000000070, 0, 0, 0],  # course angle
                           [0, 0, 0, 0, 0.00030, 0, 0],   # north wind
                           [0, 0, 0, 0, 0, 0.00030, 0],   # east wind
                           [0, 0, 0, 0, 0, 0, 0.00000000030]]) # yaw
        self.Ri = np.array([[P.gps_std_dev_ne**2], [P.gps_std_dev_ne**2], [P.gps_std_dev_vel**2], [(P.gps_std_dev_vel/P.Va0)**2], [0.3], [0.3], [0.1]])
        # self.count = 0

    def predict(self, ts, u):
        va = u.item(0)
        q = u.item(1)
        r = u.item(2)
        phi = u.item(3)
        theta = u.item(4)


        self.xhat += ts * self.calculate_f(va, q, r, phi, theta)
        A = self.calculate_dfdx(va, q, r, phi, theta)
        self.P = self.P + ts * (np.matmul(A, self.P) + np.matmul(self.P, A.T) + self.Q)
        # return np.zeros((7,1))
        return self.xhat


    def update(self, u, ygps):
        """

        :param u: input values u = (Va, q, r, phi, theta)transpose
        :param ygps: Measured values Ygps = (pn, pe, vg, chi, wn, we)transpose
        :return:
        """
        va = u.item(0)
        # q = u.item(1)
        # r = u.item(2)
        # phi = u.item(3)
        # theta = u.item(4)

        C = self.calculate_dhdx(va)
        h = self.calculate_h(va)

        for i, Ci in enumerate(C):
            Li = np.matmul(self.P, Ci.reshape((-1, 1))) / (self.Ri.item(i) + np.matmul(Ci.reshape((1,-1)), np.matmul(self.P, Ci.reshape((-1,1)))))
            self.P = np.matmul((np.identity(7) - np.matmul(Li, Ci.reshape((1,-1)))), self.P)
            self.xhat = self.xhat + Li * (ygps[i] - h.item(i))
            # self.count += 1
            # if self.count%50 ==0: print(Li)

        # return np.zeros((7, 1))
        return self.xhat

    def calculate_f(self, va, q, r, phi, theta):
        # equation on page 160
        # pn = self.xhat.item(0)
        # pe = self.xhat.item(1)
        vg = self.xhat.item(2)
        #silly line added to prevent crashing while the filter isn't working
        if vg == 0:
            vg += 0.01
        chi = self.xhat.item(3)
        wn = self.xhat.item(4)
        we = self.xhat.item(5)
        psi = self.xhat.item(6)

        psi_dot = q * (sin(phi) / cos(theta)) + r * (cos(phi) / cos(theta))
        vg_dot = ((va * cos(psi) + wn) * (-va * psi_dot * sin(psi)) + (va * sin(psi) + we) * (va * psi_dot * cos(psi)))\
                 / vg
        chi_dot = P.g / vg * tan(phi) * cos(chi - psi)

        f = np.array([[vg * cos(chi)],
                      [vg * sin(chi)],
                      [vg_dot],
                      [chi_dot],
                      [0],
                      [0],
                      [psi_dot]])
        return f

    def calculate_h(self, va):
        # equation on page 161
        pn = self.xhat.item(0)
        pe = self.xhat.item(1)
        vg = self.xhat.item(2)
        chi = self.xhat.item(3)
        wn = self.xhat.item(4)
        we = self.xhat.item(5)
        psi = self.xhat.item(6)

        wn_psuedo = va * cos(psi) + wn - vg * cos(chi)
        we_psuedo = va * sin(psi) + we - vg * sin(chi)
        h = np.array([[pn],
                      [pe],
                      [vg],
                      [chi],
                      [wn_psuedo],
                      [we_psuedo]])
        return h

    def calculate_dfdx(self, va, q, r, phi, theta):
        # equation on page 160
        pn = self.xhat.item(0)
        pe = self.xhat.item(1)
        vg = self.xhat.item(2)
        chi = self.xhat.item(3)
        wn = self.xhat.item(4)
        we = self.xhat.item(5)
        psi = self.xhat.item(6)

        xdot = self.calculate_f(va, q, r, phi, theta)
        vgdot = xdot.item(2)
        psidot = xdot.item(6)

        dvgdpsi = (-psidot * va * (wn * cos(psi) + we * sin(psi))) / vg
        dchidvg = -P.g / vg**2 * tan(psi) * cos(chi - psi)
        dchidchi = -P.g / vg * tan(psi) * sin(chi - psi)
        dchidpsi = P.g / vg * tan(psi) * sin(chi - psi)

        dfdx = np.array([[0, 0, cos(chi), -vg * sin(chi), 0, 0, 0],
                         [0, 0, sin(chi), vg * cos(chi), 0, 0, 0],
                         [0, 0, -vgdot / vg, 0, -psidot * va * sin(psi), psidot * va * cos(psi), dvgdpsi],
                         [0, 0, dchidvg, dchidchi, 0, 0, dchidpsi],
                         [0, 0, 0, 0, 0, 0, 0],
                         [0, 0, 0, 0, 0, 0, 0],
                         [0, 0, 0, 0, 0, 0, 0]])

        return dfdx

    def calculate_dhdx(self, va):
        # equation on page 161
        # pn = self.xhat.item(0)
        # pe = self.xhat.item(1)
        vg = self.xhat.item(2)
        chi = self.xhat.item(3)
        # wn = self.xhat.item(4)
        # we = self.xhat.item(5)
        psi = self.xhat.item(6)

        dhdx = np.array([[1, 0, 0, 0, 0, 0, 0],
                         [0, 1, 0, 0, 0, 0, 0],
                         [0, 0, 1, 0, 0, 0, 0],
                         [0, 0, 0, 1, 0, 0, 0],
                         [0, 0, -cos(chi), vg * sin(chi), 1, 0, -va * sin(psi)],
                         [0, 0, -sin(chi), -vg * cos(chi), 0, 1, va * cos(psi)]])
        return dhdx