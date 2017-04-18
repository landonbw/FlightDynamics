import numpy as np
from math import sin, cos, atan, atan2
import sys

sys.path.append('..')
import ParamFiles.AerosondeParameters as P

class PathFollower(object):
    def __init__(self):
        self.chi_inf = P.chi_inf
        self.k_path = P.k_path
        self.k_orbit = P.k_orbit

    def follow_path(self, y_manager, pos, chi, phi):
        follow_line = y_manager[0]
        if follow_line:
            return self.follow_straight_line(y_manager, pos, chi, phi)
        elif not follow_line:
            return self.follow_orbit(y_manager, pos, chi, phi)

    def follow_straight_line(self, y_manager, pos, chi, phi):
        r = y_manager[2]
        q = y_manager[3]
        chi_q = atan2(q.item(1), q.item(0))
        while chi_q - chi < -np.pi:
            chi_q += 2 * np.pi
        while chi_q + chi > np.pi:
            chi_q -= 2 * np.pi
        epy = -sin(chi_q) * (pos.item(0) - r.item(0)) + cos(chi_q) * (pos.item(1) - r.item(1))
        # print(epy)
        chi_c = chi_q - self.chi_inf * (2 / np.pi) * atan(self.k_path * epy)
        h_c = -r.item(2)
        phi_ff = 0
        return h_c, chi_c, phi_ff

    def follow_orbit(self, y_manager, pos, chi, phiff):
        c = y_manager[4]
        cn = c.item(0)
        ce = c.item(1)
        h_c = -c.item(2)
        rho = y_manager[5]
        direction = y_manager[6]
        pn = pos.item(0)
        pe = pos.item(1)

        d = np.sqrt((pn - cn)**2 + (pe - ce)**2)
        phi = atan2(pe - ce, pn - cn)
        # print(d-rho)
        while phi - chi < -np.pi:
            phi += 2 * np.pi
        while phi - chi > np.pi:
            phi -= 2 * np.pi
        chi_c = phi + direction * ((np.pi/2) + atan(self.k_orbit * ((d-rho) / rho)))
        phi_ff = direction * atan(y_manager[1]**2 / P.g / rho)
        return h_c, chi_c, phi_ff



