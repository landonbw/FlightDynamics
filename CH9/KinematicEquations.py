import numpy as np
import sys
from math import sin, cos

sys.path.append('..')
import ParamFiles.AerosondeParameters as P

class KinematicEquations(object):
    def __init__(self):
        self.state = np.array([[P.Pn0],
                               [P.Pe0],
                               [0],
                               [0],
                               [10],
                               [0],
                               [P.Va0]])
        self.bx_dot = P.bx_dot
        self.bx = P.bx
        self.bh_dot = P.bh_dot
        self.bh = P.bh
        self.bva = P.bva
        self.chi_dot = 0.0
        self.chi_d1 = 0.0
        self.chi_com_dot = 0.0
        self.chi_com_d1 = 0.0
        self.h_dot = 0.0
        self.h_d1 = 10.0
        self.h_com_dot = 0.0
        self.h_com_d1 = 10.0

        self.a = (2.0 * P.sigma - P.ts_control) / (2.0 * P.sigma + P.ts_control)
        self.a2 = 2.0 / (2.0 * P.sigma + P.ts_control)
        self.d_gain = 0.01

    # self.chi_dot = self.a * self.chi_dot + self.a2 * (self.chi_d1 - chi)
    # def propagate_dynamics(self, va, va_com, psi, chi, chi_com, h, h_com, wn, we):
    def propagate_dynamics(self, state, u):
        """

        :param state: the state of the system (pn, pe, chi, h, va)
        :param u: the inputs to the system(psi, chi_com, h_com, va_com, wn, we)
        :return: the propagated state of the system
        """
        pn = self.state.item(0)
        pe = self.state.item(1)
        chi = self.state.item(2)
        h = self.state.item(3)
        va = self.state.item(4)
        # psi = u.item(0)
        chi_com = u.item(1)
        h_com = u.item(2)
        # va_com = u.item(3)
        # wn = u.item(4)
        # we = u.item(5)
        # self.chi_dot = self.a * self.chi_dot + self.a2 * (self.chi_d1 - chi)
        self.chi_com_d1 = self.a * self.chi_com_d1 + self.a2 * (self.chi_com_d1 - chi_com)
        # self.h_dot = self.a * self.h_dot + self.a2 * (self.h_d1 - h)
        self.h_com_d1 = self.a * self.h_com_d1 + self.a2 * (self.h_com_d1 - h_com)
        # print(self.chi_com_dot, self.h_com_dot)

        # state_with_dot = np.array([[pn], [pe], [chi], [self.chi_dot], [h], [self.h_dot], [va]])
        state_with_dot = self.state

        k1 = self.derivatives(state_with_dot, u)
        k2 = self.derivatives(state_with_dot + P.ts_simulation / P.num_steps / 2 * k1, u)
        k3 = self.derivatives(state_with_dot + P.ts_simulation / P.num_steps / 2 * k2, u)
        k4 = self.derivatives(state_with_dot + P.ts_simulation / P.num_steps * k3, u)
        newState = state_with_dot + P.ts_simulation / P.num_steps / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        self.state = newState
        return np.array([[newState.item(0)],
                         [newState.item(1)],
                         [newState.item(2)],
                         [newState.item(4)],
                         [newState.item(6)]])


    def derivatives(self, state, u):
        chi = state.item(2)
        chi_dot = state.item(3)
        h = state.item(4)
        h_dot = state.item(5)
        va = state.item(6)
        psi = state.item(2)
        chi_com = u.item(1)
        h_com = u.item(2)
        va_com = u.item(3)
        wn = u.item(4)
        we = u.item(5)

        pn_dot = va * cos(psi) + wn
        pe_dot = va * sin(psi) + we
        x1 = self.state.item(3)
        x1_dot = self.bx_dot * (self.chi_com_dot - chi_dot) + self.bx * (chi_com - chi)
        h1 = self.state.item(5)
        h1_dot = self.bh_dot * (self.h_com_dot - h_dot) + self.bh * (h_com - h)
        va_dot = self.bva * (va_com - va)

        return np.array([[pn_dot],
                         [pe_dot],
                         [x1],
                         [x1_dot],
                         [h1],
                         [h1_dot],
                         [va_dot]])
