"""
Class to determine wind velocity at any given moment,
calculates a steady wind speed and uses a stochastic
process to represent wind gusts and other atmospheric
disturbances. (Follows section 4.4 in uav book)
"""
import sys
import numpy as np

sys.path.append('..')
import ParamFiles.AerosondeParameters as P

class WindSimulation:
    def __init__(self):
        self.steadyWind = P.vWindInertial
        self.Va = P.Va0
        self.wind = P.vWindInertial
        self.alpha = P.attackAng
        self.beta = P.sideSlipAng

    def calculateGust(self):
        """
        Calculates the gusting wind speed using stochastic processes
        :return: the gusting windspeed in a numpy matrix (u, v, w) relative to the body frame
        """
        s = np.random.rand()
        uWg = P.sigmaU * np.sqrt((2 * P.Va0) / P.Lu) * (1 / (s + P.Va0 / P.Lu))
        vWg = P.sigmav * np.sqrt((3 * P.Va0) / P.Lv) * ((s + (P.Va0 / (np.sqrt(3) * P.Lv))) / (s + P.Va0 / P.Lv) ** 2)
        wWg = P.sigmaW * np.sqrt((3 * P.Va0) / P.Lw) * ((s + (P.Va0 / (np.sqrt(3) * P.Lw))) / (s + P.Va0 / P.Lw) ** 2)
        gust = np.matrix([[uWg],
                          [vWg],
                          [wWg]])
        return gust

    def getWind(self, state):
        """
        Calculates the gust speed, adds it to the steady speed
        :param state: np.matrix(Pn(0), Pe(1), Pd(2), u(3), v(4), w(5), phi(6), theta(7), psi(8), p(9), q(10), r(11))
        :return: velocity of the
        """
        # Pn = state.item(0)
        # Pe = state.item(1)
        # Pd = state.item(2)
        # u = state.item(3)
        # v = state.item(4)
        # w = state.item(5)
        phi = state.item(6)
        theta = state.item(7)
        psi = state.item(8)
        # p = state.item(9)
        # q = state.item(10)
        # r = state.item(11)

        cPhi = np.cos(phi)
        sPhi = np.sin(phi)
        cTheta = np.cos(theta)
        sTheta = np.sin(theta)
        cPsi = np.cos(psi)
        sPsi = np.sin(psi)
        rotMat = np.array([[cTheta * cPsi, cTheta * sPsi, -sTheta],
                           [sPhi * sTheta * cPsi - cPhi * sPsi, sPhi * sTheta * sPsi + cPhi * cPsi, sPhi * cTheta],
                           [cPhi * sTheta * cPsi + sPhi * sPsi, cPhi * sTheta * sPsi - sPhi * cPsi, cPhi * cTheta]])

        steadyWind = rotMat.dot(self.steadyWind)
        gust = self.calculateGust()
        self.wind = np.add(steadyWind, gust)
        return self.wind