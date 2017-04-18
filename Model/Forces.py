import sys
import numpy as np

sys.path.append('..')
import ParamFiles.AerosondeParameters as P


class Forces:
    def __init__(self):
        pass

    def getForces(self, state, Va, alpha, beta, controlState):
        """
        return the forces on the UAV based on the state, wind, and control surfaces
        :param state: np.matrix(Pn(0), Pe(1), Pd(2), u(3), v(4), w(5), phi(6), theta(7), psi(8), p(9), q(10), r(11))
        :param Va: Airspeed
        :param alpha: Attack angle
        :param beta: Sideslip angle
        :param controlState: np.matrix(Delta A(0), Delta E(1), Delta R(2), Delta T(3))
        :return: Forces on the UAV np.matrix(Fx, Fy, Fz, Ml, Mn, Mm)
        """
        Pn = state.item(0)
        Pe = state.item(1)
        Pd = state.item(2)
        u = state.item(3)
        v = state.item(4)
        w = state.item(5)
        phi = state.item(6)
        theta = state.item(7)
        psi = state.item(8)
        p = state.item(9)
        q = state.item(10)
        r = state.item(11)

        deltaA = controlState.item(0)
        deltaE = controlState.item(1)
        deltaR = controlState.item(2)
        deltaT = controlState.item(3)

        sigAlpha = GetSigma_Alpha(alpha)
        clAlpha = GetCL_Alpha(alpha, sigAlpha)
        cdAlpha = GetCD_Alpha(alpha)
        cX =        -cdAlpha    * np.cos(alpha) + clAlpha    * np.sin(alpha)
        cXq =       -P.Cdq      * np.cos(alpha) + P.Clq      * np.sin(alpha)
        cXdelE =    -P.CdDeltae * np.cos(alpha) + P.ClDeltae * np.sin(alpha)
        cZ =        -cdAlpha    * np.sin(alpha) - clAlpha    * np.cos(alpha)
        cZq =       -P.Cdq      * np.sin(alpha) - P.Clq      * np.cos(alpha)
        cZdelE =    -P.CdDeltae * np.sin(alpha) - P.ClDeltae * np.cos(alpha)
        #term 1 of forces equation
        t1 = np.matrix([[-P.mass * P.g * np.sin(theta)],
                        [P.mass * P.g * np.cos(theta) * np.sin(phi)],
                        [P.mass * P.g * np.cos(theta) * np.cos(phi)]])
        #term 2 of forces equation
        m2 = np.matrix([[cX + cXq * (P.c / (2.0 * Va)) * q + cXdelE * deltaE],
                        [P.Cy0 + P.CyBeta * beta + P.CyP * (P.b / (2.0 * Va)) * p + P.CYr * (P.b / (2.0 * Va)) * r + P.CyDeltaa * deltaA + P.CyDeltar * deltaR],
                        [cZ + cZq * (P.c / (2.0 * Va)) * q + cZdelE * deltaE]])

        t2 = np.multiply(0.5 * P.rho * Va**2 * P.S, m2)
        #term 3 of forces equation
        m3 = np.matrix([[(P.kMotor * deltaT)**2 - Va**2],
                        [0],
                        [0]])
        t3 = np.multiply(0.5 * P.rho * P.Sprop * P.Cprop, m3)
        forces = t1 + t2 + t3   #calculate forces on the body
        #term 1 of moments equation
        m4 = np.matrix([[P.b * (P.Cl0 + P.ClBeta * beta + P.ClP * (P.b / (2 * Va)) * p + P.Clr * (P.b / (2 * Va)) * r + P.ClDeltaa * deltaA + P.ClDeltar * deltaR)],
                        [P.c * (P.Cm0 + P.CmAlpha * alpha + P.Cmq * (P.c / (2 * Va)) * q + P.CmDeltae * deltaE)],
                        [P.b * (P.Cn0 + P.CnBeta * beta + P.CnP * (P.b / (2 * Va)) * p + P.Cnr * (P.b / (2 * Va)) * r + P.CnDeltaa * deltaA + P.CnDeltar * deltaR)]])
        t4 = np.multiply(0.5 * P.rho * Va**2 * P.S, m4)
        #term 2 of moments equation
        t5 = np.matrix([[-P.kTp * (P.kOmega * deltaT)**2],
                        [0],
                        [0]])
        moments = t4 + t5   #calculate moments on the body
        return np.vstack((forces, moments))

def GetCL_Alpha(alpha, sigAlph):
    # print('alpha:',alpha)
    # print('sign(alpha):',np.sign(alpha))
    val = (1 - sigAlph) * (P.CL0 + P.ClAlpha * alpha) + sigAlph * (np.sign(alpha) * (np.sin(alpha) ** 2) * np.cos(alpha))
    return val

def GetCD_Alpha(alpha):
    val = P.Cdp + (((P.CL0 + P.ClAlpha * alpha)**2) / (np.pi * P.e * P.AR))
    return val

def GetSigma_Alpha(alpha):
    num = 1 + np.exp(-P.M * (alpha - P.alpha0)) + np.exp(P.M * (alpha + P.alpha0))
    denom = (1 + np.exp(-P.M * (alpha - P.alpha0))) * (1 + np.exp(P.M * (alpha + P.alpha0)))
    return num/denom

