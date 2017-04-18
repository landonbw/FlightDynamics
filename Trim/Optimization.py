import sys
import numpy as np
from numpy import sin, cos, tan, arcsin

sys.path.append('..')
import ParamFiles.AerosondeParameters as P
from Model.Forces import GetCD_Alpha, GetCL_Alpha, GetSigma_Alpha

def ComputeStates(alpha, beta, phi, Va, R, gamma):
    """
    Compute the states of the model given the input parameters
    :param alpha: Angle of attack
    :param beta: Sideslip angle
    :param phi: Pitch angle
    :param Va: Airspeed
    :param R: Turn radius
    :param gamma: path angle
    :return: state of the system
    """
    u = Va * cos(alpha) * cos(beta)
    v = Va * sin(beta)
    w = Va * sin(alpha) * cos(beta)
    theta = alpha + gamma
    p = (-Va / R) * sin(theta)
    q = (Va / R) * sin(phi) * cos(theta)
    r = (Va / R) * cos(phi) * cos(theta)
    return [u, v, w, theta, p, q, r]

def ComputeInputs(alpha, beta, phi, u, v, w, theta, p, q, r, Va, R, gamma):
    """
    Compute the inputs to the system that will produce the specified state
    :param alpha: angle of attack
    :param beta: sideslip angle
    :param phi: pitch
    :param u: x velocity
    :param v: y velocity
    :param w: z velocity
    :param theta: roll angle
    :param p: pitch angular velocity
    :param q: roll angular velocity
    :param r: yaw angular velocity
    :param Va: airspeed
    :param R: turn radius
    :param gamma: path angle
    :return: deltaE, deltaT, deltaA, deltaR
    """
    deltaE = (((P.jxz * (p**2 - r**2) + (P.jx - P.jz) * p * r) / (0.5 * P.rho * Va**2.0 * P.c * P.S)) -
              P.Cm0 - P.CmAlpha * alpha - P.Cmq * ((P.c * q) / (2.0 * Va))) / P.CmDeltae

    #intermediate terms for deltaT
    num1 = 2.0 * P.mass * (-r * v + q * w + P.g * sin(theta)) - \
           P.rho * Va**2 * P.S * (GetCx(alpha) + GetCxq(alpha) * ((P.c * q) / (2.0 * Va)) + GetCxdele(alpha) * deltaE)
    denom1 = P.rho * P.Sprop * P.Cprop * (P.kMotor**2)
    num2 = Va**2
    denom2 = P.kMotor**2
    t1 = num1 / denom1
    t2 = num2 / denom2
    #deltaT equation F.2
    deltaT = np.sqrt(t1 + t2)

    #intermediate terms for deltaA and deltaR
    LHS = np.array([[P.CpDeltaA, P.CpDeltaR],[P.CrDeltaA, P.CrDeltaR]])
    RHS = np.array([[((-P.gamma1 * p * q + P.gamma2 * q * r) / (0.5 * P.rho * Va**2 * P.S * P.b)) -
                      P.Cp0 - P.CpBeta * beta - P.CpP * ((P.b * p) / (2.0 * Va)) - P.CpR * ((P.b * r) / (2.0 * Va))],
                     [((-P.gamma7 * p * q + P.gamma1 * q * r) / (0.5 * P.rho * Va**2 * P.S * P.b)) -
                      P.Cr0 - P.CrBeta * beta - P.CrP * ((P.b * p) / (2.0 * Va)) - P.CrR * ((P.b * r) / (2.0 * Va))]])
    #deltaA/deltaR equation F.3
    # deltaAR = np.linalg.inv(LHS) * RHS
    deltaAR = np.linalg.solve(LHS, RHS)
    check = np.allclose(np.dot(LHS, deltaAR), RHS)
    if not check:
        print('error in Delta A and Delta R calculation')
    deltaA = deltaAR.item(0)
    deltaR = deltaAR.item(1)
    return deltaE, deltaT, deltaA, deltaR

def ComputeJ(alpha, beta, phi, Va, R, gamma):
    """
    Compute the j value that is used in the minimization problem to determine the control surface
    positions for trimmed flight
    :param alpha: Aircraft angle of attack
    :param beta: Sideslip angle
    :param phi: pitch
    :param Va: desired airspeed velocity
    :param R: radius of desired path +R indicates right hand turn -R indicates left hand
    :param gamma: desired path angle
    :return: j
    """
    #   compute xDot (equation 5.21)
    xDot = np.zeros((12,1))
    xDot[2] = Va * sin(gamma)
    xDot[8] = (Va / R) * cos(gamma)

    #   compute trimmed states
    [u, v, w, theta, p, q, r] = ComputeStates(alpha, beta, phi, Va, R, gamma)
    # u = Va * cos(alpha) * cos(beta)
    # v = Va * sin(beta)
    # w = Va * sin(alpha) * cos(beta)
    # theta = alpha + gamma
    # p = (-Va / R) * sin(theta)
    # q = (Va / R) * sin(phi) * cos(theta)
    # r = (Va / R) * cos(phi) * cos(theta)

    #   compute trimmed input (equations F.1-F.3)
    deltaE, deltaT, deltaA, deltaR = ComputeInputs(alpha, beta, phi, u, v, w, theta, p, q, r, Va, R, gamma)

    # #deltaE equation F.1
    # deltaE = (((P.jxz * (p**2 - r**2) + (P.jx - P.jz) * p * r) / (0.5 * P.rho * Va**2 * P.c * P.S)) -
    #           P.Cm0 - P.CmAlpha * alpha - P.Cmq * ((P.c * q) / (2 * Va))) / P.CmDeltae
    #
    # #intermediate terms for deltaT
    # num1 = 2 * P.mass * (-r * v + q * w + P.g * sin(theta)) - \
    #        P.rho * Va**2 * P.S * (GetCx(alpha) + GetCxq(alpha) * ((P.c * q) / (2 * Va)) + GetCxdele(alpha) * deltaE)
    # denom1 = P.rho * P.Sprop * P.Cprop * P.kMotor**2
    # num2 = Va**2
    # denom2 = P.kMotor**2
    # #deltaT equation F.2
    # deltaT = np.sqrt((num1 / denom1 + num2 / denom2))
    #
    # #intermediate terms for deltaA and deltaR
    # CpdeltaA = P.gamma3 * P.Cl0 + P.gamma4 * P.Cn0
    # mat1 = np.array([[P.CpDeltaA, P.CpDeltaR],[P.CrDeltaA, P.CrDeltaR]])
    # mat2 = np.array([[((-P.gamma1 * p * q + P.gamma2 * q * r) / (0.5 * P.rho * Va**2 * P.S * P.b)) -
    #                   P.Cp0 - P.CpBeta * beta - P.CpP * ((P.b * p) / (2.0 * Va)) - P.CpR * ((P.b * r) / (2.0 * Va))],
    #                  [((-P.gamma7 * p * q + P.gamma1 * q * r) / (0.5 * P.rho * Va**2 * P.S * P.b)) -
    #                   P.Cr0 - P.CrBeta * beta - P.CrP * ((P.b * p) / (2.0 * Va)) - P.CrR * ((P.b * r) / (2.0 * Va))]])
    # #deltaA/deltaR equation F.3
    # deltaAR = np.linalg.inv(mat1) * mat2
    # deltaA = deltaAR.item(0)
    # deltaR = deltaAR.item(1)


    #   compute F (equations 5.3-5.12)
    pNDot = 0.0
    pEDot = 0.0
    hDot = u * sin(theta) - v * sin(phi) * cos(theta) - w * cos(phi) * cos(theta)

    uDot = r * v - q * w - P.g * sin(theta) + \
           ((P.rho * Va**2 * P.S) / (2.0 * P.mass)) * \
           (GetCx(alpha) + GetCxq(alpha) * ((P.c * q) / (2.0 * Va)) + GetCxdele(alpha) * deltaE) + \
           ((P.rho * P.Sprop * P.Cprop) / (2.0 * P.mass)) * ((P.kMotor * deltaT)**2 - Va**2)

    vDot = p * w - r * u + P.g * cos(theta) * sin(phi) + \
           ((P.rho * Va**2 * P.S) / (2.0 * P.mass)) * \
           (P.Cy0 + P.CyBeta * beta + P.CyP *
            ((P.b * p) / (2.0 * Va)) + P.CYr * ((P.b * r) / (2.0 * Va)) + P.CyDeltaa * deltaA + P.CyDeltar * deltaR)

    wDot = q * u - p * v + P.g * cos(theta) * cos(phi) + \
           ((P.rho * Va**2 * P.S) / (2.0 * P.mass)) * \
           (GetCz(alpha) + GetCzq(alpha) * ((P.c * q) / (2.0 * Va)) + GetCzdele(alpha) * deltaE)

    phiDot = p + q * sin(phi) * tan(theta) + r * cos(phi) * tan(theta)

    thetaDot = q * cos(phi) - r * sin(phi)

    psiDot = q * sin(phi) * (1 / cos(theta)) + r * cos(phi) * (1 / cos(theta))

    pDot = P.gamma1 * p * q - P.gamma2 * q * r + \
           0.5 * P.rho * Va**2 * P.S * P.b * \
           (P.Cp0 + P.CpBeta * beta + P.CpP * ((P.b * p) / (2.0 * Va)) +
            P.CpR * ((P.b * r) / (2.0 * Va)) + P.CpDeltaA * deltaA + P.CpDeltaR * deltaR)

    qDot = P.gamma5 * p * r - P.gamma6 * (p**2 - r**2) + \
           ((P.rho * Va**2 * P.S * P.c) / (2 * P.jy)) * \
           (P.Cm0 + P.CmAlpha * alpha + P.Cmq * ((P.c * q) / (2.0 * Va)) + P.CmDeltae * deltaE)

    rDot = P.gamma7 * p * q - P.gamma1 * q * r + \
           0.5 * P.rho * Va**2 * P.S * P.b * \
           (P.Cr0 + P.CrBeta * beta + P.CrP * ((P.b * p) / (2.0 * Va)) +
            P.CrR * ((P.b * r) / (2 * Va)) + P.CrDeltaA * deltaA + P.CrDeltaR * deltaR)

    f = np.array([[pNDot],
                  [pEDot],
                  [hDot],
                  [uDot],
                  [vDot],
                  [wDot],
                  [phiDot],
                  [thetaDot],
                  [psiDot],
                  [pDot],
                  [qDot],
                  [rDot]])
    # print(f.T)

    #   compute j
    error = xDot - f
    print(error.T)
    errorNorm = np.linalg.norm(error)
    j = errorNorm**2
    return j

def MinimizeJ(alpha0, beta0, phi0, Va, R, gamma):
    """
    Minimize the value of j using the gradient descent method determining the
    alpha, beta, and phi values to hold the airspeed, turn radius and path angle
    :param alpha0: guess for alpha
    :param beta0: guess for beta
    :param phi0: guess for phi
    :param Va: desired airspeed
    :param R: desired turn radius
    :param gamma: desired path angle
    :return: alpha, beta and phi to hold the desired values: alpha, beta, phi
    """
    J = 1000
    epsilon = 0.01
    roundLimit = 100000
    k = 0.001
    precision = 0.00001
    for i in range(roundLimit):
        if i % 1 == 0:
            print(i)
            print("J:\t\t", J)
            print("alpha:\t", alpha0, "\nbeta:\t", beta0, "\nphi:\t", phi0)
        alphaStep = alpha0 + epsilon
        betaStep = beta0 + epsilon
        phiStep = phi0 + epsilon
        djdalpha = (ComputeJ(alphaStep, beta0, phi0, Va, R, gamma) - ComputeJ(alpha0, beta0, phi0, Va, R, gamma)) / epsilon
        djdbeta = (ComputeJ(alpha0, betaStep, phi0, Va, R, gamma) - ComputeJ(alpha0, beta0, phi0, Va, R, gamma)) / epsilon
        djdphi = (ComputeJ(alpha0, beta0, phiStep, Va, R, gamma) - ComputeJ(alpha0, beta0, phi0, Va, R, gamma)) / epsilon
        alpha0 += -k * djdalpha
        beta0 += -k * djdbeta
        phi0 += -k * djdphi
        J = ComputeJ(alpha0, beta0, phi0, Va, R, gamma)


        if J < precision:
            print('mimimum found after %s rounds' %(i))
            break
    return alpha0, beta0, phi0

def ComputeTrim(airspeed, pathAngle, turnRadius):
    """
    Compute the control surface values that will cause the UAV to fly with the desired values
    :param airspeed: the desired airspeed to hold in m/s
    :param pathAngle: the desired path angle to hold in radians
    :param turnRadius: the desired turn radius to hold in meters
    :return: the required control surface values: deltaE, deltaT, deltaA, deltaR
    """
    alphaGuess = 0.25
    betaGuess = 0.0
    phiGuess = 0.0
    #   Compute alpha, beta, gamma
    alpha, beta, phi = MinimizeJ(alphaGuess, betaGuess, phiGuess, airspeed, turnRadius, pathAngle)
    #   Compute the trimmed states
    u, v, w, theta, p, q, r = ComputeStates(alpha, beta, phi, airspeed, turnRadius, pathAngle)
    #   Compute trimmed input
    deltaE, deltaT, deltaA, deltaR = ComputeInputs(alpha, beta, phi, u, v, w,
                                                   theta, p, q, r, airspeed, turnRadius, pathAngle)
    return deltaE, deltaT, deltaA, deltaR

def GetCx(alpha):
    """
    Compute the Cx value
    :param alpha: angle of attack
    :return: Cx
    """
    sigAlph = GetSigma_Alpha(alpha)
    Cx = -GetCD_Alpha(alpha) * cos(alpha) + GetCL_Alpha(alpha, sigAlph) * sin(alpha)
    return Cx

def GetCxq(alpha):
    """
    Compute the Cxq value
    :param alpha: angle of attack
    :return: Cxq
    """
    Cxq = -P.Cdq * cos(alpha) + P.Clq * sin(alpha)
    return Cxq

def GetCxdele(alpha):
    """
    Compute the CxDeltaE value
    :param alpha: angle of attack
    :return: CxDeltaE
    """
    Cxdele = -P.CdDeltae * cos(alpha) + P.ClDeltae * sin(alpha)
    return Cxdele

def GetCz(alpha):
    """
    Compute Cz
    :param alpha: Angle of attack
    :return: Cz
    """
    Cz = -GetCD_Alpha(alpha) * sin(alpha) - GetCL_Alpha(alpha, GetSigma_Alpha(alpha)) * cos(alpha)
    return Cz

def GetCzq(alpha):
    """
    Compute Czq
    :param alpha: Angle of attack
    :return: Czq
    """
    Czq = -P.Cdq * sin(alpha) - P.Clq * cos(alpha)
    return Czq

def GetCzdele(alpha):
    """
    Compute CzDeltaE
    :param alpha: Angle of attack
    :return: CzDeltaE
    """
    Czdele = -P.CdDeltae * sin(alpha) - P.ClDeltae * cos(alpha)
    return Czdele

if __name__ == "__main__":
    aSpeed = P.Va0
    Radius = np.inf
    Angle = np.radians(0)
    dE, dT, dA, dR = ComputeTrim(aSpeed, Angle, Radius)
    print("elevator angle:\t", dE, "\nThrotle Value\t:", dT, "\nAileron Angle:\t", dA, "\nRudder Angle:\t", dR)