"""
This is the second try at creating the trim function.
funny story:
the first one worked pretty darn poorly...
I can't figure out why...
I've spent too much time trying to figure out why...
I'm starting over
"""
import sys
import numpy as np
from numpy import sin, cos

sys.path.append('..')
import ParamFiles.AerosondeParameters as P
from Model.Forces import Forces
from Model.Dynamics import Dynamics
force = Forces()
dynam = Dynamics()

"""
Calculate the Aerodynamic force coefficients in the
X and Z directions see pages 62-63 for reference
"""
def sigAlpha(alpha):
    c1 = np.exp(-P.M * (alpha - P.alpha0))
    c2 = np.exp(P.M * (alpha + P.alpha0))
    sAlpha = (1 + c1 + c2) /((1 + c1) * (1 + c2))
    # num = 1 + np.exp(-P.M * (alpha - P.alpha0)) + np.exp(P.M * (alpha + P.alpha0))
    # denom = (1 + np.exp(-P.M * (alpha - P.alpha0))) * (1 + np.exp(P.M * (alpha + P.alpha0)))
    # sAlpha = num / denom
    return sAlpha

def ClAlpha(alpha):
    sAlpha = sigAlpha(alpha)
    Cl = (1. - sAlpha) * (P.CL0 + P.ClAlpha * alpha) + sAlpha * 2.0 * np.sign(alpha) * sin(alpha)* sin(alpha) * cos(alpha)
    return Cl

def CdAlpha(alpha):
    num = (P.CL0 + P.ClAlpha * alpha)**2
    denom = np.pi * P.e * P.AR
    Cd = P.Cdp + num / denom
    return Cd

def CxAlpha(alpha):
    cx = -CdAlpha(alpha) * cos(alpha) + ClAlpha(alpha) * sin(alpha)
    return cx

def Cxq(alpha):
    cxq = -P.Cdq * cos(alpha) + P.Clq * sin(alpha)
    return cxq

def CxDelE(alpha):
    cx = -P.CdDeltae * cos(alpha) + P.ClDeltae * sin(alpha)
    return cx

def Cz(alpha):
    cz = -CdAlpha(alpha) * sin(alpha) - ClAlpha(alpha) * cos(alpha)
    return cz

def Czq(alpha):
    czq = -P.Cdq * sin(alpha) - P.Clq * cos(alpha)
    return czq

def CzDelE(alpha):
    cz = -P.CdDeltae * sin(alpha) - P.ClDeltae * cos(alpha)
    return cz

"""
Compute the trimmed states of the uav defined in
algorithm 13 (pg 281) and algorithm 14 (pg 283)
"""
def computeTrimStates(inputs, planeValues):
    """
    Compute the trimmed states of the UAV based on the designated
    values, and the plane values
    :param inputs: desired trim positions [Va, gamma, R]
    :param planeValues: plane angular values [alpha, beta, phi]
    :return: trimmed States of the plane [u, v, w, theta, phi, psi, p, q, r]
    """
    Va = inputs[0]
    gamma = inputs[1]
    R = inputs[2]

    alpha = planeValues[0]
    beta = planeValues[1]
    phi = planeValues[2]


    x = np.zeros((12,), dtype=np.double)
    x[3] = Va * np.cos(alpha) * np.cos(beta)
    x[4] = Va * np.sin(beta)
    x[5] = Va * np.sin(alpha) * np.cos(beta)
    x[6] = phi
    theta = alpha + gamma
    x[7] = theta
    x[9] = -Va / R * np.sin(theta)
    x[10] = Va / R * np.sin(phi) * np.cos(theta)
    x[11] = Va / R * np.cos(phi) * np.cos(theta)
    return x

    # this is just a place holder, I don't care about it,
    # but it makes the state vector complete and I like that
    # because it gives me the warm fuzzies inside and it lets
    # it use the dynamics code which I like
    # psi = 0.0
    #
    # u = Va * cos(alpha) * cos(beta)
    # v = Va * sin(beta)
    # w = Va * sin(alpha) * cos(beta)
    # theta = alpha + gamma
    # p = (-Va / R) * sin(theta)
    # q = (Va / R) * sin(phi) * cos(theta)
    # r = (Va / R) * cos(phi) * cos(theta)
    #
    #
    # trimmed_state = [0.0, 0.0, 0.0, u, v, w, phi, theta, psi, p, q, r]
    # trimmed_state_matrix = np.asarray(trimmed_state).T
    # return trimmed_state_matrix

"""
Compute the trimmed input base on the Trimmed output
eqn's F.1, F.2, F.3
"""
def deltaE(trimmedState, inputs, planeValues):
    """
    Compute the elevator position that is defined by the function inputs
    :param trimmedState: the trimmed state of the UAV [u, v, w, phi, theta, psi, p, q, r]
    :param inputs: the desired trimmed inputs [Va, gamma, R]
    :param planeValues: the angles of the plane [alpha, beta, phi]
    :return: the elevator angle (delta E)
    """
    p = trimmedState.item(9)
    q = trimmedState.item(10)
    r = trimmedState.item(11)

    Va = inputs[0]

    alpha = planeValues[0]

    Jxz = P.jxz
    Jx = P.jx
    Jz = P.jz
    C0 = 0.5 * P.rho * Va ** 2 * P.S
    C1 = (Jxz * (p ** 2 - r ** 2) + (Jx - Jz) * p * r) / (C0 * P.c)
    Cm0 = P.Cm0
    Cm_alpha = P.CmAlpha
    Cm_q = P.Cmq
    Cm_delta_e = P.CmDeltae
    return (C1 - Cm0 - Cm_alpha * alpha - Cm_q * P.c * q * 0.5 / Va) / Cm_delta_e

    # numT1_num = P.jxz * ((p**2) - (r**2)) + (P.jx - P.jz) * p * r
    # numT1_denom = 0.5 * P.rho * (Va**2) * P.c * P.S
    # numT1 = numT1_num / numT1_denom
    # # numT2 = -P.Cm0 - P.CmAlpha * alpha - P.Cmq * ((P.c * q) / (2.0 * Va))
    # num = numT1 -P.Cm0 - P.CmAlpha * alpha - P.Cmq * ((P.c * q) / (2.0 * Va))
    # denom = P.CmDeltae
    # delE = num / denom
    # return delE

def deltaT(trimmedState, inputs, planeValues, delE):
    """
    Calculate the throttle postion that is defined by the function inputs
    :param trimmedState: the trimmed state of the UAV [u, v, w, phi, theta, psi, p, q, r]
    :param inputs: the desired trimmed inputs [Va, gamma, R]
    :param planeValues: the angles of the plane [alpha, beta, phi]
    :param deltaE: the elevator angle
    :return: the throttle position (delta T)
    """
    v = trimmedState.item(4)
    w = trimmedState.item(5)
    theta = trimmedState.item(7)
    q = trimmedState.item(10)
    r = trimmedState.item(11)

    Va = inputs[0]

    alpha = planeValues[0]

    # C0 = 0.5 * P.rho * Va ** 2 * P.S
    # CL0 = P.Cl0
    # CL_alpha = P.ClAlpha
    # M = P.M
    # alpha_0 = P.alpha0
    # CD_alpha = P.CdAlpha
    # CD_p = P.Cdp
    # CD_q = P.Cdq
    # CL_q = P.Clq
    # CL_delta_e = P.ClDeltae
    # CD_delta_e = P.CdDeltae
    # C_prop = P.Cprop
    # c1 = np.exp(-M * (alpha - alpha_0))
    # c2 = np.exp(M * (alpha + alpha_0))
    # sigmoid_alpha = (1 + c1 + c2) / ((1 + c1) * (1 + c2))
    # CL_alpha_NL = (1. - sigmoid_alpha) * (CL0 + CL_alpha * alpha) + sigmoid_alpha * 2. * np.sign(alpha) * np.sin(
    #     alpha) * np.sin(alpha) * np.cos(alpha)
    # AR = P.b ** 2 / P.S
    # CD_alpha_NL = CD_p + (CL0 + CL_alpha * alpha) ** 2 / (np.pi * P.e * AR)
    # CX = -CD_alpha_NL * np.cos(alpha) + CL_alpha_NL * np.sin(alpha)
    # CX_delta_e = -CD_delta_e * np.cos(alpha) + CL_delta_e * np.sin(alpha)
    # CX_q = -CD_q * np.cos(alpha) + CL_q * np.sin(alpha)
    # C2 = 2 * P.mass * (-r * v + q * w + P.g * np.sin(theta))
    # C3 = -2 * C0 * (CX + CX_q * P.c * q * 0.5 / Va + CX_delta_e * delE)
    # C4 = P.rho * C_prop * P.Sprop * P.kMotor ** 2
    # return np.sqrt((C2 + C3) / C4 + Va ** 2 / P.kMotor ** 2)

    num1T1 = 2 * P.mass * (-r * v + q * w + P.g * sin(theta))
    num1T2_1 = -P.rho * (Va**2) * P.S
    num1T2_2 = CxAlpha(alpha) + Cxq(alpha) * ((P.c * q) / (2.0 * Va)) + CxDelE(alpha) * delE
    num1T2 = num1T2_1 * num1T2_2
    num1 = num1T1 + num1T2
    denom1 = P.rho * P.Sprop * P.Cprop * (P.kMotor**2)
    T1 = num1 / denom1

    num2 = Va**2
    denom2 = P.kMotor**2
    T2 = num2 / denom2

    squared = T1 + T2

    delT = np.sqrt(squared)
    return delT

def deltaAR(trimmedState, inputs, planeValues):
    """
    Compute the aileron and rudder control positions that are defined by the function inputs
    :param trimmedState: the trimmed state of the UAV [u, v, w, phi, thata, psi, p, q, r]
    :param inputs: the desired trimmed inputs [Va, gamma, R]
    :param planeValues: the angles of the plane [alpha, betta, gamma]
    :return: the aileron and rudder control positions deltaA, deltaR
    """
    p = trimmedState.item(9)
    q = trimmedState.item(10)
    r = trimmedState.item(11)

    Va = inputs[0]
    beta = planeValues[1]

    # C0 = 0.5 * P.rho * Va ** 2 * P.S
    # Cl_delta_a = P.ClDeltaa
    # Cn_delta_a = P.CnDeltaa
    # Cl_delta_r = P.ClDeltar
    # Cn_delta_r = P.CnDeltar
    # Cl0 = P.Cl0
    # Cn0 = P.Cn0
    # Cl_p = P.ClP
    # Cn_p = P.CnP
    # Cl_beta = P.ClBeta
    # Cn_beta = P.CnBeta
    # Cl_r = P.Clr
    # Cn_r = P.Cnr
    # Cp_delta_a = P.gamma3 * Cl_delta_a + P.gamma4 * Cn_delta_a
    # Cp_delta_r = P.gamma3 * Cl_delta_r + P.gamma4 * Cn_delta_r
    # Cr_delta_a = P.gamma4 * Cl_delta_a + P.gamma8 * Cn_delta_a
    # Cr_delta_r = P.gamma4 * Cl_delta_r + P.gamma8 * Cn_delta_r
    # Cp_0 = P.gamma3 * Cl0 + P.gamma4 * Cn0
    # Cp_beta = P.gamma3 * Cl_beta + P.gamma4 * Cn_beta
    # Cp_p = P.gamma3 * Cl_p + P.gamma4 * Cn_p
    # Cp_r = P.gamma3 * Cl_r + P.gamma4 * Cn_r
    # Cr_0 = P.gamma4 * Cl0 + P.gamma8 * Cn0
    # Cr_beta = P.gamma4 * Cl_beta + P.gamma8 * Cn_beta
    # Cr_p = P.gamma4 * Cl_p + P.gamma8 * Cn_p
    # Cr_r = P.gamma4 * Cl_r + P.gamma8 * Cn_r
    #
    # C5 = (-P.gamma1 * p * q + P.gamma2 * q * r) / (C0 * P.b)
    # C6 = (-P.gamma7 * p * q + P.gamma1 * q * r) / (C0 * P.b)
    # v0 = C5 - Cp_0 - Cp_beta * beta - Cp_p * P.b * p * 0.5 / Va - Cp_r * P.b * r * 0.5 / Va
    # v1 = C6 - Cr_0 - Cr_beta * beta - Cr_p * P.b * p * 0.5 / Va - Cr_r * P.b * r * 0.5 / Va
    # v = [v0, v1]
    # B = np.array([[Cp_delta_a, Cp_delta_r], [Cr_delta_a, Cr_delta_r]], dtype=np.double)
    # if Cp_delta_r == 0. and Cr_delta_r == 0.:
    #     return [v0 / B[0][0], 0.]
    # elif Cp_delta_a == 0. and Cr_delta_a == 0.:
    #     return [0.0, v1 / B[1][1]]
    # else:
    #     _delta_a_delta_r = np.dot(np.linalg.inv(B), v)
    #     return _delta_a_delta_r[0], _delta_a_delta_r[1]


    lhs = np.matrix([[P.CpDeltaA, P.CpDeltaR], [P.CrDeltaA, P.CrDeltaR]], dtype=np.double)

    rhs1T1_num = -P.gamma1 * p * q + P.gamma2 * q * r
    rhs1T1_denom = (1.0 / 2.0) * P.rho * (Va**2) * P.S * P.b
    rhs1T1 = rhs1T1_num / rhs1T1_denom
    # rhs1T2 = -P.Cp0 - P.CpBeta * beta - P.CpP * ((P.b * p) / (2.0 * Va)) - P.CpR * ((P.b * r) / (2.0 * Va))
    rhs1 = rhs1T1 - P.Cp0 - P.CpBeta * beta - P.CpP * ((P.b * p) / (2.0 * Va)) - P.CpR * ((P.b * r) / (2.0 * Va))

    rhs2T1_num = -P.gamma7 * p * q + P.gamma1 * q * r
    rhs2T1_denom = (1.0 / 2.0) * P.rho * (Va**2) * P.S * P.b
    rhs2T1 = rhs2T1_num / rhs2T1_denom
    # rhs2T2 = -P.Cr0 - P.CrBeta * beta - P.CrP * ((P.b * p) / (2.0 * Va)) - P.CrR * ((P.b * r) / (2.0 * Va))
    rhs2 = rhs2T1 - P.Cr0 - P.CrBeta * beta - P.CrP * ((P.b * p) / (2.0 * Va)) - P.CrR * ((P.b * r) / (2.0 * Va))

    rhs = np.matrix([[rhs1],[rhs2]], dtype=np.double)
    delAR = np.linalg.solve(lhs, rhs)
    validSolution = np.allclose(np.dot(lhs, delAR), rhs)
    if not validSolution:
        print('error in Delta A and Delta R calculation')
    deltaA = delAR.item(0)
    deltaR = delAR.item(1)
    return deltaA, deltaR

#This Code was all replaced with the code in the forces.py file and the Dynamics.py file
# """
# Calculate the derivatives of the equations of motion
# eqn's 5.3-5.12 (pg 61)
# """
# def computeFxu(trimmedState, controlSurfaces, inputs, planeValues):
#     """
#     Calculate the rate of change of the state of the UAV
#     :param trimmedState: Current state of the UAV
#     :param controlSurfaces: The current position on the control surfaces[A, E, R, T]
#     :param inputs: the designated values that trim is to be found for [Va, gamma, R]
#     :param planeValues: The values associate with the plane [alpha, beta, phi]
#     :return: the derivatives of the UAV state [u, v, w, phi, theta, psi, p, q, r]dots
#     """
#     u = trimmedState[0]
#     v = trimmedState[1]
#     w = trimmedState[2]
#     phi = trimmedState[3]
#     theta = trimmedState[4]
#     psi = trimmedState[5]
#     p = trimmedState[6]
#     q = trimmedState[7]
#     r = trimmedState[8]
#
#     delA = controlSurfaces[0]
#     delE = controlSurfaces[1]
#     delR = controlSurfaces[2]
#     delT = controlSurfaces[3]
#
#     Va = inputs[0]
#     gamma = inputs[1]
#     R = inputs[2]
#
#     alpha = planeValues[0]
#     beta = planeValues[1]
#     phi = planeValues[2]
#
#     # Equation 5.3
#     hDot = u * sin(theta) - v * sin(phi) * cos(theta) - w * cos(phi) * cos(theta)
#
#
#     # Equation 5.4
#     uDotT1 = r * v - q * w - P.g * sin(theta)
#     uDotT2_1 = P.rho * (Va**2) * P.S / (2.0 * P.mass)
#     uDotT2_2 = CxAlpha(alpha) + Cxq(alpha) * ((P.c * q) / (2.0 * Va)) + CxDelE(alpha) * delE
#     uDotT2 = uDotT2_1 * uDotT2_2
#     uDotT3_1 = P.rho * P.Sprop * P.Cprop / (2.0 * P.mass)
#     uDotT3_2 = (P.kMotor * delT)**2 - Va**2
#     uDotT3 = uDotT3_1 * uDotT3_2
#     uDot = uDotT1 + uDotT2 + uDotT3
#
#
#     # Equation 5.5
#     vDotT1 = p * w - r * u + P.g * cos(theta) * sin(phi)
#     vDotT2_1 = P.rho * (Va**2) * P.S / (2 * P.mass)
#     vDotT2_2 = (P.Cy0 + P.CyBeta * beta + P.CyP * ((P.b * p) / (2.0 * Va)) +
#                P.CYr * ((P.b * r) / (2.0 * Va)) + P.CyDeltaa * delA + P.CyDeltar * delR)
#     vDotT2 = vDotT2_1 * vDotT2_2
#     vDot = vDotT1 + vDotT2
#
#
#     # Equation 5.6
#     wDotT1 = q * u - p * v + P.g * cos(theta) * cos(phi)
#     wDotT2_1 = P.rho * (Va**2) * P.S / (2.0 * P.mass)
#     wDotT2_2 = (Cz(alpha) + Czq(alpha) * ((P.c * q) / (2.0 * Va)) + CzDelE(alpha) * delE)
#     wDotT2 = wDotT2_1 * wDotT2_2
#     wDot = wDotT1 + wDotT2
#
#
#     # Equation 5.7
#     phiDotT1 = p + q * sin(phi) * np.tan(theta)
#     phiDotT2 = r * cos(phi) * np.tan(theta)
#     phiDot = phiDotT1 + phiDotT2
#
#
#     # Equation 5.8
#     thetaDot = q * cos(phi) - r * sin(phi)
#
#
#     # Equation 5.9
#     psiDot = q * (sin(phi) / cos(theta)) +  r * (cos(phi) / cos(theta))
#
#
#     # Equation 5.10
#     pDotT1 = P.gamma1 * p * q - P.gamma2 * q * r
#     pDotT2_1 = (1.0 / 2.0) * P.rho * (Va**2) * P.S * P.b
#     pDotT2_2 = P.Cp0 + P.CpBeta * beta + P.CpP * ((P.b * p) / (2.0 * Va)) + \
#                P.CpR * ((P.b * r) / (2.0 * Va)) + P.CpDeltaA * delA + P.CpDeltaR * delR
#     pDotT2 = pDotT2_1 * pDotT2_2
#     pDot = pDotT1 + pDotT2
#
#
#     # Equation 5.11
#     qDotT1 = P.gamma5 * p * r - P.gamma6 * ((p**2) - (r**2))
#     qDotT2_1 = P.rho * (Va**2) * P.S * P.c / (2.0 * P.jy)
#     qDotT2_2 = P.Cm0 + P.CmAlpha * alpha + P.Cmq * ((P.c * q) / (2.0 * Va)) + P.CmDeltae * delE
#     qDotT2 = qDotT2_1 * qDotT2_2
#     qDot = qDotT1 + qDotT2
#
#
#     #Equation 5.12
#     rDotT1 = P.gamma7 * p * q - P.gamma1 * q * r
#     rDotT2_1 = (1.0 / 2.0) * P.rho * (Va**2) * P.S * P.b
#     rdotT2_2 = P.Cr0 + P.CrBeta * beta + P.CrP * ((P.b * p) / (2.0 * Va)) + \
#                P.CrR * ((P.b * r) / (2.0 * Va)) + P.CrDeltaA * delA + P.CrDeltaR * delR
#     rDotT2 = rDotT2_1 * rdotT2_2
#     rDot = rDotT1 + rDotT2
#
#     # Make the state derivative list
#     stateDot = [hDot, uDot, vDot, wDot, phiDot, thetaDot, psiDot, pDot, qDot, rDot]
#     return stateDot

"""
The algorithms to calculate the trim of the aircraft (pgs. 281-284)
"""
def calculateTrim(inputs, printValues=False):
    """
    Calculate the trim inputs of the UAV with an optimization routine
    :param inputs: the desired trimmed inputs [Va, gamma, R]
    :return: the trimmed values for the control surfaces delA, delE, delR, delT
    """
    alphaG = 0.0
    betaG = 0.0
    phiG = 0.0
    alpha, beta, phi = minimizeJ(alphaG, betaG, phiG, inputs)
    planeAngles = [alpha, beta, phi]

    trimmedState = computeTrimStates(inputs, planeAngles)

    delE = deltaE(trimmedState, inputs, planeAngles)
    delT = deltaT(trimmedState, inputs, planeAngles, delE)
    delA, delR = deltaAR(trimmedState, inputs, planeAngles)

    if printValues:
        print('Aileron Angle:\t%s' %delA)
        print('Elevator Angle:\t%s' %delE)
        print('Rudder Angle:\t%s' %delR)
        print('Throttle Positon:\t%s' %delT)
    return trimmedState, delA, delE, delR, delT

def calculateJ(alpha, beta, phi, inputs, printError=False):
    """
    Calculate the J value that is the object of the optimization
    :param alpha: angle of attack
    :param beta: sideslip angle
    :param phi: pitch
    :param inputs: the desired trim conditons [Va, gamma, R]
    :param printError: boolean used for debugging, you shouldn't need to use this
    :return: J: the norm of the error vector squared
    """
    Va = inputs[0]
    gamma = inputs[1]
    R = inputs[2]

    planeValues = [alpha, beta, phi]

    xDot = np.zeros((12,1), dtype=np.float)
    xDot[2] = -Va * sin(gamma)
    xDot[8] = (Va / R) * cos(gamma)

    trimmedState = computeTrimStates(inputs, planeValues)

    delE = deltaE(trimmedState, inputs, planeValues)
    delT = deltaT(trimmedState, inputs, planeValues, delE)
    delA, delR = deltaAR(trimmedState, inputs, planeValues)

    trimmedControl = np.array([delA, delE, delR, delT])

    fxu = getF(trimmedState, Va, alpha, beta, trimmedControl)


    forces_moments = force.getForces(trimmedState, Va, alpha, beta, trimmedControl)
    # forces_moments = calculate_forces_moments(trimmedState, Va, alpha, beta, trimmedControl)
    # fxu = dynam.Derivatives(trimmedState, forces_moments)
    # fxu3 = dynam.Derivatives(trimmedState, forces_moments2)

    fxu[0] = 0.0
    fxu[1] = 0.0

    error = np.subtract(xDot, fxu)
    norm = np.linalg.norm(error)
    j = norm**2
    if printError:
        print(error.T)
    return j

def minimizeJ(alpha, beta, phi, inputs):
    """
    Optimization routine to minimize the j value
    used to determine the alpha, beta and phi that
    are required to support the specified inputs
    :param alpha: angle of attack guess
    :param beta: sideslip angle guess
    :param phi: pitch guess
    :param inputs: desired flight path of the aircraft [Va, gamma, R]
    :return: the determined alpha, beta, phi
    """
    epsilon = 0.000001
    k = 0.0000001
    precision = 0.000000002001
    J = 1000
    count = 0
    while J > precision:
        count += 1
        alphaStep = alpha + epsilon
        betaStep = beta + epsilon
        phiStep = phi + epsilon
        J = calculateJ(alpha, beta, phi, inputs)
        if count % 1000 == 0:
            J = calculateJ(alpha, beta, phi, inputs, printError=True)
            print(J)
        if np.isnan(J):
            print("nan value found in J, at step %s Optimization diverged :(" %count)
            break

        djdalpha = (calculateJ(alphaStep, beta, phi, inputs) - J) / epsilon
        djdbeta = (calculateJ(alpha, betaStep, phi, inputs) - J) / epsilon
        djdphi = (calculateJ(alpha, beta, phiStep, inputs) - J) / epsilon

        alpha = alpha -k * djdalpha
        beta = beta -k * djdbeta
        phi = phi -k * djdphi

        if count > 30000:
            print("after %s cycles the optimization didn't converge" %count)
            print("When the cycle was broken the J value was %s" %J)
            break
    print("calculation ended after %s steps with a J value of %s" %(count,J))
    return alpha, beta, phi

def getF(y, Va, alpha, beta, control_input):
    mass = P.mass
    Jx = P.jx
    Jy = P.jy
    Jz = P.jz
    Jxz = P.jxz
    gamma_0 = Jx * Jz - Jxz ** 2
    gamma_1 = (Jxz * (Jx - Jy + Jz)) / gamma_0
    gamma_2 = (Jz * (Jz - Jy) + Jxz ** 2) / gamma_0
    gamma_3 = Jz / gamma_0
    gamma_4 = Jxz / gamma_0
    gamma_5 = (Jz - Jx) / Jy
    gamma_6 = Jxz / Jy
    gamma_7 = ((Jx - Jy) * Jx + Jxz ** 2) / gamma_0
    gamma_8 = Jx / gamma_0

    # pn = y[0]
    # pe = y[1]
    # pd = y[2]
    u = y[3]
    v = y[4]
    w = y[5]
    phi = y[6]
    theta = y[7]
    psi = y[8]
    p = y[9]
    q = y[10]
    r = y[11]
    cr = np.cos(phi)
    sr = np.sin(phi)

    cp = np.cos(theta)
    sp = np.sin(theta)
    tp = np.tan(theta)

    cy = np.cos(psi)
    sy = np.sin(psi)

    # forces_moments = force.getForces(y, Va, alpha, beta, control_input)
    forces_moments = calculate_forces_moments(y, Va, alpha, beta, control_input)
    fx = forces_moments[0]
    fy = forces_moments[1]
    fz = forces_moments[2]
    l = forces_moments[3]
    m = forces_moments[4]
    n = forces_moments[5]

    dy = np.zeros((12,1), dtype=np.double)
    dy[0] = cp * cy * u + (sr * sp * cy - cr * sy) * v + (cr * sp * cy + sr * sy) * w
    dy[1] = cp * sy * u + (sr * sp * sy + cr * cy) * v + (cr * sp * sy - sr * cy) * w
    dy[2] = -sp * u + sr * cp * v + cr * cp * w
    dy[3] = r * v - q * w + fx / mass
    dy[4] = p * w - r * u + fy / mass
    dy[5] = q * u - p * v + fz / mass
    dy[6] = p + sr * tp * q + cr * tp * r
    dy[7] = cr * q - sr * r
    dy[8] = sr / cp * q + cr / cp * r
    dy[9] = gamma_1 * p * q - gamma_2 * q * r + gamma_3 * l + gamma_4 * n
    dy[10] = gamma_5 * p * r - gamma_6 * (p * p - r * r) + m / Jy
    dy[11] = gamma_7 * p * q - gamma_1 * q * r + gamma_4 * l + gamma_8 * n
    return dy


def forces_and_moments(y, control_inputs, Va, alpha, beta):

    mass = P.mass
    S = P.S
    b = P.b
    c = P.c
    rho = P.rho
    e = P.e
    S_prop = P.Sprop
    k_motor = P.kMotor
    kT_p = P.kTp
    kOmega = P.kOmega

    # pn = y[0]
    # pe = y[1]
    # pd = y[2]
    u = y[3]
    v = y[4]
    w = y[5]
    phi = y[6]
    theta = y[7]
    psi = y[8]
    p = y[9]
    q = y[10]
    r = y[11]

    # Va = np.sqrt(u ** 2 + v ** 2 + w ** 2)
    # if u > 0:
    #     alpha = np.arctan(w / u)
    # elif Va == 0:
    #     alpha = 0
    # else:
    #     alpha = np.pi / 2
    #
    # if Va > 0:
    #     beta = np.arcsin(v / Va)
    # else:
    #     beta = 0

    delta_e = control_inputs[0]
    delta_a = control_inputs[1]
    delta_r = control_inputs[2]
    delta_t = control_inputs[3]

    def longitudinal_aerodynamic_forces_moments():  # srho, c, S, Va, Clong_coeffs, alpha, q, delta_e):
        CL0 = P.CL0
        CL_alpha = P.ClAlpha
        CL_q = P.Clq
        CL_delta_e = P.ClDeltae
        M = P.M
        alpha_0 = P.alpha0
        c1 = np.exp(-M * (alpha - alpha_0))
        c2 = np.exp(M * (alpha + alpha_0))
        sigmoid_alpha = (1 + c1 + c2) / ((1 + c1) * (1 + c2))
        CL_alpha_NL = (1. - sigmoid_alpha) * (CL0 + CL_alpha * alpha) + sigmoid_alpha * 2. * np.sign(alpha) * np.sin(
            alpha) * np.sin(alpha) * np.cos(alpha)
        lift = 0.5 * rho * S * (CL_alpha_NL * Va ** 2 + CL_q * c * q * 0.5 * Va + CL_delta_e * delta_e * Va ** 2)

        CD0 = P.Cd0
        CD_alpha = P.CdAlpha
        CD_q = P.Cdq
        CD_delta_e = P.CdDeltae
        CD_p = P.Cdp
        AR = b ** 2 / S
        CD_alpha = CD_p + (CL0 + CL_alpha * alpha) ** 2 / (np.pi * e * AR)
        drag = 0.5 * rho * S * (CD_alpha * Va ** 2 + CD_q * c * q * 0.5 * Va + CD_delta_e * delta_e * Va ** 2)

        Cm0 = P.Cm0
        Cm_alpha = P.CmAlpha
        Cm_q = P.Cmq
        Cm_delta_e = P.CmDeltae
        Cm_alpha = Cm0 + Cm_alpha * alpha
        # delta_e = -Cm_alpha/Cm_delta_e
        m = 0.5 * rho * S * c * (Cm_alpha * Va ** 2 + Cm_q * c * q * 0.5 * Va + Cm_delta_e * delta_e * Va ** 2)

        fx = -drag * np.cos(alpha) + lift * np.sin(alpha)
        fz = -drag * np.sin(alpha) - lift * np.cos(alpha)
        return fx, fz, m

    def lateral_forces_moments():  # rho, b, S, Va, lateral_coeffs, beta, p, r, delta_a, delta_r):
        const = 0.5 * rho * S
        CY0 = P.Cy0
        CY_beta = P.CyBeta
        CY_p = P.CyP
        CY_r = P.CYr
        CY_delta_a = P.CyDeltaa
        CY_delta_r = P.CyDeltar
        fy = const * (
            CY0 * Va ** 2 + CY_beta * beta * Va ** 2 + CY_p * b * p * 0.5 * Va + CY_r * r * b * 0.5 * Va +
            CY_delta_a * delta_a * Va ** 2 + CY_delta_r * delta_r * Va ** 2)

        Cl0 = P.Cl0
        Cl_beta = P.ClBeta
        Cl_p = P.ClP
        Cl_r = P.Clr
        Cl_delta_a = P.ClDeltae
        Cl_delta_r = P.ClDeltar
        l = b * const * (
            Cl0 * Va ** 2 + Cl_beta * beta * Va ** 2 + Cl_p * b * p * 0.5 * Va + Cl_r * r * b * 0.5 * Va +
            Cl_delta_a * delta_a * Va ** 2 + Cl_delta_r * delta_r * Va ** 2)

        Cn0 = P.Cn0
        Cn_beta = P.CnBeta
        Cn_p = P.CnP
        Cn_r = P.Cnr
        Cn_delta_a = P.CnDeltaa
        Cn_delta_r = P.CnDeltar
        n = b * const * (
            Cn0 * Va ** 2 + Cn_beta * beta * Va ** 2 + Cn_p * b * p * 0.5 * Va + Cn_r * r * b * 0.5 * Va +
            Cn_delta_a * delta_a * Va ** 2 + Cn_delta_r * delta_r * Va ** 2)
        return fy, l, n

    def gravitational_forces():  # mass, theta, phi):
        g = 9.81
        fx = -mass * g * np.sin(theta)
        fy = mass * g * np.cos(theta) * np.sin(phi)
        fz = mass * g * np.cos(theta) * np.cos(phi)
        return fx, fy, fz

    def propeller_forces():  # rho, S_prop, Clong_coeffs, k_motor, delta_t = 1):
        C_prop = P.Cprop
        fx = 0.5 * rho * S_prop * C_prop * (k_motor ** 2 * delta_t ** 2 - Va ** 2)
        fy = 0.
        fz = 0.
        return fx, fy, fz

    def propeller_torques():  # kT_p, kOmega, delta_t = 1):
        l = -kT_p * kOmega ** 2 * delta_t ** 2
        m = 0.
        n = 0.
        return l, m, n

    def wind(Va):
        return 0., 0., 0.

    f_lon_aerodynamic_x, f_lon_aerodynamic_z, m_aerodynamic = longitudinal_aerodynamic_forces_moments()  # rho, c, b, S, e, Va, Clong_coeffs, alpha, q, delta_e)
    f_lat_aerodynamic_y, l_aerodynamic, n_aerodynamic = lateral_forces_moments()  # rho, b, S, Va, Clateral_coeffs, beta, p, r, delta_a, delta_r)
    f_gravity_x, f_gravity_y, f_gravity_z = gravitational_forces()  # mass, theta, phi)
    f_prop_x, f_prop_y, f_prop_z = propeller_forces()  # rho, S_prop, Clong_coeffs, k_motor, delta_t)
    l_prop, m_prop, n_prop = propeller_torques()  # kT_p, kOmega, delta_t)
    fx = f_lon_aerodynamic_x + f_gravity_x + f_prop_x
    fy = f_lat_aerodynamic_y + f_gravity_y + f_prop_y
    fz = f_lon_aerodynamic_z + f_gravity_z + f_prop_z
    l = l_aerodynamic + l_prop
    m = m_aerodynamic + m_prop
    n = n_aerodynamic + n_prop
    return np.array([[fx], [fy], [fz], [l], [m], [n]])
    # return [fx, fy, fz], [l, m, n]

def calculate_forces_moments(y, Va, alpha, beta, control_inputs):
    # pn = y[0]
    # pe = y[1]
    # pd = y[2]
    u = y[3]
    v = y[4]
    w = y[5]
    phi = y[6]
    theta = y[7]
    psi = y[8]
    p = y[9]
    q = y[10]
    r = y[11]

    delta_a = control_inputs[0]
    delta_e = control_inputs[1]
    delta_r = control_inputs[2]
    delta_t = control_inputs[3]

    force_gavity = np.array([[-P.mass * P.g * np.sin(theta)],
                       [P.mass * P.g * np.cos(theta) * np.sin(phi)],
                       [P.mass * P.g * np.cos(theta) * np.cos(phi)]])

    force_aerodynamics1 = np.array([[CxAlpha(alpha) + Cxq(alpha) * P.c * q / (2.0 * Va) + CxDelE(alpha) * delta_e],
                                   [P.Cy0 + P.CyBeta * beta + P.CyP * P.b * p / (2.0 * Va) + P.CYr * P.b * r / (2.0 * Va) + P.CyDeltaa * delta_a + P.CyDeltar * delta_r],
                                   [Cz(alpha) + Czq(alpha) * P.c * q / (2.0 * Va) + CzDelE(alpha) * delta_e]])
    force_aerodynamics = 0.5 * P.rho * Va**2 * P.S * force_aerodynamics1

    force_motor1 = np.array([[(P.kMotor * delta_t)**2 - Va**2],
                             [0.0],
                             [0.0]])
    force_motor = 0.5 * P.rho * P.Sprop * P.Cprop * force_motor1

    force = force_gavity + force_aerodynamics + force_motor

    moment_aerodynamics1 = np.array([[P.b * (P.Cl0 + P.ClBeta * beta + P.ClP * p * P.b / (2.0 * Va) + P.Clr * r * P.b / (2.0 * Va) + P.ClDeltaa * delta_a + P.ClDeltar * delta_r)],
                                     [P.c * (P.Cm0 + P.CmAlpha * alpha + P.Cmq * q * P.c / (2.0 * Va) + P.CmDeltae * delta_e)],
                                     [P.b * (P.Cn0 + P.CnBeta * beta + P.CnP * p * P.b / (2.0 * Va) + P.Cnr * r * P.b / (2.0 * Va) + P.CnDeltaa * delta_a + P.CnDeltar * delta_r)]])
    moment_aerodynamics = np.multiply(0.5 * P.rho * Va**2 * P.S, moment_aerodynamics1)
    moment_motor = np.array([[-P.kTp * (P.kOmega * delta_t)**2],
                             [0.0],
                             [0.0]])
    moment = moment_aerodynamics + moment_motor
    return np.vstack((force, moment))

if __name__ == "__main__":
    Va = 35.0
    gamma = np.radians(0)
    R = np.inf
    inputs = [Va, gamma, R]
    staters, a1,b1,c1,d1 = calculateTrim(inputs, printValues=True)
    print(staters)


