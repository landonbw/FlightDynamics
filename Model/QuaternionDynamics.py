"""
Dynamics file that calculates the states of the mav given the forces and moments that are acting on the mav
Implemented with a runga-kutta solver for the second order differential equations.  Done using in quaternions in place
of Euler angles to avoid gimbal lock
"""
import sys
import numpy as np

sys.path.append('..')
import ParamFiles.AerosondeParameters as P

class Dynamics:
    def __init__(self):
        #   gamma values pulled from pg 36
        self.gamma = P.jx * P.jz - P.jxz**2
        self.gamma1 = (P.jxz * (P.jx - P.jy + P.jz)) / self.gamma
        self.gamma2 = (P.jz * (P.jz - P.jy) + P.jxz**2) / self.gamma
        self.gamma3 = P.jz / self.gamma
        self.gamma4 = P.jxz / self.gamma
        self.gamma5 = (P.jz - P.jx) / P.jy
        self.gamma6 = P.jxz / P.jy
        self.gamma7 = ((P.jx - P.jy) * P.jx + P.jxz**2) / self.gamma
        self.gamma8 = P.jx / self.gamma
        self.lam = 100.0

    def PropagateDynamics(self, state, u):
        """
        Calculates the next step of the model based on the given forces in the u vector using a runge-kutta solver
        :param state: The current state of the system(pn, pe, pd, u, v, w, phi, theta, psi, p, q, r)
        :param u: forces/moments applied to the mav (fx, fy, fz, l, m, n)
        :return: The new state of the system as calculated in the runga-kutta solver
        """
        time_step = P.ts_simulation / P.num_steps
        k1 = self.Derivatives(state, u)
        k2 = self.Derivatives(state + time_step / 2 * k1, u)
        k3 = self.Derivatives(state + time_step / 2 * k2, u)
        k4 = self.Derivatives(state + time_step * k3, u)
        newState = state + time_step / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
        return newState

    def Derivatives(self, state, U):
        """
        for internal use only
        :param state: current state of the simulation (for runge-kutta use)
        :param U: forces/moments applied to the mav
        :return: the derivatives of the state vector in the same order as the state vector
        """
        #   extract state variables from the state vector
        Pn = state.item(0)
        Pe = state.item(1)
        Pd = state.item(2)
        u = state.item(3)
        v = state.item(4)
        w = state.item(5)
        e0 = state.item(6)
        e1 = state.item(7)
        e2 = state.item(8)
        e3 = state.item(9)
        p = state.item(10)
        q = state.item(11)
        r = state.item(12)
        #   extract forces/moments from the u vector
        fx = U.item(0)
        fy = U.item(1)
        fz = U.item(2)
        l = U.item(3)
        m = U.item(4)
        n = U.item(5)

        #   Calculate the inertial reference frame location of the plane
        body2inertial = np.matrix([[e1**2 + e0**2 - e2**2 - e3**2, 2 * (e1 * e2 - e3 * e0), 2 * (e1 * e3 + e2 * e0)],
                                   [2 * (e1 * e2 + e3 * e0), e2**2 + e0**2 - e1**2 - e3**2, 2 * (e2 * e3 - e1 * e0)],
                                   [2 * (e1 * e3 - e2 * e0), 2 * (e2 * e3 + e1 * e0), e3**2 + e0**2 - e1**2 - e2**2]])

        locDot = np.dot(body2inertial, np.matrix([[u], [v], [w]]))


        #   Calculate the body frame velocities of the plane
        cross = np.matrix([[r * v - q * w],
                           [p * w - r * u],
                           [q * u - p * v]])

        forces = np.matrix([[fx/P.mass],
                            [fy/P.mass],
                            [fz/P.mass]])
        velDot = np.add(cross, forces)


        #   Calculate the pitch, roll and yaw with respect to vehicle frame 0,1,2
        e = np.matrix([[e0], [e1], [e2], [e3]])
        normE = np.linalg.norm(e)
        correctionTerm = self.lam * (1 - normE**2)
        angVelMat = np.matrix([[correctionTerm, -p, -q, -r],
                               [p, correctionTerm, r, -q],
                               [q, -r, correctionTerm, p],
                               [r, q, -p, correctionTerm]])

        attitudeDot = 0.5 * np.dot(angVelMat, np.matrix([[e0], [e1], [e2], [e3]]))


        #   Calculate the rotational accelerations of the plane
        mat1 = np.matrix([[self.gamma1 * p * q - self.gamma2 * q * r],
                          [self.gamma5 * p * r - self.gamma6 * (p**2 - r**2)],
                          [self.gamma7 * p * q - self.gamma1 * q * r]])
        mat2 = np.matrix([[self.gamma3 * l + self.gamma4 * n],
                          [(1 / P.jy) * m],
                          [self.gamma4 * l + self.gamma8 * n]])
        angVelDot = np.add(mat1, mat2)

        xDot = np.vstack((locDot, velDot, attitudeDot, angVelDot))
        return xDot



if __name__ == "__main__":
    d = Dynamics()
    print(d.States())
    d.PropagateDynamics(np.array([1,1,1,1,1,1]))
    print(d.States())
