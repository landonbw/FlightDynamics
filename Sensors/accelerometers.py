import abc
from numpy import sin, cos, zeros
import numpy.random as rand
import sys

sys.path.append('..')
import ParamFiles.AerosondeParameters as P
from Sensors.sensorabstract import sensorAbstract


class Accelerometer(sensorAbstract):
    def __init__(self):
        super(Accelerometer, self).__init__(P.accel_gain, 0.0, P.accel_std_dev)
#
# self.bodyForces = np.matrix([[P.Fx],
#                              [P.Fy],
#                              [P.Fz],
#                              [P.Ml],
#                              [P.Mm],
#                              [P.Mn]])
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


class xAccel(Accelerometer):
    def __init__(self):
        super(xAccel, self).__init__()

    def sensor_value(self, state):
        [forces, states] = state
        fx = forces.item(0)
        theta = states.item(7)
        accel = fx / P.mass + (P.g * sin(theta)) + rand.normal(loc=0.0, scale=self.std_dev)
        return accel


class yAccel(Accelerometer):
    def __init__(self):
        super(yAccel, self).__init__()

    def sensor_value(self, state):
        [forces, states] = state
        fy = forces.item(1)
        phi = states.item(6)
        theta = states.item(7)
        accel = fy / P.mass - (P.g * cos(theta) * sin(phi)) + rand.normal(loc=0.0, scale=self.std_dev)
        return accel


class zAccel(Accelerometer):
    def __init__(self):
        super(zAccel, self).__init__()

    def sensor_value(self, state):
        [forces, states] = state
        fz = forces.item(2)
        phi = states.item(6)
        theta = states.item(7)
        accel = fz / P.mass - (P.g * cos(theta) * cos(phi)) + rand.normal(loc=0.0, scale=self.std_dev)
        return accel


if __name__ == '__main__':
    forces = zeros((4, 1))
    forces += 13.5
    states = zeros((12, 1))
    x = xAccel()
    y = yAccel()
    z = zAccel()
    for i in range(50):
        print(y.sensor_value([forces, states]), x.sensor_value([forces, states]), z.sensor_value([forces, states]))
