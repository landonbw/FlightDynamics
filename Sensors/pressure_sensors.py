import sys
from numpy import zeros
import numpy.random as rand

sys.path.append('..')
from Sensors.sensorabstract import sensorAbstract
import ParamFiles.AerosondeParameters as P


class AltitudeSensor(sensorAbstract):
    def __init__(self):
        super(AltitudeSensor, self).__init__(gain=1.0, bias=P.ps_beta_abs_pressure, std_dev=P.ps_std_dev)
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

    def sensor_value(self, state):
        h = -state.item(2)
        abs_pres = P.rho * P.g * h + self.bias + rand.normal(loc=0.0, scale=self.std_dev)
        return abs_pres


class AirspeedSensor(sensorAbstract):
    def __init__(self):
        super(AirspeedSensor, self).__init__(gain=1.0, bias=P.aspeed_beta_diff_pressure, std_dev=P.aspeed_std_dev)

    def sensor_value(self, state):
        va = state
        diff_pres = P.rho * va**2 / 2.0 + self.bias + rand.normal(loc=0.0, scale=self.std_dev)
        return diff_pres

if __name__ == '__main__':
    va = 5
    stater = zeros((12, 1))
    alt = AltitudeSensor()
    aspeed = AirspeedSensor()
    for i in range(20):
        print(alt.sensor_value(stater), aspeed.sensor_value(va))


