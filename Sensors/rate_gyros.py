import sys
from numpy import zeros
import numpy.random as rand

sys.path.append('..')
import ParamFiles.AerosondeParameters as P
from Sensors.sensorabstract import sensorAbstract

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


class RollGyro(sensorAbstract):
    def __init__(self):
        super(RollGyro, self).__init__(gain=1.0, bias=0.0, std_dev=P.rate_gyro_std_dev)

    def sensor_value(self, state):
        p_measured = state.item(9) + rand.normal(0.0, self.std_dev)
        return p_measured


class PitchGyro(sensorAbstract):
    def __init__(self):
        super(PitchGyro, self).__init__(gain=1.0, bias=0.0, std_dev=P.rate_gyro_std_dev)

    def sensor_value(self, state):
        q_measured = state.item(10) + rand.normal(0.0, self.std_dev)
        return q_measured


class YawGyro(sensorAbstract):
    def __init__(self):
        super(YawGyro, self).__init__(gain=1.0, bias=0.0, std_dev=P.rate_gyro_std_dev)

    def sensor_value(self, state):
        r_measured = state.item(11) + rand.normal(0.0, self.std_dev)
        return r_measured

if __name__ == '__main__':
    stater = zeros((12, 1))
    x = RollGyro()
    y = PitchGyro()
    z = YawGyro()
    for i in range(20):
        print(x.sensor_value(stater), y.sensor_value(stater), z.sensor_value(stater))