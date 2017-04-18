"""
A class that makes the sampling all of the sensors easy.  It aggregates the different signals
and filters them for you.
"""
import sys
import numpy as np
import numpy.linalg as lalg


sys.path.append('..')
import ParamFiles.AerosondeParameters as P
from Sensors.pressure_sensors import AltitudeSensor, AirspeedSensor
from Sensors.accelerometers import xAccel, yAccel, zAccel
from Sensors.rate_gyros import RollGyro, PitchGyro, YawGyro
from Filters.LowPassFilter import LowPassFilter


class Sensors(object):
    def __init__(self):
        self.accel_x = xAccel()
        self.accel_y = yAccel()
        self.accel_z = zAccel()
        self.rate_roll = RollGyro()
        self.rate_pitch = PitchGyro()
        self.rate_yaw = YawGyro()
        self.altitude = AltitudeSensor()
        self.airspeed = AirspeedSensor()

        # self.x_filt = LowPassFilter(1, P.accel_alpha)
        # self.y_filt = LowPassFilter(1, P.accel_alpha)
        # self.z_filt = LowPassFilter(1, P.accel_alpha)
        # self.roll_filt = LowPassFilter(1, P.rate_alpha)
        # self.roll_filt = LowPassFilter(1, 0.0)
        # self.pitch_filt = LowPassFilter(1, P.rate_alpha)
        # self.yaw_filt = LowPassFilter(1, P.rate_alpha)
        # self.aspeed_filt = LowPassFilter(1, P.aspeed_alpha)
        # self.h_filt = LowPassFilter(1 / (P.rho * P.g), P.ps_alpha)


    def sensor_values(self, state, forces, va):
        """
        makes getting all the data from the sensors easy.

        :param state: the current euler state of the UAV, get your quaternions out of here

        :param forces: the current forces acting on the UAV

        :param va: the current airspeed of the UAV, the vector, not the magnitude.
        :return:
        """
        x_accel = self.accel_x.sensor_value([forces, state])
        y_accel = self.accel_y.sensor_value([forces, state])
        z_accel = self.accel_z.sensor_value([forces, state])

        # phi = np.arctan2(y_accel, z_accel)
        # theta = np.arcsin(x_accel / P.g)

        roll_rate = self.rate_roll.sensor_value(state)
        pitch_rate = self.rate_pitch.sensor_value(state)
        yaw_rate = self.rate_yaw.sensor_value(state)

        altitude = self.altitude.sensor_value(state)
        altitude = altitude / (P.rho * P.g)

        va_mag = lalg.norm(va)
        va_filt = self.airspeed.sensor_value(va_mag)
        va_cond = np.sqrt((2 / P.rho) * va_filt)

        return np.array([[x_accel], [y_accel], [z_accel], [roll_rate], [pitch_rate], [yaw_rate], [altitude], [va_cond]])
