import abc

class sensorAbstract(metaclass=abc.ABCMeta):
    def __init__(self, gain=1.0, bias=0.0, std_dev=1.0):
        self.gain = gain
        self.bias = bias
        self.std_dev = std_dev

    @abc.abstractmethod
    def sensor_value(self, state):
        """
        Return the
        :param state: State information that is required to calculate the sensor value

        for accelerometer this is [forces, state]

        for rate gyros it is just the state

        for the altitude sensor it is just the state

        for the airspeed sensor it is the magnitude of the airspeed

        for the gps channels it is the value of the position that we are
        approximating (ie pn for the north gps channel)

        for the gps it is [state, wind, va]

        :return: the sensor value
        """
        pass