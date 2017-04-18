import numpy as np
from scipy import signal


class Signals(object):
    """ This class inherits the Signal class. It is used to organize
    	1 or more signals of different types: square_wave,
    	sawtooth_wave, triangle_wave, random_wave."""
    # You only need to modify the code inside '#'s as shown below.
    ##################################################################
    #                       CODE TO BE MODIFIED
    ##################################################################
    def __init__(self):
        self.handle = []  # This is the handle to various signals

        ''' Down below you will create mySignal objects for each signal
	    you need. The syntax for each object is as follows:
	    self.Object1 = Signal(A, F,signal_type, phase)
	    self.Object2 = Signal(A=1, F=0.1, signal_type, phase)
	    ...
	    self.ObjectN = Signal(A, F)
	    The class MySignal parameters can be passed by reference or name.
	    The parameters also have default values as shown below. This makes
	    creating a myClass simple if the default values are wanted.
	    self.handle.append(mySignal())
	    For a description of each parameter, see the mySignal Class
	    below.
	    All mySignal objects need to be appended to the list self.handle '''

    #SECTION 1
    ##################################################################
        self.handle.append(mySignal(A=0.0, f=0.1, signal_type='triangle_wave', phase=0.1, offset=45)) # airspeed
        self.handle.append(mySignal(A=50, f=0.05, signal_type='square_wave', phase=0, offset=200)) # altitude
        self.handle.append(mySignal(A=1.2, f=0.005, signal_type='square_wave', phase=np.pi/2.0, offset=0.0)) # course
    ##################################################################

	# This function returns the values of the signal generator as
	# as a function of time. The order in which the values are returned is
	# the same order that the mySignal objects were appended to the
	# list self.handle.
    def getRefInputs(self,t):
        ref_inputs = []
        for i in range(len(self.handle)):
            ref_inputs.append(self.handle[i].signal(t))
        return np.asarray(ref_inputs)


class mySignal:
    def __init__(self,A = 1.0,f = 1.0,signal_type = 'square_wave', phase = 0.0, offset=0.0):
        self.A = A                  # Amplitude of the signal
        self.f = f                  # Frequency of the signal, Hz
        self.phase = phase          # Phase of the signal, Radians
        self.offset = offset

        if signal_type == 'square_wave':
            self.signal = self.square_wave
        elif signal_type == 'sawtooth_wave':
            self.signal = self.sawtooth_wave
        elif signal_type == 'triangle_wave':
            self.signal = self.triangle_wave
        elif signal_type == 'random_wave':
            self.signal = self.random_wave
        else:
            print("input signal type not recognized")
    def square_wave(self,t):
        return self.A*signal.square((2*np.pi*self.f*t+self.phase), duty = 0.5) + self.offset

    def sawtooth_wave(self,t):
        return self.A*signal.sawtooth((2*np.pi*self.f*t + self.phase), width = 0) + self.offset

    def triangle_wave(self,t):
        return self.A*signal.sawtooth((2*np.pi*self.f*t + self.phase), width = 0.5) + self.offset

    def random_wave(self,t):
        return self.A*np.random.rand() + self.offset


if __name__ == "__main__":
    sig = Signals()
    for i in range(50):
        print(sig.getRefInputs(i))