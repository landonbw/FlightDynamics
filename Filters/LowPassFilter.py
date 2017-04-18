import numpy.random as rand
import matplotlib.pyplot as plt


class LowPassFilter(object):
    def __init__(self, alpha, gain=1):
        self.gain = gain
        self.alpha = alpha
        self.y = False

    def filter(self, val):
        if self.y is not False:
            y_new = self.alpha * self.y + (1 - self.alpha) * val
            self.y = y_new
        else:
            self.y = val
        return self.gain * self.y

class SensorFilter(object):
    def __init__(self, alphas):
        self.alpha = alphas
        self.y = False

    def filter(self, vals):
        if self.y is not False:
            y_new = self.alpha * self.y + (1 - self.alpha) * vals
            self.y = y_new
        else:
            self.y = vals
        return self.y



if __name__ == '__main__':
    filt = LowPassFilter(2, 0.95)
    filt2 = LowPassFilter(1, 0.95)

    filt_data = []
    filt_data2 = []

    rand_data = rand.normal(scale=0.4, size=(1000,1))



    for val in range(len(rand_data)):
        filt_data.append(filt.filter(rand_data[val]))
        filt_data2.append(filt2.filter(rand_data[val]))
        # print(rand_data[val], filt_data[val])



    plt.plot(rand_data)
    plt.plot(filt_data)
    plt.plot(filt_data2)

    plt.show()


