import sys
import numpy as np
from PyQt5.QtCore import *
from pyqtgraph.Qt import QtCore, QtGui
from PyQt5.QtWidgets import *

sys.path.append('..')
# from KinematicsAndDynamics.Dynamics import Dynamics
import ParamFiles.AerosondeParameters as P
from ViewerFiles.Viewer import Viewer
from Model.MAVQuaternionModel import MAVModel


class Sliders(QWidget):
    def __init__(self, parent = None):
        super(Sliders, self).__init__(parent)

        self.model = MAVModel(chapter=4)

        self.layout = QVBoxLayout()
        self.numOfSliders = 6

        self.deltaA = mySlider(self.numOfSliders, 0, maxV=15, minV=-15, intV=P.deltaA0 * 180 / np.pi, gain=np.pi / 180.0, name='delta A', model=self.model)
        self.deltaE = mySlider(self.numOfSliders, 1, maxV=60, minV=-60, intV=P.deltaE0 * 180 / np.pi, gain=np.pi / 180.0, name='delta E', model=self.model)
        self.deltaR = mySlider(self.numOfSliders, 2, maxV=5, minV=-5, intV=P.deltaR0 * 180 / np.pi, gain=np.pi / 180.0, name='delta R', model=self.model)
        self.deltaT = mySlider(self.numOfSliders, 3, maxV=1, minV=  0, intV=P.deltaT0, gain=1, name='delta T', model=self.model)

        self.layout.addWidget(QLabel('Delta A'))
        self.layout.addWidget(self.deltaA.slider)
        self.layout.addWidget(QLabel('Delta E'))
        self.layout.addWidget(self.deltaE.slider)
        self.layout.addWidget(QLabel('Delta R'))
        self.layout.addWidget(self.deltaR.slider)
        self.layout.addWidget(QLabel('Delta T'))
        self.layout.addWidget(self.deltaT.slider)



        self.setLayout(self.layout)
        self.setWindowTitle("Control Values")
        self.resize(500, 300)
        self.move(20, 20)

        self.thread = Viewer(self.model, 16)
        self.thread.draw_mav(self.model.Output())
        self.thread.start()
        # QtGui.QApplication.instance().exec_()

    def getInputValue(self):
        """
        :return: The current values of the sliders in a list
        """
        values = [self.X.getValue(), self.Y.getValue(), self.Z.getValue(), self.roll.getValue(), self.pitch.getValue(),
                  self.yaw.getValue()]
        return values

class mySlider:
    def __init__(self, numOfSliders = 1, sliderNumber = 1, maxV = 1, minV = -1, intV = 0, gain = 1, name = 'Slider', model = None):
        """
        :param numOfSliders: Not used
        :param sliderNumber: position in model array that slider will update
        :param maxV: Maximum value
        :param minV: Minimum Value
        :param intV: Initial Value
        :param gain: Gain of the slider (ex display slider in degrees, but output to program in radians)
        :param name: Name of the slider (doesn't actually do anything)
        :param model: Model of the UAV
        """
        self.scaleValue = 1000  #scale to give resolution in the thousandths when sliders only handle ints

        self.sliderNumber = sliderNumber    #slider number corresponding to position in the model array that will be updated by the slider
        self.maxValue = maxV*self.scaleValue
        self.minValue = minV*self.scaleValue
        self.initValue = intV*self.scaleValue
        self.gain = gain
        self.data = self.initValue
        self.model = model

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMaximum(self.maxValue)
        self.slider.setMinimum(self.minValue)
        self.slider.setValue(self.initValue)


        self.slider.valueChanged.connect(self.valuechange)

    def valuechange(self):
        """
        Function called when the value of the slider is changed
        """
        self.data = self.slider.value() / self.scaleValue   #calculate and store the desired value of the slider
        # print(self.data)
        self.model.controlState.itemset(self.sliderNumber, self.getValue())
        print(self.model.controlState.tolist())

    def getValue(self):
        return self.data * self.gain

def main():
   app = QApplication(sys.argv)
   ex = Sliders()
   ex.show()
   # for i in range(500000000):
   #     print(ex.getInputValue())
   # sys.exit(app.exec_())
   app.instance().exec_()



if __name__ == '__main__':
   main()
