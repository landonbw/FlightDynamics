import sys
from PyQt5.QtCore import *
from pyqtgraph.Qt import QtCore, QtGui
from PyQt5.QtWidgets import *

sys.path.append('..')
# from KinematicsAndDynamics.Dynamics import Dynamics
from ViewerFiles.Viewer import Viewer
from Model.MAVModelDONTUSEME import MAVModel

class Sliders(QWidget):
    def __init__(self, parent = None):
        super(Sliders, self).__init__(parent)

        self.model = MAVModel(chapter=3)

        self.layout = QVBoxLayout()
        self.numOfSliders = 6

        self.X = mySlider(self.numOfSliders, 0, maxV=1, minV=-1, intV=0, gain=1, name='X Force', model=self.model)
        self.Y = mySlider(self.numOfSliders, 1, maxV=1, minV=-1, intV=0, gain=1, name='Y Force', model=self.model)
        self.Z = mySlider(self.numOfSliders, 2, maxV=1, minV=-1, intV=0, gain=1, name='Z Force', model=self.model)
        self.roll = mySlider(self.numOfSliders, 3, maxV=0.1, minV=-0.1, intV=0, gain=1, name='L (x moment)', model=self.model)
        self.pitch = mySlider(self.numOfSliders, 4, maxV=0.1, minV=-0.1, intV=0, gain=1, name='M (y moment)', model=self.model)
        self.yaw = mySlider(self.numOfSliders, 5, maxV=0.1, minV=-0.1, intV=0, gain=1, name='N (z moment)', model=self.model)

        self.layout.addWidget(QLabel('Fx'))
        self.layout.addWidget(self.X.slider)
        self.layout.addWidget(QLabel('Fy'))
        self.layout.addWidget(self.Y.slider)
        self.layout.addWidget(QLabel('Fz'))
        self.layout.addWidget(self.Z.slider)
        self.layout.addWidget(QLabel('L'))
        self.layout.addWidget(self.roll.slider)
        self.layout.addWidget(QLabel('M'))
        self.layout.addWidget(self.pitch.slider)
        self.layout.addWidget(QLabel('N'))
        self.layout.addWidget(self.yaw.slider)


        self.setLayout(self.layout)
        self.setWindowTitle("UAV Forces")

        self.thread = Viewer(self.model, 2)
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
        print(self.data)
        self.model.bodyForces.itemset(self.sliderNumber, self.getValue())

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
