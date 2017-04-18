import sys
import numpy as np
from PyQt5.QtCore import *
from pyqtgraph.Qt import QtCore, QtGui
from PyQt5.QtWidgets import *
import time

sys.path.append('..')
# from KinematicsAndDynamics.Dynamics import Dynamics
import ParamFiles.AerosondeParameters as P
from ViewerFiles.Viewer import Viewer
from Model.MAVQuaternionModel import MAVModel

timers = []

class Sliders(QWidget):
    def __init__(self, parent = None):
        super(Sliders, self).__init__(parent)
        # QWidget.__init__(self)
        # QtCore.QThread.__init__(self)

        self.model = MAVModel(chapter=7)

        self.layout = QVBoxLayout()
        self.numOfSliders = 2

        self.airspeed = mySlider(self.numOfSliders, 0, maxV=50, minV=10, intV=P.Va0, gain=1, name='Airspeed', model=self.model)
        self.altitude = mySlider(self.numOfSliders, 1, maxV=300, minV=0, intV=P.altitude0, gain=1, name='Altitude', model=self.model)
        self.course_angle = mySlider(self.numOfSliders, 2, maxV=180, minV=-180, intV=0, gain=np.pi / 180.0, name='Course Angle', model=self.model)

        self.layout.addWidget(QLabel('Airspeed'))
        self.layout.addWidget(self.airspeed.slider)
        self.layout.addWidget(QLabel('Altitude'))
        self.layout.addWidget(self.altitude.slider)
        self.layout.addWidget(QLabel('course_angle'))
        self.layout.addWidget(self.course_angle.slider)

        self.setLayout(self.layout)
        self.setWindowTitle("Control Values")
        self.resize(500, 300)
        self.move(20, 20)

        # self.start = QtCore.QTimer()
        # timers.append(self.start)
        # self.start.setSingleShot(True)
        # self.start.timeout.connect(self.print_vals)
        # self.start.start(1000)

        # self.thread = Viewer(self.model)
        # self.thread.draw_mav(self.model.Output())
        # self.thread.start()
        # QtGui.QApplication.instance().exec_()
        self.printer = PrintInputs(self.model)
        self.printer.start()


    def getInputValue(self):
        """
        :return: The current values of the sliders in a list
        """
        values = [self.airspeed.getValue(), self.altitude.getValue(), self.course_angle.getValue()]
        return values




class mySlider:
    def __init__(self, numOfSliders = 1, sliderNumber = 1, maxV = 1, minV = -1, intV = 0, gain = 1., name = 'Slider', model = None):
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
        self.data = self.initValue / self.scaleValue
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
        self.model.autopilot.itemset(self.sliderNumber, self.getValue())
        # print(self.model.autopilot.tolist())

    def getValue(self):
        return self.data * self.gain

class PrintInputs(QtCore.QThread):
    def __init__(self, model):
        QThread.__init__(self)
        self.model = model
        # self.app = QApplication(sys.argv)
        # self.ex = Sliders()
        # self.ex.show()

        # self.start = QtCore.QTimer()
        # timers.append(self.start)
        # self.start.setSingleShot(True)
        # self.start.timeout.connect(self.print_vals)
        # self.start.start(1000)

        # self.window = Sliders()
        # self.window.show()
        # self.window_thread = SliderThread()
        # self.window.moveToThread(self.window_thread)
        # self.window_thread.start()
        # QtGui.QApplication.instance().exec_()

        # self.app.instance().exec()

    def print_vals(self):
        # for I in range(50):
        while True:
            time.sleep(0.1)
            print(self.model.autopilot.T)

    def run(self):
        self.print_vals()
        self.exec_()


# class SliderThread(QThread):
#     def __init__(self):
#         QThread.__init__(self)
#         self.window = Sliders()
#         self.window.show()
#
#     def run(self):
#         # self.window.show()
#         self.exec_()
#
#     def getInputs(self):
#         return self.window.getInputValue()


def main():
    # app = QApplication(sys.argv)
    # thing = PrintInputs()
    # app.instance().exec_()
   app = QApplication(sys.argv)
   ex = Sliders()
   ex.show()
   # # for i in range(500000000):
   # #     print(ex.getInputValue())
   # # sys.exit(app.exec_())
   app.instance().exec_()




if __name__ == '__main__':
   main()
