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
        # self.app = QApplication(sys.argv)

        self.model = MAVModel(2)

        self.layout = QVBoxLayout()
        self.numOfSliders = 6

        self.X = mySlider(self.numOfSliders, 0, maxV=10, minV=-10, intV=0, gain=1, name='X Pos', dynam=self.model)
        self.Y = mySlider(self.numOfSliders, 1, maxV=10, minV=-10, intV=0, gain=1, name='Y Pos', dynam=self.model)
        self.Z = mySlider(self.numOfSliders, 2, maxV=10, minV=-10, intV=0, gain=1, name='Z Pos', dynam=self.model)
        self.roll = mySlider(self.numOfSliders, 6, maxV=180, minV=-180, intV=0, gain=3.14159/180, name='roll', dynam=self.model)
        self.pitch = mySlider(self.numOfSliders, 7, maxV=180, minV=-180, intV=0, gain=3.14159/180, name='pitch', dynam=self.model)
        self.yaw = mySlider(self.numOfSliders, 8, maxV=180, minV=-180, intV=0, gain=3.14159/180, name='yaw', dynam=self.model)

        self.layout.addWidget(QLabel('x'))
        self.layout.addWidget(self.X.slider)
        self.layout.addWidget(QLabel('y'))
        self.layout.addWidget(self.Y.slider)
        self.layout.addWidget(QLabel('z'))
        self.layout.addWidget(self.Z.slider)
        self.layout.addWidget(QLabel('roll'))
        self.layout.addWidget(self.roll.slider)
        self.layout.addWidget(QLabel('pitch'))
        self.layout.addWidget(self.pitch.slider)
        self.layout.addWidget(QLabel('yaw'))
        self.layout.addWidget(self.yaw.slider)


        self.setLayout(self.layout)
        self.setWindowTitle("UAV Position")
        self.resize(500, 300)
        self.move(25,25)

        self.thread = Viewer(self.model)
        self.thread.draw_mav(self.model.Output())
        self.thread.start()
        # QtGui.QApplication.instance().exec_()

    def getInputValue(self):
        values = [self.X.getValue(), self.Y.getValue(), self.Z.getValue(), self.roll.getValue(), self.pitch.getValue(),
                  self.yaw.getValue()]
        return values

class mySlider:
    def __init__(self,numOfSliders = 1, sliderNumber = 1, maxV = 1, minV = -1, intV = 0, gain = 1, name = 'Slider', dynam = None):
        self.scaleValue = 1000
        self.sliderLayout = QHBoxLayout()
        self.r1 = QLabel(name)
        self.sliderNumber = sliderNumber
        self.sliderLayout.addWidget(self.r1)
        self.maxValue = maxV*self.scaleValue
        self.minValue = minV*self.scaleValue
        self.initValue = intV*self.scaleValue
        self.gain = gain
        self.data = self.initValue
        self.dynam = dynam

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMaximum(self.maxValue)
        self.slider.setMinimum(self.minValue)
        self.slider.setValue(self.initValue)
        # self.slider.setTickPosition(QSlider.TicksBelow)

        self.sliderLayout.addWidget(self.slider)

        self.slider.valueChanged.connect(self.valuechange)

    def valuechange(self):
        self.data = self.slider.value() / self.scaleValue
        print(self.data)
        self.dynam.state.itemset(self.sliderNumber, self.getValue())

    def getValue(self):
        return self.data * self.gain

def main():
   app = QApplication(sys.argv)
   ex = Sliders()
   ex.show()
   app.instance().exec_()



if __name__ == '__main__':
   main()
