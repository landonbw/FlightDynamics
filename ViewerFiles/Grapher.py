
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np

import sys
sys.path.append('..')
import ParamFiles.AerosondeParameters as P



class UAVplot(QtCore.QThread):
    """
    Class to handle the plotting of all the states of the UAV
    """
    def __init__(self):
        QtCore.QThread.__init__(self)
        #   create a wind and add the title
        self.win = pg.GraphicsWindow()
        self.win.setWindowTitle('MAV states')

        self.plots = []

        self.stateData = []
        self.state_curves = []

        self.flight_state_data = []
        self.flight_state_curves = []

        self.va_data = None
        self.va_curve = None

        self.wind_data = []
        self.wind_curves = []

        self.des_flight_state_data = []
        self.des_flight_state_curves = []

        self.sensor_data = []
        self.sensor_curves = []

        self.att_filt_data = []
        self.att_filt_curves = []

        self.gps_data = []
        self.gps_curves = []

        self.gps_sensor_data = []
        self.gps_sensor_curves = []

        self.simple_kin_data = []
        self.simple_kin_curves = []


        self.ptr = -5000

    def initializePlots(self, state, flight_state, va, wind, flight_state_com, sensors, att_filt, gps_filt, gps_unfilt, simple_kin):
        #add all the plots to the window
        distanceLabels = {'left':"distance (m)"}
        self.plots.append(self.win.addPlot(title="North Position", labels=distanceLabels))
        self.plots.append(self.win.addPlot(title="East Position"))
        self.plots.append(self.win.addPlot(title="Down Position"))
        self.win.nextRow()
        velocityLabels = {'left':"velocity (m/s)"}
        self.plots.append(self.win.addPlot(title="U Velocity", labels=velocityLabels))
        self.plots.append(self.win.addPlot(title="V Velocity"))
        self.plots.append(self.win.addPlot(title="W Velocity"))
        self.win.nextRow()
        angularLabels = {'left':"angle (rad)"}
        self.plots.append(self.win.addPlot(title='Roll', labels=angularLabels))
        self.plots.append(self.win.addPlot(title='Pitch'))
        self.plots.append(self.win.addPlot(title='Yaw'))
        self.win.nextRow()
        angVelLabels = {'left':"angular vel (rad/s)", 'bottom':'time (s)'}
        self.plots.append(self.win.addPlot(title='Roll Rate', labels=angVelLabels))
        self.plots.append(self.win.addPlot(title='Pitch Rate'))
        self.plots.append(self.win.addPlot(title='Yaw Rate'))
        # plots for the commanded flight state of the uav
        self.win.nextRow()
        self.plots.append(self.win.addPlot(title='Ground Velocity mag',labels={'left':'vel (m/s)'}))
        self.plots.append(self.win.addPlot(title='Altitude', labels={'left': 'altitude (m)'}))
        self.plots.append(self.win.addPlot(title='Heading', labels={'left': 'Heading (rad)'}))
        # plots for the last of the gps predicted states
        self.win.nextRow()
        self.plots.append(self.win.addPlot(title='Airspeed Velocity mag', labels={'left': 'vel (m/s)'}))
        self.plots.append(self.win.addPlot(title='North Wind', labels={'left': 'vel (m/s)'}))
        self.plots.append(self.win.addPlot(title='East Wind', labels={'left': 'vel (m/s)'}))

        # for plot in self.plots:
        #     plot.setDownsampling(ds=25, auto=True, mode='mean')

        # plot baselines for the uav state
        for i, state_val in enumerate(state):
            curve = np.linspace(state_val, state_val, -self.ptr)
            self.stateData.append(curve)
            self.state_curves.append(self.plots[i].plot(curve, pen='b'))

        # plot baselines for the uav flight state
        for i, f_state_val in enumerate(flight_state):
            curve = np.linspace(f_state_val, f_state_val, -self.ptr)
            self.flight_state_data.append(curve)
            self.flight_state_curves.append(self.plots[i+12].plot(curve, pen='b'))

        # plot the baseline for Vg
        self.va_data = np.linspace(va, va, -self.ptr)
        self.va_curve = self.plots[15].plot(self.va_data, pen='b')

        # plot the baselines for the wind
        for i, wind_val in enumerate(wind):
            curve = np.linspace(wind_val, wind_val, -self.ptr)
            self.wind_data.append(curve)
            self.wind_curves.append(self.plots[i+16].plot(curve, pen='b'))

        # plot baselines for the commanded flight state
        for i, f_state_com in enumerate(flight_state_com):
            curve = np.linspace(f_state_com, f_state_com, -self.ptr)
            self.des_flight_state_data.append(curve)
            self.des_flight_state_curves.append(self.plots[i+12].plot(curve, pen='y'))

        # plot baselines for the sensors
        # for i, sensor_val in enumerate(sensors):
        #     curve = np.linspace(sensor_val, sensor_val, -self.ptr)
        #     self.sensor_data.append(curve)
        # self.sensor_curves.append(self.plots[9].plot(self.sensor_data[0], pen='c'))
        # self.sensor_curves.append(self.plots[10].plot(self.sensor_data[1], pen='c'))
        # self.sensor_curves.append(self.plots[11].plot(self.sensor_data[2], pen='c'))
        # self.sensor_curves.append(self.plots[13].plot(self.sensor_data[3], pen='c'))
        # self.sensor_curves.append(self.plots[15].plot(self.sensor_data[4], pen='c'))
            # self.plots[i + 9].setDownsampling(ds=50, mode='subsample')

        orange_pen = [255, 170, 0]
        # plot baselines for the att filter
        for i in range(len(att_filt)):
            curve = np.linspace(att_filt.item(i), att_filt.item(i), -self.ptr)
            self.att_filt_data.append(curve)
            self.att_filt_curves.append(self.plots[i+6].plot(curve, pen=orange_pen))

        # plot baselines for the gps_filt
        for i, filt_val in enumerate(gps_filt):
            curve = np.linspace(filt_val, filt_val, -self.ptr)
            self.gps_data.append(curve)
        self.gps_curves.append(self.plots[0].plot(self.gps_data[0], pen=orange_pen))
        self.gps_curves.append(self.plots[1].plot(self.gps_data[1], pen=orange_pen))
        self.gps_curves.append(self.plots[12].plot(self.gps_data[2], pen=orange_pen))
        self.gps_curves.append(self.plots[14].plot(self.gps_data[3], pen=orange_pen))
        self.gps_curves.append(self.plots[16].plot(self.gps_data[4], pen=orange_pen))
        self.gps_curves.append(self.plots[17].plot(self.gps_data[5], pen=orange_pen))
        self.gps_curves.append(self.plots[8].plot(self.gps_data[6], pen=orange_pen))

        # for i, val in enumerate(gps_unfilt):
        #     curve = np.linspace(val, val, -self.ptr)
        #     self.gps_sensor_data.append(curve)
        # self.gps_sensor_curves.append(self.plots[0].plot(self.gps_sensor_data[0], pen='r'))
        # self.gps_sensor_curves.append(self.plots[1].plot(self.gps_sensor_data[1], pen='r'))
        # self.gps_sensor_curves.append(self.plots[13].plot(self.gps_sensor_data[2], pen='r'))
        # self.gps_sensor_curves.append(self.plots[14].plot(self.gps_sensor_data[3], pen='r'))
        # self.gps_sensor_curves.append(self.plots[12].plot(self.gps_sensor_data[4], pen='r'))


        # for i, val in enumerate(simple_kin):
        #     curve = np.linspace(val, val, -self.ptr)
        #     self.simple_kin_data.append(curve)
        # self.simple_kin_curves.append(self.plots[0].plot(self.simple_kin_data[0], pen='g'))
        # self.simple_kin_curves.append(self.plots[1].plot(self.simple_kin_data[1], pen='g'))
        # self.simple_kin_curves.append(self.plots[14].plot(self.simple_kin_data[2], pen='g'))
        # self.simple_kin_curves.append(self.plots[13].plot(self.simple_kin_data[3], pen='g'))
        # self.simple_kin_curves.append(self.plots[15].plot(self.simple_kin_data[4], pen='g'))



    def updateData(self, state, flight_state, va, wind, flight_state_com, sensors, att_filt, gps_filt, gps_unfilt, simple_kin):
        self.ptr += 1
        #update the state (pn, pe, pd, roll, pitch,... pitch rate, yaw rate) data
        for i, state in enumerate(state):
            self.stateData[i] = np.roll(self.stateData[i], -1)
            self.stateData[i][-1] = state

        #update the fight state (va, altitude, heading) data
        for i, state in enumerate(flight_state):
            self.flight_state_data[i] = np.roll(self.flight_state_data[i], -1)
            self.flight_state_data[i][-1] = state

        # update the vg data
        self.va_data = np.roll(self.va_data, -1)
        self.va_data[-1] = va

        # update the wind data
        for i, wind_val in enumerate(wind):
            self.wind_data[i] = np.roll(self.wind_data[i], -1)
            self.wind_data[i][-1] = wind_val

        # update the commanded flight state data
        for i, val in enumerate(flight_state_com):
            self.des_flight_state_data[i] = np.roll(self.des_flight_state_data[i], -1)
            self.des_flight_state_data[i][-1] = val

        #update the sensor data
        # for i, state in enumerate(sensors):
        #     self.sensor_data[i] = np.roll(self.sensor_data[i], -1)
        #     self.sensor_data[i][-1] = state

        # update the attitude filter data
        for i, val in enumerate(att_filt):
            self.att_filt_data[i] = np.roll(self.att_filt_data[i], -1)
            self.att_filt_data[i][-1] = val

        # update the gps measurements data
        for i, val in enumerate(gps_filt):
            self.gps_data[i] = np.roll(self.gps_data[i], -1)
            self.gps_data[i][-1] = val
            # self.gps_curves[i].setPos(self.ptr, 0)

        # for i, val in enumerate(gps_unfilt):
        #     self.gps_sensor_data[i] = np.roll(self.gps_sensor_data[i], -1)
        #     self.gps_sensor_data[i][-1] = val

        # for i, val in enumerate(simple_kin):
        #     self.simple_kin_data[i] = np.roll(self.simple_kin_data[i], -1)
        #     self.simple_kin_data[i][-1] = val

    def updatePlots(self):
        # update state plots
        for i, curve in enumerate(self.state_curves):
            self.state_curves[i].setData(self.stateData[i])
            self.state_curves[i].setPos(self.ptr, 0)
        # update flight state and commanded flight state plots
        for i, curve in enumerate(self.flight_state_curves):
            self.des_flight_state_curves[i].setData(self.des_flight_state_data[i])
            self.des_flight_state_curves[i].setPos(self.ptr, 0)
            self.flight_state_curves[i].setData(self.flight_state_data[i])
            self.flight_state_curves[i].setPos(self.ptr, 0)
        # update vg plot
        self.va_curve.setData(self.va_data)
        self.va_curve.setPos(self.ptr, 0)
        # update wind plots
        for i in range(len(self.wind_curves)):
            self.wind_curves[i].setData(self.wind_data[i])
            self.wind_curves[i].setPos(self.ptr, 0)
        # update the sensor plots
        # for i in range(len(self.sensor_curves)):
        #     self.sensor_curves[i].setData(self.sensor_data[i])
        #     self.sensor_curves[i].setPos(self.ptr, 0)
        # update the attitude filter plots
        for i in range(len(self.att_filt_curves)):
            self.att_filt_curves[i].setData(self.att_filt_data[i])
            self.att_filt_curves[i].setPos(self.ptr, 0)
        # update the gps filter plots
        for i in range(len(self.gps_curves)):
            self.gps_curves[i].setData(self.gps_data[i])
            self.gps_curves[i].setPos(self.ptr, 0)

        # for i in range(len(self.gps_sensor_curves)):
        #     self.gps_sensor_curves[i].setData(self.gps_sensor_data[i])
        #     self.gps_sensor_curves[i].setPos(self.ptr, 0)

        # for i in range(len(self.simple_kin_curves)):
        #     self.simple_kin_curves[i].setData(self.simple_kin_data[i])
        #     self.simple_kin_curves[i].setPos(self.ptr, 0)


    def run(self):
        # self.initializePlots()
        self.exec_()


## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    window = UAVplot()
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
