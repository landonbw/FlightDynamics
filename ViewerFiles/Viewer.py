import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore, QtGui
import pyqtgraph as pg
import sys
import numpy as np
import copy
import pyqtgraph.Vector as Vector

sys.path.append("..")
import ParamFiles.AerosondeParameters as P
import ViewerFiles.Grapher as GR


timers = []


class Viewer(QtCore.QThread):
    def __init__(self, model, timerPause=P.ts_simulation*1000):
        """
        Initialize the viewer module
        :param model: The model that contains the current state of the model and the info required to calculate the next step
        :param timerPause: The wait between calculating and displaying each frame of the simulation
        """
        QtCore.QThread.__init__(self)       #Threading initialization
        self.model = model                  #store the UAV model that will define the state of the UAV
        self.timerPause = timerPause
        #Create the graph window to display the state of the UAV
        self.plotThread = GR.UAVplot()
        self.plotThread.start(QtCore.QThread.LowestPriority)
        # self.plotThread.start()

        #Initialize the window to display the simulation
        self.app = QtGui.QApplication([])
        self.w = gl.GLViewWidget()
        self.w.show()
        self.w.setWindowTitle('UAV Simulator')
        self.w.setCameraPosition(distance=10)
        self.g = gl.GLGridItem()            #make a grid to represent the ground
        self.g.scale(20, 20, 20)               #set the size of the grid (distance between each line)
        self.w.addItem(self.g)              #add grid to viewer
        self.init = False       #has the uav been plotted yet?
        self.body = None        #initialize the variable that will store the mesh object of the UAV
        self.verts = np.array([[P.points[0], P.points[1], P.points[2]],     #verticies of the meshes for the UAV
                      [P.points[0], P.points[1], P.points[4]],
                      [P.points[0], P.points[3], P.points[4]],
                      [P.points[0], P.points[3], P.points[2]],
                      [P.points[5], P.points[2], P.points[3]],
                      [P.points[5], P.points[1], P.points[2]],
                      [P.points[5], P.points[1], P.points[4]],
                      [P.points[5], P.points[3], P.points[4]],
                      [P.points[6], P.points[7], P.points[9]],
                      [P.points[7], P.points[8], P.points[9]],
                      [P.points[10], P.points[11], P.points[12]],
                      [P.points[10], P.points[12], P.points[13]],
                      [P.points[5], P.points[14], P.points[15]]])
        self.colors = P.bodyColors      #Colors of the meshes


    def __del__(self):
        """
        Required for multi-threaded qt apps (I think)
        """
        self.wait()

    def run(self):
        """
        Entrance into the thread for the viewer window
        """
        self.window_timer = QtCore.QTimer()     #Create a timer to update the window simulation
        self.window_timer.timeout.connect(self.update)      #Connect timer to the update function on timeout
        timers.append(self.window_timer)        #Add timer handle to global timers list (required so that timer isn't deleted by trash collector)
        self.window_timer.start(self.timerPause)
        # Create timer to update plots
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self.updatePlots)
        timers.append(self.plot_timer)
        self.plot_timer.start(350)
        # self.plot_timer.start(100)
        # Create timer to update control surfaces
        if self.model.chapter >= 5:
            self.control_timer = QtCore.QTimer()
            self.control_timer.timeout.connect(self.update_controls)
            timers.append(self.control_timer)
            self.control_timer.start(int(P.ts_control * 1000))
        if self.model.chapter >= 7:
            self.gps_timer = QtCore.QTimer()
            self.gps_timer.timeout.connect(self.update_gps)
            timers.append(self.gps_timer)
            self.gps_timer.start(int(P.gps_sample_time * 1000))
        self.exec_()    #And Go!
        #


    def update(self):
        """
        Tell the model to update with the given inputs
        get the updated state
        update UAV plot with updated state
        """
        self.model.UpdateState()  # update UAV model
        state = self.model.Output()     #get UAV plotting state after update
        # fullState = self.model.EulerStates()
        self.draw_mav(state)        #Update UAV plot
        # self.plotThread.updateData(fullState)
        # self.plotThread.updatePlots()

    def updatePlots(self):
        """
        Method to update the plots of the UAV.
        Gets the full state from the model and updates the plot window
        with the information

        Gets called every time the plot_timer ticks
        :return:
        """
        fullState = self.model.EulerStates()
        flight_state = self.model.get_flight_state()
        commanded_flight_state = self.model.autopilot
        # sensor_data = self.model.sensor_value
        sensor_data = np.vstack((self.model.kalman_attitude, self.model.sensor_values[3:8]))
        # gps_measurements = np.vstack((self.model.measured_gps[0:2], self.model.measured_gps[3]))
        gps_measurements = np.vstack((self.model.kalman_loc[0:2], self.model.kalman_loc[3]))
        self.plotThread.updateData(fullState, flight_state, commanded_flight_state, sensor_data, gps_measurements)
        self.plotThread.updatePlots()

    def update_controls(self):
        self.model.control()

    def update_gps(self):
        self.model.update_gps()

    def draw_mav(self, state):
        """
        Draws the location of the mav at the specified location with the given roll pitch and yaw
        :param state: The state that the mav should be drawn. [X, Y, Z, Roll, Pitch, Yaw]
        :return: null
        """
        #pull out the relevant information from the state vector
        loc = np.array([state[0], -state[1], -state[2]])
        phi = state[3]
        theta = -state[4]
        psi = -state[5]
        if not self.init:   #create body if not yet initialized
            self.body = gl.GLMeshItem(vertexes=self.verts, smooth=False, drawEdges=False, vertexColors=self.colors) #create meshes
            self.w.addItem(self.body)       #add meshes to plot

        if self.init:
            #   rotate bods to specified state
            #precalculate sines and cosines
            cPhi = np.cos(phi)
            sPhi = np.sin(phi)
            cTheta = np.cos(theta)
            sTheta = np.sin(theta)
            cPsi = np.cos(psi)
            sPsi = np.sin(psi)
            rotMat = np.array([[cTheta * cPsi,                      -cTheta*sPsi,                        sTheta], #Make the rotation matrix
                               [sPhi * sTheta * cPsi + cPhi * sPsi, -sPhi * sTheta * sPsi + cPhi * cPsi, -sPhi * cTheta],
                               [-cPhi * sTheta * cPsi + sPhi * sPsi, cPhi * sTheta * sPsi + sPhi * cPsi, cPhi * cTheta]])

            # rotMat = np.array([[cPhi * cTheta, cPhi * sTheta * sPsi - sPhi * cPsi, cPhi * sTheta * cPsi + sPhi * sPsi],
            #                     [sPhi * cTheta, sPhi * sTheta * sPsi + cPhi * cPsi, sPhi * sTheta * cPsi - cPhi * sPsi],
            #                     [-sTheta, cTheta*sPsi, cTheta * cPsi]])
            r_roll = np.array([[1,  0,      0],
                               [0,  cPhi,   -sPhi],
                               [0,  sPhi,   cPhi]])

            r_pitch = np.array([[cTheta,    0,  sTheta],
                                [0,         1,  0],
                                [-sTheta,   0,  cTheta]])

            r_yaw = np.array([[cPsi,    -sPsi,  0],
                              [sPsi,    cPsi,   0],
                              [0,       0,      1]])
            # rotMat = r_yaw * r_pitch * r_roll
            # rotMat = r_roll * r_pitch * r_yaw
            # newPoints = np.array([rotMat.dot(P.points[0]) + loc,        #rotate and translate original points
            #                       rotMat.dot(P.points[1]) + loc,
            #                       rotMat.dot(P.points[2]) + loc,
            #                       rotMat.dot(P.points[3]) + loc,
            #                       rotMat.dot(P.points[4]) + loc,
            #                       rotMat.dot(P.points[5]) + loc,
            #                       rotMat.dot(P.points[6]) + loc,
            #                       rotMat.dot(P.points[7]) + loc,
            #                       rotMat.dot(P.points[8]) + loc,
            #                       rotMat.dot(P.points[9]) + loc,
            #                       rotMat.dot(P.points[10]) + loc,
            #                       rotMat.dot(P.points[11]) + loc,
            #                       rotMat.dot(P.points[12]) + loc,
            #                       rotMat.dot(P.points[13]) + loc,
            #                       rotMat.dot(P.points[14]) + loc,
            #                       rotMat.dot(P.points[15]) + loc])

            newPoints = np.array([np.dot(r_yaw,np.dot(r_pitch, np.dot(r_roll, P.points[0]))) + loc,
                                  np.dot(r_yaw,np.dot(r_pitch, np.dot(r_roll, P.points[1]))) + loc,
                                  np.dot(r_yaw,np.dot(r_pitch, np.dot(r_roll, P.points[2]))) + loc,
                                  np.dot(r_yaw,np.dot(r_pitch, np.dot(r_roll, P.points[3]))) + loc,
                                  np.dot(r_yaw,np.dot(r_pitch, np.dot(r_roll, P.points[4]))) + loc,
                                  np.dot(r_yaw,np.dot(r_pitch, np.dot(r_roll, P.points[5]))) + loc,
                                  np.dot(r_yaw,np.dot(r_pitch, np.dot(r_roll, P.points[6]))) + loc,
                                  np.dot(r_yaw,np.dot(r_pitch, np.dot(r_roll, P.points[7]))) + loc,
                                  np.dot(r_yaw,np.dot(r_pitch, np.dot(r_roll, P.points[8]))) + loc,
                                  np.dot(r_yaw,np.dot(r_pitch, np.dot(r_roll, P.points[9]))) + loc,
                                  np.dot(r_yaw,np.dot(r_pitch, np.dot(r_roll, P.points[10]))) + loc,
                                  np.dot(r_yaw,np.dot(r_pitch, np.dot(r_roll, P.points[11]))) + loc,
                                  np.dot(r_yaw,np.dot(r_pitch, np.dot(r_roll, P.points[12]))) + loc,
                                  np.dot(r_yaw,np.dot(r_pitch, np.dot(r_roll, P.points[13]))) + loc,
                                  np.dot(r_yaw,np.dot(r_pitch, np.dot(r_roll, P.points[14]))) + loc,
                                  np.dot(r_yaw,np.dot(r_pitch, np.dot(r_roll, P.points[15]))) + loc])

            #convert points to xyz coordinates

            newVerts = np.array([[newPoints[0], newPoints[1], newPoints[2]],    #Make the new verticies of the meshes
                                 [newPoints[0], newPoints[1], newPoints[4]],
                                 [newPoints[0], newPoints[3], newPoints[4]],
                                 [newPoints[0], newPoints[3], newPoints[2]],
                                 [newPoints[5], newPoints[2], newPoints[3]],
                                 [newPoints[5], newPoints[1], newPoints[2]],
                                 [newPoints[5], newPoints[1], newPoints[4]],
                                 [newPoints[5], newPoints[3], newPoints[4]],
                                 [newPoints[6], newPoints[7], newPoints[9]],
                                 [newPoints[7], newPoints[8], newPoints[9]],
                                 [newPoints[10], newPoints[11], newPoints[12]],
                                 [newPoints[10], newPoints[12], newPoints[13]],
                                 [newPoints[5], newPoints[14], newPoints[15]]])

            self.body.setMeshData(vertexes=newVerts, vertexColors=self.colors)

        viewLoc = Vector(loc[0], loc[1], loc[2])    #vector to define the new camera view location
        self.w.opts['center'] = viewLoc     #update the camera view location to maintain UAV in the center of the view
        if self.init is False:  #Flip the init flag if it's false
            self.init = True


if __name__ == '__main__':
    import sys

    graphic = Viewer()
    graphic.draw_mav([1.0,1.0,1.0, 0., 0., 0.])
    for i in range(5000):
        graphic.draw_mav([300.0/500 * i, 0, 200.0/500 * i, 0.025 * i, 0.025 * i, 0.025 * i])
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

