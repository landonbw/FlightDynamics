import pyqtgraph.opengl as gl
from pyqtgraph.Qt import QtCore, QtGui
from math import sin, cos
import pyqtgraph as pg
import sys
import numpy as np
import copy
import pyqtgraph.Vector as Vector

sys.path.append("..")
import ParamFiles.AerosondeParameters as P
import ViewerFiles.Grapher as GR
import CH12.PathManager as PM


timers = []


class Viewer(QtCore.QThread):
    def __init__(self):
        """
        Initialize the viewer module
        :param model: The model that contains the current state of the model and the info required to calculate the next step
        :param timerPause: The wait between calculating and displaying each frame of the simulation
        """
        QtCore.QThread.__init__(self)       #Threading initialization

        #Create the graph window to display the state of the UAV
        self.plotThread = GR.UAVplot()
        self.plotThread.start(QtCore.QThread.LowestPriority)
        # self.plotThread.start()

        #Initialize the window to display the simulation
        self.app = QtGui.QApplication([])
        self.w = gl.GLViewWidget()
        self.w.show()
        self.w.setWindowTitle('UAV Simulator')
        self.w.setCameraPosition(distance=1000)
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

        self.exec_()    #And Go!
        #


    def update(self, state, y_manager=None):
        """
        Tell the model to update with the given inputs
        get the updated state
        update UAV plot with updated state
        """
        self.draw_mav(state, y_manager)        #Update UAV plot

    def init_plots(self, state, flight_state, va, wind, flight_state_com, sensors, att_filt, gps_filt, gps_unfilt, simple_kin=None):
        self.plotThread.initializePlots(state, flight_state, va, wind, flight_state_com, sensors, att_filt, gps_filt, gps_unfilt, simple_kin)

    def update_plot_data(self, state, flight_state, va, wind, flight_state_com, sensors, att_filt, gps_filt, gps_unfilt, simple_kin=None):
        self.plotThread.updateData(state, flight_state, va, wind, flight_state_com, sensors, att_filt, gps_filt, gps_unfilt, simple_kin)

    def updatePlots(self):
        """
        Method to update the plots of the UAV.
        Gets the full state from the model and updates the plot window
        with the information
        :return:
        """
        # self.plotThread.updateData(state, flight_state, va, wind, flight_state_com, sensors, att_filt, gps_filt)
        self.plotThread.updatePlots()

    def draw_buildings(self, map):
        """
        Draw the buildings that represent the world
        :param map: map object that is generated in the CreateWorld class
        :return: none
        """
        building_meshes = []
        building_colors = []
        for b in map.buildings:
            building_meshes.append([b.b1, b.b2, b.t2])
            building_meshes.append([b.b1, b.t1, b.t2])
            building_meshes.append([b.b1, b.b3, b.t1])
            building_meshes.append([b.b3, b.t1, b.t3])
            building_meshes.append([b.b2, b.b4, b.t4])
            building_meshes.append([b.b2, b.t2, b.t4])
            building_meshes.append([b.b3, b.b4, b.t3])
            building_meshes.append([b.b4, b.t3, b.t4])
            building_meshes.append([b.t1, b.t2, b.t4])
            building_meshes.append([b.t1, b.t3, b.t4])
        mesh_array = np.asarray(building_meshes)
        color_array = np.empty((mesh_array.shape[0], 3, 4))
        for i in range(mesh_array.shape[0]):
            color_array[i] = np.array([20/256.0, 20/256.0, 20/256.0, 1])
        building_view = gl.GLMeshItem(vertexes=mesh_array, smooth=False, drawEdges=True, vertexColors=color_array)
        # building_view.setColor([0.9, 0.7, 0.5, 1])
        self.w.addItem(building_view)

    def draw_path(self, start, end, path_color=[0, 0, 255, 0.3]):
        path_pos = np.vstack((start.reshape((1,3)), end.reshape(1,3)))
        path_pos = path_pos * np.array([[1, -1, -1], [1, -1, -1]])
        # path_color = [0, 0, 255, 1]
        path = gl.GLLinePlotItem(pos=path_pos, color=path_color, width=1, antialias=True, mode='lines')
        self.w.addItem(path)

    def draw_multisegment_path(self, points, path_color=[255, 0, 0, 0.7]):
        points_array = np.zeros((len(points), 3))
        for i in range(len(points)):
            points_array[i] = points[i].reshape((1,3)) * np.array([1, -1, -1])
        path = gl.GLLinePlotItem(pos=points_array, color = path_color, width=2, antialias=True, mode='line_strip')
        self.w.addItem(path)


    def draw_mav(self, state, y_manager):
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

            path_pos = np.array([[0, 0, 0], [0, 0, 0]])
            path_color = [0, 255, 0, 1]
            self.path = gl.GLLinePlotItem(pos=path_pos, color=path_color, width=1, antialias=True, mode='lines')
            self.w.addItem(self.path)

        if self.init:
            #   rotate bodies to specified state
            #precalculate sines and cosines
            cPhi = np.cos(phi)
            sPhi = np.sin(phi)
            cTheta = np.cos(theta)
            sTheta = np.sin(theta)
            cPsi = np.cos(psi)
            sPsi = np.sin(psi)
            # rotMat = np.array([[cTheta * cPsi,                      -cTheta*sPsi,                        sTheta], #Make the rotation matrix
            #                    [sPhi * sTheta * cPsi + cPhi * sPsi, -sPhi * sTheta * sPsi + cPhi * cPsi, -sPhi * cTheta],
            #                    [-cPhi * sTheta * cPsi + sPhi * sPsi, cPhi * sTheta * sPsi + sPhi * cPsi, cPhi * cTheta]])

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
            if y_manager is not None:
                if y_manager[0] is True:
                    a = y_manager[2] - 500000 * y_manager[3]
                    b = y_manager[2] + 500000 * y_manager[3]
                    path_pos = np.array([[a.item(0), -a.item(1), -a.item(2)],
                                        [b.item(0), -b.item(1), -b.item(2)]])
                    self.path.setData(pos=path_pos)
                else:
                    c = y_manager[4]
                    rho = y_manager[5]
                    cx = c.item(0)
                    cy = -c.item(1)
                    cz = -c.item(2)
                    theta_circ = np.linspace(0, 2*np.pi, 100)
                    theta_circ = theta_circ.reshape((100,1))
                    x = cx + rho * np.cos(theta_circ)
                    y = cy + rho * np.sin(theta_circ)
                    z = np.linspace(cz, cz, 100)
                    z = z.reshape((100,1))
                    path_pos = np.hstack((x, y, z))

                    self.path.setData(pos=path_pos, mode='line_strip')

        viewLoc = Vector(loc[0], loc[1], loc[2])    #vector to define the new camera view location
        self.w.opts['center'] = viewLoc     #update the camera view location to maintain UAV in the center of the view
        if self.init is False:  #Flip the init flag if it's false
            self.init = True


if __name__ == '__main__':
    import sys

    graphic = Viewer()
    graphic.draw_mav([1.0,1.0,1.0, 0., 0., 0.])
    # for i in range(5000):
    #     graphic.draw_mav([300.0/500 * i, 0, 200.0/500 * i, 0.025 * i, 0.025 * i, 0.025 * i])
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

