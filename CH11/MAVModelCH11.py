"""
The model that will track the current state of the UAV.
"""
import sys
import numpy as np
import time
from pyqtgraph.Qt import QtCore, QtGui
from PyQt5.QtCore import *
from PyQt5.QtWidgets import *


sys.path.append('..')
from Model.QuaternionDynamics import Dynamics
from Model.WindSimulation import WindSimulation
from Model.Forces import Forces
import ParamFiles.AerosondeParameters as P
from Control.lateral_controller import lateral_control
from Control.longitudinal_controller import longitudinal_control
import Model.rotations as rotations
from Sensors.sensors import Sensors
from Sensors.gps import Gps
from Filters.AttitudeFilter import AttitudeFilter
from Filters.GPSFilter import GPSFilter
from Filters.LowPassFilter import SensorFilter
from ViewerFiles.ViewerNoTimers import Viewer
from Signal.Signals import Signals
from CH9.KinematicEquations import KinematicEquations
from CH11.PathFollower import PathFollower
from CH11.PathManager import PathManager
from CH11.PathPlanner import PathPlanner

timers = []

def Quaternion2Euler(quaternion):
    """
    converts a quaternion attitude to an euler angle attitude
    :param quaternion: the quaternion to be converted to euler angles in a np.matrix
    :return: the euler angle equivalent (phi, theta, psi) in a np.matrix
    """
    e0 = quaternion.item(0)
    e1 = quaternion.item(1)
    e2 = quaternion.item(2)
    e3 = quaternion.item(3)
    phi = np.arctan2(2 * (e0 * e1 + e2 * e3), e0**2 + e3**2 - e1**2 - e2**2)
    theta = np.arcsin(2 * (e0 * e2 - e1 * e3))
    psi = np.arctan2(2 * (e0 * e3 + e1 * e2), e0**2 + e1**2 - e2**2 - e3**2)

    return np.matrix([[phi],[theta],[psi]])

def Euler2Quaternion(euler):
    """
    Converts an euler angle attitude to a quaternian attitude
    :param euler: Euler angle attitude in a np.matrix(phi, theta, psi)
    :return: Quaternian attitude in np.matrix(e0, e1, e2, e3)
    """
    phi = euler.item(0)
    theta = euler.item(1)
    psi = euler.item(2)

    e0 = np.cos(psi/2.0) * np.cos(theta/2.0) * np.cos(phi/2.0) + np.sin(psi/2.0) * np.sin(theta/2.0) * np.sin(phi/2.0)
    e1 = np.cos(psi/2.0) * np.cos(theta/2.0) * np.sin(phi/2.0) - np.sin(psi/2.0) * np.sin(theta/2.0) * np.cos(phi/2.0)
    e2 = np.cos(psi/2.0) * np.sin(theta/2.0) * np.cos(phi/2.0) + np.sin(psi/2.0) * np.cos(theta/2.0) * np.sin(phi/2.0)
    e3 = np.sin(psi/2.0) * np.cos(theta/2.0) * np.cos(phi/2.0) - np.cos(psi/2.0) * np.sin(theta/2.0) * np.sin(phi/2.0)

    return np.matrix([[e0],[e1],[e2],[e3]])

class MAVModel(QWidget):
    def __init__(self, parent = None):
        super(MAVModel, self).__init__(parent)
        # QtCore.QThread.__init__(self)
        #   The current state of the UAV
        # self.viewer = viewer

        self.view_thread = Viewer()
        # self.view_thread.draw_mav(self.Output())
        self.view_thread.start()

        self.state = np.array([[P.Pn0],  # (0)
                               [P.Pe0],  # (1)
                               [P.Pd0],  # (2)
                               [P.u0],  # (3)
                               [P.v0],  # (4)
                               [P.w0],  # (5)
                               [P.e0],  # (6)
                               [P.e1],  # (7)
                               [P.e2],  # (8)
                               [P.e3],  # (9)
                               [P.p0],  # (10)
                               [P.q0],  # (11)
                               [P.r0]])  # (12)
        # The flight state of the UAV
        self.flight_state = np.array([[P.Va0],
                                      [P.alpha0],
                                      [P.beta0],
                                      [P.path_angle0],
                                      [P.heading0]])
        #   The forces that are applied to the body of the UAV
        self.bodyForces = np.array([[P.Fx],
                                    [P.Fy],
                                    [P.Fz],
                                    [P.Ml],
                                    [P.Mm],
                                    [P.Mn]])
        #   The state of the control surfaces
        self.controlState = np.array([[P.deltaA0],
                                      [P.deltaE0],
                                      [P.deltaR0],
                                      [P.deltaT0]])
        # The commanded aircraft path following values
        self.autopilot = np.array([[P.Va0],
                                   [P.altitude0],
                                   [P.heading0]])
        # self.autopilot = np.matrix([[P.Va0],
        #                             [P.altitude0],
        #                             [3.14159]])
        # The attitude of the UAV in euler angles (phi, theta, psi)
        self.eulerAttitude = Quaternion2Euler(self.state[6:10])
        self.euler_state = self.euler_states_array()

        self.kinematics = Dynamics()
        self.windsim = WindSimulation()
        self.forces = Forces()
        self.lat_control = lateral_control()
        self.lon_control = longitudinal_control()
        self.sensors = Sensors()
        self.gps = Gps()

        self.va = np.array([[P.u0], [P.v0], [P.w0]])
        self.wind = np.array([[0], [0], [0]])

        self.sensor_values = np.array([[0], [0], [0], [0], [0], [0], [0], [0]])
        self.measured_gps = np.array([[0], [0], [0], [0], [0]])

        self.kalman_attitude = self.eulerAttitude[0:2]
        self.attitude_filter = AttitudeFilter(self.kalman_attitude)
        self.kalman_loc = np.array([[0], [0], [P.u0], [0], [1], [1], [0]])
        self.loc_filter = GPSFilter(self.kalman_loc)

        self.sensor_LPF = SensorFilter(np.array([[0.1], [0.1], [0.1], [0.81], [0.81], [0.81], [0.51], [0.81]]))

        self.signal_gen = Signals()

        self.t_gps = 0.0
        self.t_plot = 0.0
        self.t_control = 0.0
        self.t_elapse = 0.0

        self.simple_kin = KinematicEquations()
        self.simple_kin_state = np.zeros((5, 1))

        self.path_follow = PathFollower()
        self.path_manager = PathManager()
        self.path_planner = PathPlanner()
        self.phi_ff = 0.0

        self.view_thread.init_plots(self.EulerStates(), self.get_flight_state(), np.linalg.norm(self.va),
                                    self.wind[0:2], self.autopilot, self.sensor_values[3:8], self.kalman_attitude,
                                    self.kalman_loc, self.measured_gps, self.simple_kin_state)


    def calculate_loop(self):
        waypoints = self.path_planner.get_waypoints()
        y_manager = self.path_manager.get_path(waypoints,self.state[0:3])
        if self.t_gps >= P.ts_gps:
            self.update_gps()
            self.t_gps = 0.0

        if self.t_plot >= P.ts_plotting:
            self.view_thread.updatePlots()
            self.t_plot = 0.0

        if self.t_control >= P.ts_control:
            # self.autopilot = self.signal_gen.getRefInputs(self.t_elapse)
            waypoints = self.path_planner.get_waypoints()
            y_manager = self.path_manager.get_path(waypoints, self.state[0:3])
            h_c, chi_c, self.phi_ff = self.path_follow.follow_path(y_manager, self.state[0:4], self.flight_state[4], self.state.item(6))
            self.autopilot = np.array([[y_manager[1]], [h_c], [chi_c]])
            self.control()
            self.t_control = 0.0

        for i in range(P.num_steps):
            self.kalman_attitude = self.attitude_filter.predict(P.ts_simulation / P.num_steps,
                                                                self.sensor_values)
            self.kalman_loc = self.loc_filter.predict(P.ts_simulation / P.num_steps,
                                                      np.array([self.sensor_values[7],
                                                                self.sensor_values[4],
                                                                self.sensor_values[5],
                                                                self.kalman_attitude[0],
                                                                self.kalman_attitude[1]]), )
        self.sensor_values = self.sensors.sensor_values(self.euler_state, self.bodyForces, self.va)
        self.kalman_attitude = self.attitude_filter.update(P.ts_simulation / P.num_steps,
                                                           self.sensor_values)
        self.update_wind()
        self.update_flight_state()
        self.bodyForces = self.forces.getForces(self.euler_state, self.flight_state.item(0),
                                                self.flight_state.item(1),
                                                self.flight_state.item(2),
                                                self.controlState)
        for i in range(P.num_steps):
            self.state = self.kinematics.PropagateDynamics(self.state, self.bodyForces)
            # self.update_simple_kin()
        self.eulerAttitude = Quaternion2Euler(self.state[6:10])
        self.euler_state = self.euler_states_array()
        self.view_thread.update(self.Output(), y_manager)
        self.view_thread.update_plot_data(self.EulerStates(), self.get_flight_state(), np.linalg.norm(self.va),
                                          self.wind[0:2],self.autopilot, self.sensor_values[3:8], self.kalman_attitude,
                                          self.kalman_loc, self.measured_gps, self.simple_kin_state)

        self.t_gps += P.ts_simulation
        self.t_plot += P.ts_simulation
        self.t_control += P.ts_simulation
        self.t_elapse += P.ts_simulation

    def update_simple_kin(self):
        state = np.array([[self.state.item(0)],
                          [self.state.item(1)],
                          [self.flight_state.item(4)],
                          [-self.state.item(2)],
                          [np.linalg.norm(self.va)]])
        u = np.array([[self.state.item(8)],
                      [self.autopilot.item(2)],
                      [self.autopilot.item(1)],
                      [self.autopilot.item(0)],
                      [self.wind.item(0)],
                      [self.wind.item(1)]])
        self.simple_kin_state = self.simple_kin.propagate_dynamics(state, u)

    def update_wind(self):
        """
        Update the state of the wind in the simulation
        :return:
        """
        self.wind = self.windsim.getWind(self.state)
        # self.wind = np.matrix([[0], [0], [0]])
        # self.va = np.subtract(self.state[3:6], self.wind)

    def control(self):
        """
        Update the control surface commandes to obtain the commanded autopilot values

        uses the controllers in the control file.  PID on the lateral control and total
        energy control on the longitudinal control
        :return:
        """
        [delta_a, delta_r] = self.lat_control.lateral_control_loop(self.euler_states_array(), self.autopilot.item(2), self.flight_state, phi_ff=self.phi_ff)
        [delta_e, delta_t] = self.lon_control.longitudinal_control_loop(self.euler_states_array(), self.flight_state, self.autopilot.item(1), self.autopilot.item(0))
        # measured_states = np.array([[self.measured_gps[0]], [self.measured_gps[1]], [-self.sensor_values[6]],
        #                            [0], [0], [0],
        #                            [self.kalman_attitude[0]], [self.kalman_attitude[1]], [0],
        #                            [self.sensor_values[3]], [self.sensor_values[4]], [self.sensor_values[5]]])
        # measured_flight_state = np.array([[self.sensor_values[7]], #va*
        #                                  [0], #alpha
        #                                  [0], #beta*
        #                                  [0], #path angle
        #                                  [self.measured_gps[3]]]) #course angle*
        # [delta_a, delta_r] = self.lat_control.lateral_control_loop(measured_states, self.autopilot.item(2),measured_flight_state)
        # [delta_e, delta_t] = self.lon_control.longitudinal_control_loop(measured_states, measured_flight_state, self.autopilot.item(1), self.autopilot.item(0))
        self.controlState.itemset(0, delta_a)
        self.controlState.itemset(1, delta_e)
        self.controlState.itemset(2, delta_r)
        self.controlState.itemset(3, delta_t)
        # print(self.controlState.T)

    def update_flight_state(self):
        self.va = np.subtract(self.state[3:6], self.wind)
        # print(np.linalg.norm(self.va))
        # va_ground = rotations.body_2_vehicle(self.va, phi, theta, psi)
        va_ground = rotations.body_2_inertial_quaternion(self.va, self.state[6:10])
        va_ground_mag = np.linalg.norm(va_ground)
        # wind_ground = rotations.body_2_vehicle(self.wind, phi, theta, psi)
        wind_ground = rotations.body_2_inertial_quaternion(self.wind, self.state[6:10])
        v_ground = va_ground + wind_ground
        # v_ground = va_ground

        v_ground_mag = np.linalg.norm(v_ground)
        ur = self.va.item(0)
        vr = self.va.item(1)
        wr = self.va.item(2)

        alpha = np.arctan(wr / ur)
        beta = np.arcsin(vr / va_ground_mag)
        course_angle_ground = np.arctan2(v_ground.item(1), v_ground.item(0))
        path_angle_ground = np.arcsin(v_ground.item(2) / np.linalg.norm(v_ground))
        self.flight_state = np.array([[v_ground_mag],
                                      [alpha],
                                      [beta],
                                      [path_angle_ground],
                                      [course_angle_ground]])

    def update_gps(self):
        self.measured_gps = self.gps.sensor_value([self.euler_state, self.wind, self.va])
        self.kalman_loc = self.loc_filter.update(np.array([self.sensor_values[7],
                                                           self.sensor_values[4],
                                                           self.sensor_values[5],
                                                           self.kalman_attitude[0],
                                                           self.kalman_attitude[1]]),
                                                 np.array([self.measured_gps[0],
                                                           self.measured_gps[1],
                                                           self.measured_gps[4],
                                                           self.measured_gps[3],
                                                           [1],
                                                           [1]]))

    def Output(self):
        """
        :return: The states that define the location and alltitude of the UAV in a list
        """
        x = self.state.item(0)
        y = self.state.item(1)
        z = self.state.item(2)
        roll = self.eulerAttitude.item(0)
        pitch = self.eulerAttitude.item(1)
        yaw = self.eulerAttitude.item(2)
        return [x, y, z, roll, pitch, yaw]

    def EulerStates(self):
        """

        :return: list of the current state of the UAV in euler states
        """
        set1 = self.state[0:6]
        # set2 = Quaternion2Euler(self.state[6:10])
        set2 = self.eulerAttitude
        set3 = self.state[10:13]
        ret = np.vstack((set1, set2, set3)).T.tolist()[0]
        # print(ret)
        return ret

    def euler_states_array(self):
        """
        Calculate the euler states for the UAV and return them in a numpy matrix
        :return: matrix of the current state of the UAV in euler states
        """
        set1 = self.state[0:6]
        # set2 = Quaternion2Euler(self.state[6:10])
        set2 = self.eulerAttitude
        set3 = self.state[10:13]
        ret = np.vstack((set1, set2, set3))
        return ret

    def get_flight_state(self):
        """
        :return: the actual flight state
         [va_mag, altitude, course_angle]
         of the uav.
        """
        flight_state = [self.flight_state.item(0), -self.state.item(2), self.flight_state.item(4)]
        return flight_state

if __name__ == "__main__":
    app = app = QApplication(sys.argv)
    uav = MAVModel()

    startTimer = QtCore.QTimer()
    startTimer.timeout.connect(uav.calculate_loop)
    # startTimer.setSingleShot(True)
    timers.append(startTimer)
    startTimer.start(0.01)
    # startTimer.start(P.ts_simulation)


    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
