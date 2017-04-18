"""
The model that will track the current state of the UAV.

This model is depricated, and is no longer in use.  The quaternion model is
the new supported model.
"""
import sys
import numpy as np


sys.path.append('..')
from Model.Dynamics import Dynamics
from Model.WindSimulation import WindSimulation
from Model.Forces import Forces
import ParamFiles.AerosondeParameters as P
from Control.lateral_controller import lateral_control as lat_con

class MAVModel:
    def __init__(self, chapter):
        self.chapter = chapter
        #   The current state of the UAV
        self.state = np.matrix([[P.Pn0],    #1
                                [P.Pe0],    #2
                                [P.Pd0],    #3
                                [P.u0],     #4
                                [P.v0],     #5
                                [P.w0],     #6
                                [P.phi0],   #7
                                [P.theta0], #8
                                [P.psi0],   #9
                                [P.p0],     #10
                                [P.q0],     #11
                                [P.r0]])    #12
        #   The forces that are applied to the body of the UAV
        self.bodyForces = np.matrix([[P.Fx],
                                    [P.Fy],
                                    [P.Fz],
                                    [P.Ml],
                                    [P.Mm],
                                    [P.Mn]])
        #   The state of the control surfaces
        self.controlState = np.matrix([[P.deltaA0],
                                       [P.deltaE0],
                                       [P.deltaR0],
                                       [P.deltaT0]])
        #   The commanded aircraft path following commands
        self.autopilot = np.matrix([[P.Va0],
                                    [P.altitude0],
                                    [P.heading0]])
        self.kinematics = Dynamics()
        self.wind = WindSimulation()
        self.forces = Forces()
        self.lat_control = lat_con()
        self.airspeed = np.matrix([[P.u0], [P.v0], [P.w0]])
        self.wind = np.matrix([[0], [0], [0]])

    def updateWind(self):
        """
        update the amount of wind that the MAV is experiencing
        :return:
        """
        # wind = self.wind.getWind(self.state)
        self.wind = np.matrix([[0],[0],[0]])
        self.airspeed = np.subtract(self.state[3:6], self.wind)

    def control(self):
        """
        update the control surfaces to obtain the commanded autopilot values

        uses the controllers in the control file. PID on the lateral control and total energy
        of the longitudinal control
        :return:
        """
        [delta_a, delta_r] = self.lat_control.lateral_control_loop(self.state, self.airspeed, self.wind, self.autopilot.item(2))
        self.controlState.itemset(0, delta_a)
        self.controlState.itemser(2, delta_r)

    def UpdateState(self):
        """
        Perform all necessary calculations to determine the next state of the uav

        uses that chapter value that was passed in on initialization to determine which actions
        need to be carried out.
        :return: nothing
        """

        if self.chapter >= 4:
            # Chapter 4: calculate forces based on the control surface positions
            VaMag = np.linalg.norm(self.airspeed)
            alpha = np.arctan(self.airspeed.item(2) / self.airspeed.item(0))
            beta = np.arcsin(self.airspeed.item(1) / VaMag)
            self.bodyForces = self.forces.getForces(self.state, VaMag, alpha, beta, self.controlState)

        if self.chapter >= 3:
            #   Chapter 3: Propagate the dynamics
            for i in range(5):
                self.state = self.kinematics.PropagateDynamics(self.state, self.bodyForces)

        #   Chapter 2: Do nothing
        return



    def Output(self):
        """
        :return: The states that define the location and alltitude of the UAV in a list
        """
        x = self.state.item(0)
        y = self.state.item(1)
        z = self.state.item(2)
        roll = self.state.item(6)
        pitch = self.state.item(7)
        yaw = self.state.item(8)
        return [x, y, z, roll, pitch, yaw]

    def States(self):
        """
        :return: All the current states of the UAV in a list
        """
        return self.state.T.tolist()[0]

    def EulerStates(self):
        return self.state.T.tolist()[0]

