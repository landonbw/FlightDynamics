import numpy as np
import sys

sys.path.append('..')
from Control.lateral_controller import saturate
import ParamFiles.AerosondeParameters as P

class longitudinal_control(object):
    def __init__(self):
        self.elevator_control = pitch_control()
        self.balance_control = energy_balance()
        self.energy_control = energy_control()

    def longitudinal_control_loop(self, state, flight_state, h_ref, va_ref):
        Pd = state.item(2)
        va_fs = flight_state.item(0)
        h = -Pd
        h_error = saturate(h_ref - h, P.h_error_max)
        U_error = P.mass * P.g * (h_error)
        K_error = (1/2) * P.mass * (va_ref**2 - va_fs**2)
        E = U_error + K_error
        B = U_error - K_error

        #get pitch command
        theta_c = self.balance_control.balance_pi_loop(B)
        #get elevator command
        delta_e = self.elevator_control.pitch_pd_loop(state, theta_c)
        # delta_e = -0.1
        #get throttle command
        delta_t = self.energy_control.energy_pi_loop(E)
        return [delta_e, delta_t]

class energy_balance(object):
    def __init__(self):
        self.kp = P.kp_bal
        self.ki = P.ki_bal
        self.integrator = 0.0
        self.B_d1 = 0.0

    def balance_pi_loop(self, B):
        self.integrator += (P.ts_control / 2.0) * (B + self.B_d1)
        self.B_d1 = B

        theta_c_unsat = self.kp * B + self.ki * self.integrator
        theta_c = saturate(theta_c_unsat, P.theta_max)

        #anti windup on the integrator
        self.integrator += (P.ts_control / self.ki) * (theta_c - theta_c_unsat)

        return theta_c

class pitch_control(object):
    def __init__(self):
        self.kp = P.kp_theta
        self.kd = P.kd_theta

    def pitch_pd_loop(self, state, theta_ref):
        theta = state.item(7)
        q = state.item(10)

        error = theta - theta_ref
        delta_e_prime = error * self.kp
        delta_e_unsat = delta_e_prime - self.kd * q
        delta_e = saturate(delta_e_unsat, P.delta_e_max)
        return delta_e

class energy_control(object):
    def __init__(self):
        self.kp = P.kp_throttle
        self.ki = P.ki_throttle
        self.integrator = 0.0
        self.e_d1 = 0.0

    def energy_pi_loop(self, E):
        self.integrator += (P.ts_control / 2.0) * (E + self.e_d1)
        self.e_d1 = E

        throttle_c_unsat = self.kp * E + self.ki * self.integrator
        if throttle_c_unsat < 0:
            throttle_c_unsat = 0.0
        throttle_c = saturate(throttle_c_unsat, P.delta_t_max)

        #anti windup on the integrator
        self.integrator += (P.ts_control / self.ki) * (throttle_c - throttle_c_unsat)

        return throttle_c




