import numpy as np
import sys

sys.path.append('..')
import ParamFiles.AerosondeParameters as P
import Model.rotations as rotations
##########################################################################################
                    # unanswered questions
# Should the kp/kd values be calculated on every iteration through the control algorithm
# or should they be calculated around some initial value?

# what's up with the sign(a2) term in the book that is ommited in the slides?

# the ki_beta term is different between the book(105, 6.15) and the slides, what's up?

##########################################################################################
def saturate(control, max_control):
    if np.abs(control) > max_control:
        control = np.sign(control) * max_control
        # print('something saturated')
    return control

class lateral_control(object):
    def __init__(self):
        self.init = True
        self.phi_control = roll_control()
        self.chi_control = course_control()
        self.beta_control = sideslip_control()
        self.beta0 = 0.0
    def lateral_control_loop(self, state, commanded_course, flight_state, phi_ff=0.0):

        vg_fs = flight_state.item(0)
        beta_fs = flight_state.item(2)
        heading_fs = flight_state.item(4)


        phi_c = self.chi_control.course_pi_loop(state, commanded_course, heading_fs, phi_ff)
        delta_a = self.phi_control.roll_pd_loop(state, phi_c, vg_fs)
        delta_r = self.beta_control.sideslip_pi_loop(state, 0.0, beta_fs, vg_fs)
        return [delta_a, delta_r]


class roll_control(object):
    def __init__(self):
        self.a_phi1 = P.a_phi1
        self.a_phi2 = P.a_phi2
        self.zeta = P.zeta_phi
        self.omega_n = P.omega_n_phi
        self.kp = P.kp_phi
        self.kd = P.kd_phi
        self.derivative = 0.0

    def roll_pd_loop(self, state, phi_c, Va_mag):
        phi = state.item(6)
        p = state.item(9)

        phi_error = phi_c - phi
        delta_a_prime = phi_error * self.kp
        delta_a_unsat = delta_a_prime - self.kd * p

        # Saturate the delta a value
        delta_a = saturate(delta_a_unsat, P.delta_a_max)

        return delta_a


class course_control(object):
    def __init__(self):
        self.zeta = P.zeta_chi
        self.omega_n = P.omega_n_chi
        self.kp = P.kp_phi
        self.kd = P.kd_chi
        self.ki = P.ki_chi
        self.integrator = 0.0
        self.error_d1 = 0.0
        self.chi_dot = 0.0
        self.chi_d1 = P.psi0
        self.a = (2.0 * P.sigma - P.ts_control) / (2.0 * P.sigma + P.ts_control)
        self.a2 = 2.0 / (2.0 * P.sigma + P.ts_control)

    def course_pi_loop(self, state, chi_c, chi, phi_ff):
        error1 = chi_c - chi
        error2 = unit_circle_compliment(error1)
        error = error1 if abs(error1) <= abs(error2) else error2
        # print(error1, error2, error)

        self.integrator += (P.ts_control / 2.0) * (error + self.error_d1)
        self.chi_dot = self.a * self.chi_dot + self.a2 * (self.chi_d1 - chi)

        phi_c_unsat = self.kp * error + self.kd * self.chi_dot + self.ki * self.integrator + phi_ff
        self.error_d1 = error
        self.chi_d1 = chi
        phi_c = saturate(phi_c_unsat, P.phi_max)
        # anti windup on the integrator
        self.integrator = self.integrator + (P.ts_control / self.ki) * (phi_c - phi_c_unsat)

        return phi_c


class sideslip_control(object):
    def __init__(self):
        self.zeta = P.zeta_beta
        self.integrator = 0.0
        self.kp = P.kp_beta
        self.ki = P.ki_beta
        self.error_d1 = 0.0

    def sideslip_pi_loop(self, state, beta_c, beta, Va_mag):
        beta_c = 0.0

        error1 = beta_c - beta
        error2 = unit_circle_compliment(error1)
        error = error1 if abs(error1) <= abs(error2) else error2

        self.integrator += (P.ts_control / 2.0) * (error + self.error_d1)

        delta_r_unsat = self.kp * error - self.ki * self.integrator
        self.error_d1 = error
        delta_r = saturate(delta_r_unsat, P.delta_r_max)

        #anti windup on the integrator
        self.integrator = self.integrator + (P.ts_control / self.ki) * (delta_r - delta_r_unsat)

        return delta_r

def unit_circle_compliment(ang):
    """
    find the complimentary angle on the unit circle for a given angle.  For example
    the angle pi/2 is the same, on the unit circle, as the angle -3pi/2.  In this example
     -3pi/2 would be the compliment to pi/2.

     I'm using this function to find the compliment of the error, this allows the controller
     to calculate and push towards the smallest error value, because we don't actually care
     if we turn left or right to get to the desired course angle
    :param ang:
        the angle to find the unit circle compliment of.  The value passed in should be
        the angle in radians
    :return:
        The unit circle compliment of the angle that was passed in. the value returned will
        be in radians
    """
    ret = np.sign(ang) * (abs(ang) - 2.0 * np.pi)
    return ret



