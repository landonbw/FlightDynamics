import numpy as np
import sys
from math import cos, sin, atan2, asin, acos

sys.path.append('..')


def crossed_plane(plane_point, normal, loc):
    crossed = False
    point_diff = loc - plane_point
    dot_prod = np.dot(point_diff.reshape((1, 3)), normal)
    if dot_prod > 0:
        crossed = True
    return crossed

def rotz(theta):
    R = np.array([[cos(theta), -sin(theta), 0],
                  [sin(theta), cos(theta), 0],
                  [0, 0, 1]])
    return R

def mod2pi(phi):
    mod = phi%(2 * np.pi)
    return mod

class PathManager(object):
    def __init__(self):
        self.flag = True
        self.vg_des = 35
        self.r = np.array([[10], [0], [-10]])
        self.q = np.array([[1], [.1], [0]])
        self.c = np.array([[0], [400], [-10]])
        self.rho = 400
        self.direction = 1
        self.waypoints = []
        self.waypoint_counter = 0
        self.state = 1
        self.n_now = np.array([[0],[0],[0]])
        self.t_rad = 175

    def get_path(self, new_waypoints, pos):
        # return self.get_path_straight_line(new_waypoints, pos)
        # return self.get_path_fillet(new_waypoints, pos)
        return self.get_path_dubins(new_waypoints, pos)

    def get_path_straight_line(self, new_waypoints, pos):
        if new_waypoints != self.waypoints:
            self.waypoints = new_waypoints
            self.waypoint_counter = 1
        w_past = np.array([[self.waypoints[self.waypoint_counter-1][0]],
                      [self.waypoints[self.waypoint_counter-1][1]],
                      [self.waypoints[self.waypoint_counter-1][2]]])

        w_now = np.array([[self.waypoints[self.waypoint_counter][0]],
                      [self.waypoints[self.waypoint_counter][1]],
                      [self.waypoints[self.waypoint_counter][2]]])

        try:
            w_next = np.array([[self.waypoints[self.waypoint_counter+1][0]],
                          [self.waypoints[self.waypoint_counter+1][1]],
                          [self.waypoints[self.waypoint_counter+1][2]]])
        except:
            self.waypoint_counter -= 4
            w_next = np.array([[self.waypoints[self.waypoint_counter + 1][0]],
                               [self.waypoints[self.waypoint_counter + 1][1]],
                               [self.waypoints[self.waypoint_counter + 1][2]]])

        r = w_past

        q_past = (w_now-w_past) / np.linalg.norm(w_now-w_past)
        q_now = (w_next-w_now) / np.linalg.norm(w_next-w_now)
        n_now = (q_past+q_now) / np.linalg.norm(q_past+q_now)

        #check if we crossed the plane
        # point_diff = pos - w_now
        # dot_prod = np.dot(point_diff.reshape((1,3)), n_now)
        # if dot_prod > 0:
        if crossed_plane(w_now, n_now, pos):
            self.waypoint_counter += 1
        return [self.flag, self.vg_des, r, q_past, self.c, self.rho, self.direction]

    def get_path_fillet(self, new_waypoints, pos):
        c = -5
        lam = -5
        if new_waypoints != self.waypoints:
            self.waypoints = new_waypoints
            self.waypoint_counter = 1
            self.state = 1

        w_past = np.array([[self.waypoints[self.waypoint_counter - 1][0]],
                           [self.waypoints[self.waypoint_counter - 1][1]],
                           [self.waypoints[self.waypoint_counter - 1][2]]])

        w_now = np.array([[self.waypoints[self.waypoint_counter][0]],
                          [self.waypoints[self.waypoint_counter][1]],
                          [self.waypoints[self.waypoint_counter][2]]])

        try:
            w_next = np.array([[self.waypoints[self.waypoint_counter + 1][0]],
                               [self.waypoints[self.waypoint_counter + 1][1]],
                               [self.waypoints[self.waypoint_counter + 1][2]]])
        except:
            self.waypoint_counter -= len(self.waypoints)
            w_next = np.array([[self.waypoints[self.waypoint_counter + 1][0]],
                               [self.waypoints[self.waypoint_counter + 1][1]],
                               [self.waypoints[self.waypoint_counter + 1][2]]])

        q_past = (w_now - w_past) / np.linalg.norm(w_now - w_past)
        q_now = (w_next - w_now) / np.linalg.norm(w_next - w_now)

        phi_bar = np.arccos(np.dot(-q_past.reshape((1,3)), q_now))
        if self.state == 1:
            self.flag = True
            q = q_past
            z = w_now - (self.t_rad / (np.tan(phi_bar / 2))) * q_past
            if crossed_plane(z, q_past, pos):
                self.state = 2
        elif self.state == 2:
            self.flag = False
            c = w_now - (self.t_rad / np.sin(phi_bar / 2)) * ((q_past - q_now) / np.linalg.norm(q_past - q_now))
            lam = np.sign(q_past.item(0) * q_now.item(1) - q_past.item(1) * q_now.item(0))
            z = w_now + (self.t_rad / np.tan(phi_bar / 2)) * q_now

            if crossed_plane(z, q_now, pos):
                self.waypoint_counter += 1
                self.state = 1
        return [self.flag, self.vg_des, w_past, q_past, c, self.t_rad, lam]

    def get_path_dubins(self, new_waypoints, pos):
        assert (len(new_waypoints) >= 3), "insufficient waypoints"
        if new_waypoints != self.waypoints:
            self.waypoints = new_waypoints
            self.waypoint_counter = 1
            self.state = 1
        w_past = np.array([[self.waypoints[self.waypoint_counter - 1][0]],
                           [self.waypoints[self.waypoint_counter - 1][1]],
                           [self.waypoints[self.waypoint_counter - 1][2]]])
        chi_past = self.waypoints[self.waypoint_counter-1][3]

        w_now = np.array([[self.waypoints[self.waypoint_counter][0]],
                          [self.waypoints[self.waypoint_counter][1]],
                          [self.waypoints[self.waypoint_counter][2]]])
        chi_now = self.waypoints[self.waypoint_counter][3]

        try:
            w_next = np.array([[self.waypoints[self.waypoint_counter + 1][0]],
                               [self.waypoints[self.waypoint_counter + 1][1]],
                               [self.waypoints[self.waypoint_counter + 1][2]]])
            chi_next = self.waypoints[self.waypoint_counter + 1][3]
        except:
            self.waypoint_counter -= len(self.waypoints)
            w_next = np.array([[self.waypoints[self.waypoint_counter + 1][0]],
                               [self.waypoints[self.waypoint_counter + 1][1]],
                               [self.waypoints[self.waypoint_counter + 1][2]]])
            chi_next = self.waypoints[self.waypoint_counter + 1][3]

        [L, cs, lam_s, ce, lam_e, z1, q1, z2, z3, q3] = self.find_dubins_parameters(w_past, chi_past, w_now, chi_now)
        r = 'r'
        q = 'q'
        c = 'c'
        lam = 'lam'
        if self.state == 1:
            self.flag = False
            c = cs
            lam = lam_s
            if crossed_plane(z1, -q1, pos):
                self.state = 2
        if self.state == 2:
            self.flag = False
            c = cs
            lam = lam_s
            if crossed_plane(z1, q1, pos):
                self.state = 3
        if self.state == 3:
            self.flag = True
            r = z1
            q = q1
            if crossed_plane(z2, q1, pos):
                self.state = 4
        if self.state == 4:
            self.flag = False
            c = ce
            lam = lam_e
            if crossed_plane(z3, -q3, pos):
                self.state = 5
        if self.state == 5:
            self.flag = False
            c = ce
            lam = lam_e
            if crossed_plane(z3, q3, pos):
                self.state = 1
                self.waypoint_counter += 1

        return [self.flag, self.vg_des, r, q, c, self.t_rad, lam]



    def find_dubins_parameters(self, ps, chi_s, pe, chi_e):
        dist = np.linalg.norm(ps - pe)
        e1 = np.array([[1], [0], [0]])

        assert (dist >= 3 * self.t_rad), "waypoints are too close together!"
        # assert (self.fillet_rad > minTurnRad)
        crs = ps + self.t_rad * np.matmul(rotz(np.pi/2), np.array([[cos(chi_s)], [sin(chi_s)], [0]]))
        cls = ps + self.t_rad * np.matmul(rotz(-np.pi/2), np.array([[cos(chi_s)], [sin(chi_s)], [0]]))
        cre = pe + self.t_rad * np.matmul(rotz(np.pi/2), np.array([[cos(chi_e)], [sin(chi_e)], [0]]))
        cle = pe + self.t_rad * np.matmul(rotz(-np.pi/2), np.array([[cos(chi_e)], [sin(chi_e)], [0]]))

        #compute length for case 1 rsr
        ang = atan2(cre.item(1)-crs.item(1), cre.item(0)-crs.item(0))
        L1 = np.linalg.norm(crs-cre) + self.t_rad * mod2pi(2 * np.pi + mod2pi(ang - np.pi / 2) - mod2pi(chi_s - np.pi / 2)) \
             + self.t_rad * mod2pi(2 * np.pi + mod2pi(chi_e - np.pi / 2) - mod2pi(ang - np.pi / 2))

        # Compute length for case 2 rsl
        ang = atan2(cle.item(1)-crs.item(1), cle.item(0)-crs.item(0))
        l = np.linalg.norm(cle - crs)
        ang2 = ang - np.pi / 2 + asin((2 * self.t_rad) / l)
        L2 = np.sqrt(l ** 2 - 4 * self.t_rad ** 2) + self.t_rad * mod2pi(2 * np.pi + mod2pi(ang2) - mod2pi(chi_s - np.pi / 2)) \
             + self.t_rad * mod2pi(2 * np.pi + mod2pi(ang2 + np.pi) - mod2pi(chi_e + np.pi / 2))

        # Compute length for case 3 lsr
        ang = atan2(cre.item(1)-cls.item(1), cre.item(0)-cls.item(0))
        l = np.linalg.norm(cre-cls)
        ang2 = acos((2 * self.t_rad) / l)
        L3 = np.sqrt(l ** 2 - 4 * self.t_rad ** 2) + self.t_rad * mod2pi(2 * np.pi + mod2pi(chi_s + np.pi / 2) - mod2pi(ang + ang2)) \
             + self.t_rad * mod2pi(2 * np.pi + mod2pi(chi_e - np.pi / 2) - mod2pi(ang + ang2 - np.pi))

        # Compute length for case 4 lsl
        ang = atan2(cle.item(1)-cls.item(1), cle.item(0)-cls.item(0))
        L4 = np.linalg.norm(cls-cle) + self.t_rad * mod2pi(2 * np.pi + mod2pi(chi_s + np.pi / 2) - mod2pi(ang + np.pi / 2)) \
             + self.t_rad * mod2pi(2 * np.pi + mod2pi(ang + np.pi / 2) - mod2pi(chi_e + np.pi / 2))

        lengths = [L1, L2, L3, L4]
        if min(lengths) == L1:
            cs = crs
            lam_s = 1
            ce = cre
            lam_e = 1
            q1 = (ce - cs) / np.linalg.norm(ce - cs)
            z1 = cs + self.t_rad * np.matmul(rotz(-np.pi/2), q1)
            z2 = ce + self.t_rad * np.matmul(rotz(-np.pi/2), q1)

        elif min(lengths) == L2:
            cs = crs
            lam_s = 1
            ce = cle
            lam_e = -1
            l = np.linalg.norm(ce - cs)
            ang = atan2(ce.item(1) - cs.item(1), ce.item(0) - cs.item(0))
            ang2 = ang - np.pi/2 + asin((2 * self.t_rad) / l)
            q1 = np.matmul(rotz(ang2 + np.pi/2), e1)
            z1 = cs + self.t_rad * np.matmul(rotz(ang2), e1)
            z2 = ce + self.t_rad * np.matmul(rotz(ang2 + np.pi), e1)

        elif min(lengths) == L3:
            cs = cls
            lam_s = -1
            ce = cre
            lam_e = 1
            l = np.linalg.norm(ce - cs)
            ang = atan2(ce.item(1) - cs.item(1), ce.item(0) - cs.item(0))
            ang2 = acos((2 * self.t_rad) / l)
            q1 = np.matmul(rotz(ang + ang2 - np.pi/2), e1)
            z1 = cs + self.t_rad * np.matmul(rotz(ang + ang2), e1)
            z2 = ce + self.t_rad * np.matmul(rotz(ang + ang2 -np.pi), e1)

        # elif min(lengths) == L4:
        else:
            cs = cls
            lam_s = -1
            ce = cle
            lam_e = -1
            q1 = (ce - cs) / np.linalg.norm(ce - cs)
            z1 = cs + self.t_rad * np.matmul(rotz(np.pi/2), q1)
            z2 = ce + self.t_rad * np.matmul(rotz(np.pi/2), q1)

        z3 = pe
        q3 = np.matmul(rotz(chi_e), e1)

        return [min(lengths), cs, lam_s, ce, lam_e, z1, q1, z2, z3, q3]
