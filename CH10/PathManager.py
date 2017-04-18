import numpy as np
import sys

sys.path.append('..')

class PathManager(object):
    def __init__(self):
        self.flag = False
        self.vg_des = 45
        self.r = np.array([[10], [0], [-10]])
        self.q = np.array([[1], [.1], [0]])
        self.c = np.array([[0], [400], [-10]])
        self.rho = 400
        self.direction = 1
    def get_path(self):
        return [self.flag, self.vg_des, self.r, self.q, self.c, self.rho, self.direction]