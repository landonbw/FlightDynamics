import numpy as np
import sys

sys.path.append('..')
import ParamFiles.AerosondeParameters as P

class PathPlanner(object):
    def __init__(self):
        pass

    def get_waypoints(self):
        waypoints = [[0, 0, -200, 0, P.Va0],
                     [1200, 0, -200, np.radians(45), P.Va0],
                     [0, 1200, -200, np.radians(45), P.Va0],
                     [1200, 1200, -200, np.radians(-135), P.Va0]]
        return waypoints