import numpy as np
import sys
from math import sin, cos, atan2, sqrt
import random
from copy import deepcopy

sys.path.append('..')
import ParamFiles.AerosondeParameters as P
from CH12.PathManager import find_dubins_parameters


class RRT(object):
    def __init__(self, map, viewer, path_manager):
        self.map = map
        self.viewer = viewer
        self.path_manager = path_manager
        self.num_valid_paths = 0
        self.nodes = set()
        self.end_nodes = set()
        start_node = rrtNode(0, None, np.array([[P.Pn0], [P.Pe0], [P.Pd0]]), 0)
        self.nodes.add(start_node)
        self.path_elevation = P.Pd0
        self.end_node = rrtNode(0, None, np.array([[P.map_width], [P.map_width], [P.Pd0]]), np.radians(45), end_node=True)
        # self.find_path()

    def find_path(self):
        while self.num_valid_paths < 1:
        # while len(self.nodes) < 100:
            valid_flag = True
            # pick a random point
            x = random.random() * self.map.map_width
            y = random.random() * self.map.map_width
            print('loc generated')
            # find the nearest node
            nearest = None
            nearest_dist = self.map.map_width**3
            chi = 0
            for node in self.nodes:
                dist, ang = node.dist_to(x, y)
                if dist < nearest_dist and dist > 3*P.t_rad:
                    nearest_dist = dist
                    nearest = node
                    chi = ang

            if nearest is None:
                continue
            # take a step in the right direction
            if dist > P.step_size:
                x_test = nearest.pos.item(0) + cos(chi) * P.step_size
                y_test = nearest.pos.item(1) + sin(chi) * P.step_size
            else:
                x_test = x
                y_test = y
            pos_test = np.array([[x_test], [y_test], [self.path_elevation]])
            new_node = rrtNode(nearest.cost + nearest_dist, nearest, pos_test, chi+0.01)
            # get the dubins path between the two
            # self.path_manager.find_dubins_parameters(nearest.pos, nearest.chi, pos_test, chi)
            # discritize the path into points
            valid_flag = self.check_dubins_path(nearest, new_node)
            # if valid point add a new node
            if valid_flag:
                # add the point to valid nodes stack
                self.nodes.add(new_node)
                print('node added')
                # add the path to the map

                # self.viewer.draw_path(nearest.pos, new_node.pos)
                # find path to end node

                # check if path to end is valid
                to_end_valid = self.check_dubins_path(new_node, self.end_node)
                to_end_dist, to_end_ang = self.end_node.dist_to(x_test, y_test)
                # if valid add end node
                if to_end_valid and to_end_dist > 3*P.t_rad:
                    dist_end, ang_end = new_node.dist_to(self.end_node.pos.item(0), self.end_node.pos.item(1))
                    new_end = rrtNode(new_node.cost + dist_end, new_node, self.end_node.pos, self.end_node.chi, end_node=True)
                    self.end_nodes.add(new_end)
                    print('end found')
                    # self.viewer.draw_path(new_node.pos, new_end.pos)
                    # increment count
                    self.num_valid_paths += 1
        waypoints, points = self.find_shortest_path()
        print('found shortest path')
        self.viewer.draw_multisegment_path(points)
        print('drew things')
        return waypoints

    def check_path(self, start, end):
        valid_path = True
        path_points = points_along_line(start, end, 1)
        for point in path_points:
            if self.map.get_height(point.item(0), point.item(1)) >= -1 * self.path_elevation:
                valid_path = False
                break
        return valid_path

    def check_dubins_path(self, start_node, end_node):
        valid_path = True
        path_points = dubins_points(start_node, end_node, 1)
        for point in path_points:
            if self.map.get_height(point.item(0), point.item(1)) >= -1 * self.path_elevation:
                valid_path = False
                break
        return valid_path

    def find_shortest_path(self):
        node_path = []
        waypoint_path = []
        path_points = []
        shortest_node = None
        length = 99999999999999999999
        for node in self.end_nodes:
            if node.cost < length:
                shortest_node = node
                length = node.cost
            # print('checked end node')
        node_path.append(shortest_node)
        while node_path[-1].parent is not None:
            node_path.append(node_path[-1].parent)
            # print('added parent')
        node_path.reverse()
        # print('path reversed')
        node_path = self.smooth_path(node_path)
        # print('found smooth path')
        for node in node_path:
            waypoint_path.append([node.pos.item(0), node.pos.item(1), node.pos.item(2), node.chi])
            if len(waypoint_path) > 1:
                path_points += dubins_points(node_path[node_path.index(node)-1], node, 2)
        waypoint_path = np.asarray(waypoint_path)
        return waypoint_path, path_points

    def smooth_path(self, node_path):
        node_counter = 1
        smooth_path = [node_path[0]]
        end_reached = False
        while not end_reached:
            furthest_possible = node_counter
            for i in range(node_counter, len(node_path)):
                if self.check_path(smooth_path[-1].pos, node_path[i].pos):
                    furthest_possible = i
            smooth_path.append(node_path[furthest_possible])
            node_counter = furthest_possible + 1
            if self.check_dubins_path(smooth_path[-1], self.end_node):
                smooth_path.append(self.end_node)
            if smooth_path[-1].end_node:
                end_reached = True
        return smooth_path





class rrtNode(object):
    """
    Class to hold the nodes for the RRT path planning algorythm
    """
    def __init__(self, cost, parent, pos, chi, end_node=False):
        """
        :param cost: path distance to get to the node
        :param parent: previous node in the path chain
        :param orientation: (x, y, h, angle) the location/course angle of node
        """
        self.cost = cost
        self.parent = parent
        self.pos = pos
        self.chi = chi
        self.end_node = end_node

    def dist_to(self, x, y):
        """
        Calculate the distance from the node to an xy location
        elevation is assumed to be the same as the node
        :param x: x location
        :param y: y location
        :return: distance between the node and the location
        """
        node_x = self.pos[0]
        node_y = self.pos[1]
        dist = sqrt((node_x-x)**2 + (node_y-y)**2)
        ang = atan2(y-node_y, x-node_x)
        return dist, ang

def points_along_line(start_pos, end_pos, delta):
    """
    return points along a path separated by a distance delta
    :param start_pos: path start position
    :param end_pos: path end position
    :param delta: distance between each point
    :return: list points along the path
    """
    points = []
    q = end_pos - start_pos
    L = np.linalg.norm(q)
    q = q / L
    for i in range(int(L/delta) + 1):
        points.append(start_pos + i * delta * q)
    points.append(end_pos)
    return points

def points_along_circle(center, start_pos, end_pos, lam, delta):
    """
    Calculates points along a circular arc segment defined by a start, end and center
    position spaced out by a distance of delta
    :param center:
    :param start_pos:
    :param end_pos:
    :param delta:
    :return:
    """
    rad = np.linalg.norm(center-start_pos)
    start_ang = atan2(start_pos.item(1)-center.item(1), start_pos.item(0)-center.item(0))
    end_ang = atan2(end_pos.item(1)-center.item(1), end_pos.item(0)-center.item(0))
    # if start_ang < 0:
    #     start_ang += 2 * np.pi
    # if end_ang < 0:
    #     end_ang += 2 * np.pi
    theta_step = lam * delta / rad
    theta_travel = abs(end_ang - start_ang)
    theta_travel = max((end_ang, start_ang)) - min((end_ang, start_ang))
    # if lam < 0:
    #     theta_travel = 360 - theta_travel
    num_steps = int(theta_travel/theta_step)
    # if lam > 0:
    #     num_steps = int((end_ang-start_ang)/theta_step)
    # else:
    #     num_steps = int((start_ang-end_ang)/theta_step)
    points = []
    for i in range(num_steps):
        x = center.item(0) + rad * cos(start_ang + i * theta_step)
        y = center.item(1) + rad * sin(start_ang + i * theta_step)
        z = center.item(2)
        points.append(np.array([[x], [y], [z]]))
    points.append(np.array([[end_pos.item(0)], [end_pos.item(1)], [end_pos.item(2)]]))
    return points



def dubins_points(start, end, delta):
    """
    Find points along a dubins path
    :param start: rrtnode for the start point
    :param end: rrtnode for the end point
    :param delta: step size along the path
    :return: list of points
    """
    L, cs, lam_s, ce, lam_e, z1, q1, z2, z3, q3 = find_dubins_parameters(start.pos, start.chi, end.pos, end.chi)
    points = points_along_circle(cs, start.pos, z1, lam_s, delta)
    points += points_along_line(z1, z2, delta)
    points += points_along_circle(ce, z2, z3, lam_e, delta)
    return points




if __name__ == "__main__":
    # a = np.array([[0],[0],[0]])
    # b = np.array([[10],[10],[10]])
    # points = points_along_path(a, b, 0.25)
    # # print(points[-1])
    # for point in points:
    #     print(point)
    a = [1,2,3,4,5,6,7,8,9,10]
    for i in range(len(a)):
        for j in range(i, len(a)):
            print(j)
        print("new")