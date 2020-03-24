"""
Benchmark Runner
"""
# pylint: disable=invalid-name

# Import Stuff
import re
import os
import sys
import time
import math
import Queue
import random
import marshal
import os.path
import pickledb
import cProfile 
import threading
import subprocess
import multiprocessing
from time import sleep
from pprint import pprint
from copy import deepcopy
from PyQt4 import QtGui, QtCore
from subprocess import STDOUT,PIPE


# For the program itself
TIMEOUT = 1800
INITIAL_RADIUS_RATIO = 1.5
STEP_SIZE = 0.1
GRID_SIDE_LENGTH = 4.0 / math.sqrt(3)
# For drawer
FONT_SIZE = 0.045
NODE_RADIUS = 1
VERTEX_RADIUS = 0.5
SPEED = 0.1
INTERVALS = 0.1
FPS = 60
STRETCH = 0.8
SCREEN_RESOLUTION = 0
SCREEN_WIDTH = 0
SCREEN_HEIGHT = 0

def closer(p1, p2):
    """
    Custom sorting function for vertices
    """
    if p1[1] < p2[1]:
        return -1
    return 1

def get_euclidean_dist(p1, p2):
    return math.sqrt((p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]))

def get_manhattan_dist(p1, p2):
    return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

def list_duplicates(seq):
    tally = dict()
    for i, item in enumerate(seq):
        item = marshal.dumps(item)
        if item not in tally:
            tally[item] = [i]
        else:
            tally[item].append(i)
    return ((key,locs) for key,locs in tally.items() if len(locs)>1)

class Problem(object):
    """
    The main class
    """
    def __init__(self):
        self.n = 2
        self.d = 0.0
        self.epsilon = 0.0
        self.starts = list()
        self.goals = list()
        self.grid_x_size = 5
        self.grid_y_size = 4
        self.r_b = 0.0
        self.path_start = list()
        self.path_goal = list()
        self.path_mid = list()
        self.grid_points = list()
        self.start_match = dict()
        self.goal_match = dict()
        self.total_path = list()

    def lower_bound(self):
        """
        Return solution lower bound
        """
        lb = 0.0
        for i in range(self.n):
            if lb < get_euclidean_dist(self.starts[i][1], (self.goals[i][1][0] + self.d, self.goals[i][1][1])):
                lb = get_euclidean_dist(self.starts[i][1], (self.goals[i][1][0] + self.d, self.goals[i][1][1]))
        return lb

    def post_process(self):
        """
        Deal with path
        """
        self.total_path = self.path_start + self.path_mid + self.path_goal
        total_length = self.d
        total_length += len(self.path_mid) * GRID_SIDE_LENGTH
        for the_list in [self.path_start, self.path_goal]:
            for i in range(len(the_list) - 1):
                unit_makespan = 0
                for j in range(self.n):
                    if get_euclidean_dist(the_list[i][j][1], the_list[i + 1][j][1]) > unit_makespan:
                        unit_makespan = get_euclidean_dist(the_list[i][j][1], the_list[i + 1][j][1])
                total_length += unit_makespan
        return total_length
    
    def draw_path(self):
        """
        Visualize
        """
        pass
        global FONT_SIZE
        global NODE_RADIUS
        global VERTEX_RADIUS
        global SPEED
        global INTERVALS
        global FPS
        global STRETCH
        global SCREEN_RESOLUTION
        global SCREEN_WIDTH
        global SCREEN_HEIGHT    
        app = QtGui.QApplication(sys.argv)
        SCREEN_RESOLUTION = app.desktop().screenGeometry()
        SCREEN_WIDTH = SCREEN_RESOLUTION.width()
        SCREEN_HEIGHT = SCREEN_RESOLUTION.height()
        view = MyView()
        view.showFullScreen()
        view.run_animation(app, self.grid_points, self.grid_x_size, self.grid_y_size, self.total_path)

    def linear_scaling(self):
        """
        Linear expansion and matching
        """
        # Generate grid
        x_step = (math.sqrt(3) / 2) * GRID_SIDE_LENGTH
        shortest_distance = self.r_b
        expand_ratio = 2 * GRID_SIDE_LENGTH / (2 + self.epsilon)
        self.grid_points = list()
        self.grid_x_size = int((2 * self.r_b * expand_ratio) / x_step)
        self.grid_y_size = int((2 * self.r_b * expand_ratio) / (3 * GRID_SIDE_LENGTH) * 2) + 1
        if self.grid_x_size < 5:
            self.grid_x_size = 5
        if self.grid_y_size < 4:
            self.grid_y_size = 4
        if self.grid_x_size % 2 != 1:
            self.grid_x_size += 1
        if self.grid_y_size % 2 != 0:
            self.grid_y_size += 1
        for i in range(self.grid_y_size):
            odd_even_1 = i % 2
            if i == 0:
                self.grid_points.append((- self.r_b * expand_ratio, -self.r_b * expand_ratio))
            else:
                self.grid_points.append((self.grid_points[-self.grid_x_size][0] + (
                    2 - odd_even_1) * GRID_SIDE_LENGTH, -self.r_b * expand_ratio))
            for j in range(1, self.grid_x_size):
                self.grid_points.append((self.grid_points[-j][0] + (j % 2) * (
                    odd_even_1 - 0.5) * GRID_SIDE_LENGTH, self.grid_points[-j][1] + j * x_step))
        # add points
        for i in range(3):
            self.path_start.append(deepcopy(self.starts))
            self.path_goal.append(deepcopy(self.goals))
        for the_list in [self.path_start[1], self.path_goal[1]]:
            for i, _garbage in enumerate(the_list):
                the_list[i] = (the_list[i][0], (
                    the_list[i][1][0] * expand_ratio, the_list[i][1][1] * expand_ratio))
        # Match vertices with robots
        self.start_match = dict()
        self.goal_match = dict()
        for robot in self.path_start[1]:
            shortest_distance = self.r_b
            for index2, point in enumerate(self.grid_points):
                if get_euclidean_dist(robot[1], point) < shortest_distance:
                    self.start_match[robot[0]] = index2
                    shortest_distance = get_euclidean_dist(robot[1], point)
        for index, _garbage in self.path_start[2]:
            self.path_start[2][index] = (self.path_start[2][index][0], self.grid_points[
                self.start_match[self.path_start[2][index][0]]])
        for robot in self.path_goal[1]:
            shortest_distance = self.r_b
            for index2, point in enumerate(self.grid_points):
                if get_euclidean_dist(robot[1], point) < shortest_distance:
                    self.goal_match[robot[0]] = index2
                    shortest_distance = get_euclidean_dist(robot[1], point)
        for index, _garbage in self.path_goal[0]:
            self.path_goal[0][index] = (self.path_goal[0][index][0], self.grid_points[
                self.goal_match[self.path_goal[0][index][0]]])

    def call_ILP(self, mode=16):
        """
        Solve the problem use the ILP solver
        """
        cmd = ['java', 'projects.multipath.ILP.Main', str(mode), str(
            self.grid_x_size), str(self.grid_y_size)]
        for i in range(self.n):
            s_temp = self.grid_points.index(self.path_start[2][i][1])
            s_temp = (s_temp / self.grid_x_size, s_temp % self.grid_x_size)
            cmd.append(str(s_temp[0] + s_temp[1] * self.grid_y_size))
            g_temp = self.grid_points.index(self.path_goal[0][i][1])
            g_temp = (g_temp / self.grid_x_size, g_temp % self.grid_x_size)
            cmd.append(str(g_temp[0] + g_temp[1] * self.grid_y_size)) 
        print cmd
        proc = subprocess.Popen(cmd, stdin=PIPE, stdout=PIPE, stderr=STDOUT)
        stdout, _stderr = proc.communicate()
        lines = stdout.splitlines()
        pointList = dict()
        for i, _garbage in enumerate(lines):
            lines[i] = str(lines[i])
            if len(lines[i]) <= 1:
                continue
            if lines[i][0] != 'A' or lines[i][1] != 'g':
                continue
            lines[i] = re.sub("[^0-9]", " ", lines[i])
            data = [int(s) for s in lines[i].split() if s.isdigit()]
            pointList[data[0]] = list()
            for j in range(3, len(data), 4):
                pointList[data[0]].append(data[j] * self.grid_x_size + data[j + 1])
        self.path_mid = [list() for i in range(len(pointList[0]))]
        for i, _garbage in enumerate(self.path_mid):
            for j in range(self.n):
                self.path_mid[i].append((
                    self.path_start[2][j][0], self.grid_points[pointList[j][i]]))

    def call_SAG(self):
        """
        Solve the problem use the ILP solver
        """
        # Shrink the problem, first get the targer grid size
        x_range = 5
        y_range = 4
        while x_range * y_range < self.n:
            if x_range < y_range:
                x_range += 2
            else:
                y_range += 2
        # Then extract target locations
        new_corner_x = (self.grid_x_size - x_range) / 2
        new_corner_y = (self.grid_y_size - y_range) / 2
        if new_corner_x % 2 != new_corner_y % 2:
            new_corner_y += 1
        new_vertices = list()
        for i in range(x_range):
            for j in range(y_range):
                new_vertices.append((new_corner_x + i) + (new_corner_y + j) * self.grid_x_size)
                new_vertices.append((new_corner_x + i) + (new_corner_y + j) * self.grid_x_size)
        # Start
        cmd_s = ['java', 'projects.formation.system.Main', '0', str(self.grid_x_size), str(self.grid_y_size)]
        cmd_g = ['java', 'projects.formation.system.Main', '0', str(self.grid_x_size), str(self.grid_y_size)]
        for i in range(self.n):
            cmd_s.append(str(self.start_match[i]))
            cmd_s.append(str(new_vertices.pop(0))) 
            cmd_g.append(str(self.goal_match[i]))
            cmd_g.append(str(new_vertices.pop(0))) 
        st = float(time.clock())
        proc = subprocess.Popen(cmd_s, stdin=PIPE, stdout=PIPE, stderr=STDOUT, cwd="formation\\source\\")
        stdout, _stderr = proc.communicate()
        lines = [line.split() for line in stdout.split("\n")]
        lines.pop(-1)
        path_prior = list()
        zeros = [list() for line in lines]
        matches = dict()
        for i in range(self.n):
            for j in range(self.n):
                if self.start_match[i] == int(lines[j][0]):
                    matches[j] = i
                    break
        first = True
        while lines != zeros:
            if first:
                path_prior.append(deepcopy(self.starts))
                first = False
            else:
                path_prior.append(deepcopy(path_prior[-1]))
            occupied = list()
            for i in range(self.n):
                j = matches[i]
                if len(lines[i]) > 0:
                    if lines[i][0] not in occupied:
                        path_prior[-1][j] = (j, int(lines[i][0]))
                        occupied.append(lines[i][0])
                        lines[i].pop(0)
                        continue
                occupied.append(path_prior[-1][j][1])
        proc = subprocess.Popen(cmd_g, stdin=PIPE, stdout=PIPE, stderr=STDOUT, cwd="formation\\source\\")
        stdout, _stderr = proc.communicate()
        lines = [line.split() for line in stdout.split("\n")]
        lines.pop(-1)
        matches = dict()
        path_post = list()
        for i in range(self.n):
            for j in range(self.n):
                if self.goal_match[i] == int(lines[j][0]):
                    matches[j] = i
                    break
        first = True
        while lines != zeros:
            if first:
                path_post.append(deepcopy(self.starts))
                first = False
            else:
                path_post.append(deepcopy(path_post[-1]))
            occupied = list()
            for i in range(self.n):
                j = matches[i]
                if len(lines[i]) > 0:
                    if lines[i][0] not in occupied:
                        path_post[-1][j] = (j, int(lines[i][0]))
                        occupied.append(lines[i][0])
                        lines[i].pop(0)
                        continue
                occupied.append(path_prior[-1][j][1])
        path_post = list(reversed(path_post))
        for i in range(self.n):
            self.start_match[i] = path_prior[-1][i][1] 
            self.goal_match[i] = path_post[0][i][1] 
        # Solve it.
        cmd = ['python', 'grid_solver.py', str(x_range), str(y_range)]
        for i in range(self.n):
            s_temp_x = self.start_match[i] % self.grid_x_size
            s_temp_y = self.start_match[i] / self.grid_x_size
            s_temp = (s_temp_x - new_corner_x) + (s_temp_y - new_corner_y) * x_range
            cmd.append(str(s_temp))
            g_temp_x = self.goal_match[i] % self.grid_x_size
            g_temp_y = self.goal_match[i] / self.grid_x_size
            g_temp = (g_temp_x - new_corner_x) + (g_temp_y - new_corner_y) * x_range
            cmd.append(str(g_temp)) 
        proc = subprocess.Popen(cmd, stdin=PIPE, stdout=PIPE, stderr=STDOUT)
        stdout, _stderr = proc.communicate()
        lines = [line.split() for line in stdout.split("\n")]
        lines.pop(-1)
        self.path_mid = list()
        for line in lines:
            self.path_mid.append(list())
            for i in range(self.n):
                vertex = int(line[i])
                vertex_x = vertex % x_range + new_corner_x
                vertex_y = vertex / x_range + new_corner_y
                self.path_mid[-1].append((i, vertex_x + vertex_y * self.grid_x_size))
        # while True:
        #     dup = sorted(list_duplicates(self.path_mid))
        #     if len(dup) == 0:
        #         break
        #     else:
        #         largest = 0
        #         index = 0
        #         for i, d in enumerate(dup):
        #             if d[1][-1] - d[1][0] > largest:
        #                 largest = d[1][-1] - d[1][0]
        #                 index = i
        #         self.path_mid = self.path_mid[:dup[index][1][0]] + self.path_mid[dup[index][1][-1]:]
        self.path_mid = path_prior + self.path_mid + path_post
        for index, line in enumerate(self.path_mid):
            for i in range(self.n):
                self.path_mid[index][i] = (i, self.grid_points[line[i][1]])

    def generate_new_problem(self):
        """
        Randomly select start and goal configurations
        """
        # Reset parameters
        self.grid_x_size = 5
        self.grid_y_size = 4
        self.r_b = 0.0
        self.starts = list()
        self.goals = list()
        self.path_start = list()
        self.path_goal = list()
        self.path_mid = list()
        # First get the right bounding size
        radius_file = open("pack_new.txt", "r")
        lines = radius_file.readlines()
        self.r_b = 1 / float(lines[self.n - 1]) * INITIAL_RADIUS_RATIO * (1 + self.epsilon / 2)
        # Next, generate starts and goals
        while True:
            if self.generate_configs(self.starts):
                break
        while True:
            if self.generate_configs(self.goals):
                break

    def refresh_problem(self):
        self.path_start = list()
        self.path_goal = list()
        self.path_mid = list()        

    def generate_configs(self, the_list):
        """
        Subroutine for generate a configuration
        """
        del the_list[:]
        r_range = self.r_b - 1
        fail_counter = 0
        i = 0
        while i < self.n:
            if fail_counter > 5 * self.n:
                return False
            fail = False
            new_config = (random.random() * 2 * r_range - r_range, random.random() * 2 * r_range - r_range)
            if get_euclidean_dist(new_config, (0, 0)) > r_range:
                fail_counter += 1
                continue
            for robot in the_list:
                if get_manhattan_dist(new_config, robot[1]) < (2 + self.epsilon) * 2:
                    if get_euclidean_dist(new_config, robot[1]) < 2 + self.epsilon:
                        fail = True
                        fail_counter += 1
                        break
            if not fail:
                the_list.append((i, new_config))
                i += 1
        return True


class Node(QtGui.QGraphicsEllipseItem):
    """
    One robot
    """
    def __init__(self, text, init_location, mode):
        # It is a vertex
        if mode == 0:
            super(Node, self).__init__(
                init_location[0] - VERTEX_RADIUS / 2, init_location[1] - VERTEX_RADIUS / 2, VERTEX_RADIUS, VERTEX_RADIUS)
            self.setZValue(1)   # Drawn on a higher layer
            self.setBrush(QtCore.Qt.gray)
        elif mode == 1:
            super(Node, self).__init__(- NODE_RADIUS / 2, - NODE_RADIUS / 2, NODE_RADIUS, NODE_RADIUS)
            self.text = QtGui.QGraphicsTextItem(str(text))
            self.text.setScale(FONT_SIZE)
            self.text_x = self.text.boundingRect().width() / 2 * FONT_SIZE
            self.text_y = self.text.boundingRect().height() / 2 * FONT_SIZE
            self.text.setPos(init_location[0] - self.text_x, init_location[1] - self.text_y)
            self.text.setZValue(4)
            self.setZValue(2)   # Drawn on a higher layer
            self.setBrush(QtGui.QBrush(QtGui.QColor("#000000"), style = QtCore.Qt.SolidPattern))
            self.front = QtGui.QGraphicsEllipseItem(
                - NODE_RADIUS / 2 * 0.9, - NODE_RADIUS / 2 * 0.9, NODE_RADIUS * 0.9, NODE_RADIUS * 0.9)
            self.front.setBrush(QtGui.QBrush(QtGui.QColor("#42cbf4"), style = QtCore.Qt.SolidPattern))
            self.front.setZValue(3)

    def move(self, new_location):
        """
        Change the location of the node
        """
        self.setPos(new_location[0], new_location[1])
        self.front.setPos(new_location[0], new_location[1])
        self.text.setPos(new_location[0] - self.text_x, new_location[1] - self.text_y)

class MyView(QtGui.QGraphicsView):
    """
    A view
    """
    def __init__(self):
        QtGui.QGraphicsView.__init__(self)
        self.scene = QtGui.QGraphicsScene(self)
        self.setScene(self.scene)
        x_size = 0
        y_size = 0
        self.num_robot = 0
        grid_vertices = list()
        self.paths = list()
        self.nodes = list()

    def run_animation(self, app, grid_vertices, x_size, y_size, paths_raw):
        """
        Run the animation in a smooth way
        """
        self.num_robot = len(paths_raw[0])
        # Find border
        x_border = 0
        y_border = 0
        for vertex in grid_vertices:
            if abs(vertex[0]) > x_border:
                x_border = abs(vertex[0])
            if abs(vertex[1]) > y_border:
                y_border = abs(vertex[1])
        # Find best stretch
        x_stretch = (float(SCREEN_WIDTH) / 2) / x_border
        y_stretch = (float(SCREEN_HEIGHT) / 2) / y_border
        real_stretch = min(x_stretch, y_stretch) * STRETCH
        # Stretch everything
        for i, _garbage in enumerate(grid_vertices):
            grid_vertices[i] = [j * real_stretch for j in grid_vertices[i]]
        for i, _garbage in enumerate(paths_raw):
            for j in range(self.num_robot):
                paths_raw[i][j] = (paths_raw[i][j][0], (
                    paths_raw[i][j][1][0] * real_stretch, paths_raw[i][j][1][1] * real_stretch))
        global NODE_RADIUS, VERTEX_RADIUS, FONT_SIZE
        NODE_RADIUS = NODE_RADIUS * real_stretch * 2
        VERTEX_RADIUS = VERTEX_RADIUS * real_stretch * 2
        FONT_SIZE = FONT_SIZE * real_stretch * 2
        # Postprocess paths
        global SPEED
        SPEED = SPEED * math.sqrt(math.pow(grid_vertices[0][0] - grid_vertices[1][0], 2) + math.pow(
            grid_vertices[0][1] - grid_vertices[1][1], 2))
        for i in range(len(paths_raw) - 1):
            for j in range(int(INTERVALS * FPS)):
                self.paths.append(paths_raw[i])
            finished = False
            while not finished:
                finished = True
                temp_config = list()
                for j in range(self.num_robot):
                    x_dist = paths_raw[i + 1][j][1][0] - self.paths[-1][j][1][0]
                    y_dist = paths_raw[i + 1][j][1][1] - self.paths[-1][j][1][1]
                    total_dist = math.sqrt(x_dist * x_dist + y_dist * y_dist)
                    if total_dist > SPEED:
                        x_increment = x_dist / total_dist * SPEED
                        y_increment = y_dist / total_dist * SPEED
                        temp_config.append((self.paths[-1][j][0], (
                            self.paths[-1][j][1][0] + x_increment, self.paths[-1][j][1][1] + y_increment)))
                    else:
                        temp_config.append(paths_raw[i + 1][j])
                    if x_dist != 0.0 or y_dist != 0.0:
                        finished = False
                self.paths.append(temp_config)
        for i in range(300):
            self.paths.append(paths_raw[-1])
        # Add vertices
        for i in range(len(grid_vertices)):
            self.scene.addItem(Node(i, grid_vertices[i], 0))
        # Add edges
        line_pen = QtGui.QPen(QtCore.Qt.gray)
        line_pen.setWidth(VERTEX_RADIUS / 3)
        for j in range(y_size):
            for i in range(x_size - 1):
                line = QtGui.QGraphicsLineItem(QtCore.QLineF(
                    grid_vertices[j * x_size + i][0], grid_vertices[j * x_size + i][1], 
                    grid_vertices[j * x_size + i + 1][0], grid_vertices[j * x_size + i + 1][1]))
                line.setZValue(0)
                line.setPen(line_pen)
                self.scene.addItem(line)
            if j < y_size - 1:
                for i in range(j % 2, x_size, 2):
                    line = QtGui.QGraphicsLineItem(QtCore.QLineF(
                        grid_vertices[j * x_size + i][0], grid_vertices[j * x_size + i][1], 
                        grid_vertices[i + (j + 1) * x_size][0], grid_vertices[i + (j + 1) * x_size][1]))
                    line.setZValue(0)
                    line.setPen(line_pen)
                    self.scene.addItem(line)               
        # Add nodes
        for i in range(self.num_robot):
            self.nodes.append(Node(str(i), self.paths[0][i][1], 1))
            self.scene.addItem(self.nodes[-1])
            self.scene.addItem(self.nodes[-1].text)
            self.scene.addItem(self.nodes[-1].front)
        # Follow paths
        for config in self.paths:
            for i in range(self.num_robot):
                self.nodes[i].move(config[i][1])
            app.processEvents()
            sleep(1.0 / float(FPS))

        NODE_RADIUS = NODE_RADIUS / (real_stretch * 2)
        VERTEX_RADIUS = VERTEX_RADIUS / (real_stretch * 2)
        FONT_SIZE = FONT_SIZE / (real_stretch * 2)
        SPEED = SPEED / math.sqrt(math.pow(grid_vertices[0][0] - grid_vertices[1][0], 2) + math.pow(
            grid_vertices[0][1] - grid_vertices[1][1], 2))

if __name__ == "__main__":
    # cases = list()
    # for i in range(100):
    #     cases.append((i, 0.0, 0.0))
    # for i in range(21):
    #     cases.append((20, i * 0.01, 0.0))
    # for i in range(101):
    #     cases.append((20, 0.0, i * 1.0))
    # IN_FILE = open("result.txt", "r")
    # lines = IN_FILE.readlines()
    # case_index = 0
    # try:
    #     case_index = cases.index(int(lines[-1][0]), float(lines[-1][0]), float(lines[-1][2]))
    # except ValueError:
    #     pass
    # IN_FILE.close()
    # OUT_FILE = open("result.txt", "a")
    # for i in range(case_index, len(cases)):
    #     print "Test case: " + str(cases[i])



    # IN_FILE.write('{:<15}'.format("N") + '{:<15}'.format("EPSILON") + '{:<15}'.format("D") +\
    # '{:<15}'.format("ILP_TIME") + '{:<15}'.format("ILP_MAKESPAN") + \
    # '{:<15}'.format("SAG_TIME") + '{:<15}'.format("SAG_MAKESPAN") + "\n")
    PROB = Problem()
    PROB.n = 10
    PROB.epsilon = 0
    PROB.generate_new_problem()
    # PROB.linear_scaling()
    # PROB.call_SAG()
    # print PROB.post_process()
    # print len(PROB.path_mid)
    # PROB.draw_path() 
    PROB.refresh_problem()
    PROB.linear_scaling()
    PROB.call_ILP()
    print PROB.post_process()
    print len(PROB.path_mid)
    PROB.draw_path() 
    # print PROB.post_process()
    # PROB.refresh_problem()
    # PROB.linear_scaling()
    # PROB.call_SAG()
    # print PROB.post_process()
    # PROB.draw_path()


        # edges = list()
        # # Add horizontal edges
        # for i in range(self.grid_x_size - 1):
        #     for j in range(self.grid_y_size):
        #         edges.append(((i + j * self.grid_x_size), (i + 1 + j * self.grid_x_size)))
        #         edges.append(((i + 1 + j * self.grid_x_size), (i + j * self.grid_x_size)))
        # # Add vertical edges
        # for i in range(self.grid_x_size):
        #     for j in range(self.grid_y_size - 1):
        #         if i % 2 == j % 2:    
        #             edges.append(((i + j * self.grid_x_size), (i + self.grid_x_size + j * self.grid_x_size)))
        #             edges.append(((i + self.grid_x_size + j * self.grid_x_size), (i + j * self.grid_x_size)))
        # # Find mid point
        # mid_x = self.grid_x_size / 2
        # mid_y = self.grid_y_size / 2
        # mid_pt = (mid_x, mid_y)
        # shrink_complete = False
        # current_match = deepcopy(self.start_match)
        # while not shrink_complete:
        #     shrink_complete = True
        #     occupied = dict()
        #     for i in range(len(self.grid_points)):
        #         occupied[i] = False
        #     # Sort
        #     info = current_match.items()
        #     for index, _garbage in enumerate(info):
        #         occupied[info[index][1]] = True
        #         info[index] = (info[index][0], get_manhattan_dist((info[index][1] % self.grid_x_size, info[index][1] / self.grid_x_size), mid_pt))
        #     info = sorted(info, cmp=closer)
        #     for ide, _garbage in info:
        #         if current_match[ide] % self.grid_x_size > mid_x:
        #             if (current_match[ide], current_match[ide] - 1) in edges:
        #                 if not occupied[current_match[ide] - 1]:               
        #                     occupied[current_match[ide]] = False
        #                     occupied[current_match[ide] - 1] = True
        #                     current_match[ide] = current_match[ide] - 1
        #                     shrink_complete = False
        #                     continue
        #         if current_match[ide] % self.grid_x_size < mid_x:
        #             if (current_match[ide], current_match[ide] + 1) in edges:
        #                 if not occupied[current_match[ide] + 1]:                     
        #                     occupied[current_match[ide]] = False
        #                     occupied[current_match[ide] + 1] = True
        #                     current_match[ide] = current_match[ide] + 1
        #                     shrink_complete = False
        #                     continue  
        #         if current_match[ide] / self.grid_x_size > mid_y:
        #             if (current_match[ide], current_match[ide] - self.grid_x_size) in edges:
        #                 if not occupied[current_match[ide] - self.grid_x_size]:                        
        #                     occupied[current_match[ide]] = False
        #                     occupied[current_match[ide] - self.grid_x_size] = True
        #                     current_match[ide] = current_match[ide] - self.grid_x_size                         
        #                     shrink_complete = False
        #                     continue
        #         if current_match[ide] / self.grid_x_size < mid_y:
        #             if (current_match[ide], current_match[ide] + self.grid_x_size) in edges:
        #                 if not occupied[current_match[ide] + self.grid_x_size]:                    
        #                     occupied[current_match[ide]] = False
        #                     occupied[current_match[ide] + self.grid_x_size] = True
        #                     current_match[ide] = current_match[ide] + self.grid_x_size                           
        #                     shrink_complete = False
        #                     continue                  
        #     self.path_start.append(deepcopy(self.path_start[-1]))
        #     for ide, vertex in current_match.items():
        #         self.path_start[-1][ide] = (ide, self.grid_points[vertex])
        # self.start_match = current_match
        # # Deal with goal
        # shrink_complete = False
        # current_match = deepcopy(self.goal_match)
        # while not shrink_complete:
        #     shrink_complete = True
        #     occupied = dict()
        #     for i in range(len(self.grid_points)):
        #         occupied[i] = False
        #     # Sort
        #     info = current_match.items()
        #     for index, _garbage in enumerate(info):
        #         occupied[info[index][1]] = True
        #         info[index] = (info[index][0], get_manhattan_dist((info[index][1] % self.grid_x_size, info[index][1] / self.grid_x_size), mid_pt))
        #     info = sorted(info, cmp=closer)
        #     for ide, _garbage in info:
        #         if current_match[ide] % self.grid_x_size > mid_x:
        #             if (current_match[ide], current_match[ide] - 1) in edges:
        #                 if not occupied[current_match[ide] - 1]:               
        #                     occupied[current_match[ide]] = False
        #                     occupied[current_match[ide] - 1] = True
        #                     current_match[ide] = current_match[ide] - 1
        #                     shrink_complete = False
        #                     continue
        #         if current_match[ide] % self.grid_x_size < mid_x:
        #             if (current_match[ide], current_match[ide] + 1) in edges:
        #                 if not occupied[current_match[ide] + 1]:                     
        #                     occupied[current_match[ide]] = False
        #                     occupied[current_match[ide] + 1] = True
        #                     current_match[ide] = current_match[ide] + 1
        #                     shrink_complete = False
        #                     continue  
        #         if current_match[ide] / self.grid_x_size > mid_y:
        #             if (current_match[ide], current_match[ide] - self.grid_x_size) in edges:
        #                 if not occupied[current_match[ide] - self.grid_x_size]:                        
        #                     occupied[current_match[ide]] = False
        #                     occupied[current_match[ide] - self.grid_x_size] = True
        #                     current_match[ide] = current_match[ide] - self.grid_x_size                         
        #                     shrink_complete = False
        #                     continue
        #         if current_match[ide] / self.grid_x_size < mid_y:
        #             if (current_match[ide], current_match[ide] + self.grid_x_size) in edges:
        #                 if not occupied[current_match[ide] + self.grid_x_size]:                    
        #                     occupied[current_match[ide]] = False
        #                     occupied[current_match[ide] + self.grid_x_size] = True
        #                     current_match[ide] = current_match[ide] + self.grid_x_size                           
        #                     shrink_complete = False
        #                     continue    
        #     self.path_goal.insert(0, deepcopy(self.path_goal[0]))
        #     for ide, vertex in current_match.items():
        #         self.path_goal[0][ide] = (ide, self.grid_points[vertex])
        # self.goal_match = current_match
        # x_max = 0
        # y_max = 0
        # x_min = self.grid_x_size
        # y_min = self.grid_y_size
        # for _garbage, vertex in self.start_match.items() + self.goal_match.items():
        #     if vertex % self.grid_x_size > x_max:
        #         x_max = vertex % self.grid_x_size
        #     if vertex % self.grid_x_size < x_min:
        #         x_min = vertex % self.grid_x_size
        #     if vertex / self.grid_x_size > y_max:
        #         y_max = vertex / self.grid_x_size
        #     if vertex / self.grid_x_size < y_min:
        #         y_min = vertex / self.grid_x_size
        # x_range = x_max - x_min + 1
        # y_range = y_max - y_min + 1
        # if x_range < 5:
        #     x_range = 5
        # if y_range < 4:
        #     y_range = 4
        # if x_range % 2 != 1:
        #     x_range += 1
        # if y_range % 2 != 0:
        #     y_range += 1  
        # print self.grid_x_size, self.grid_y_size, x_range, y_range