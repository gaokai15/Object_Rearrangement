from __future__ import print_function
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from klampt.plan.cspace import CSpace, MotionPlan
from klampt.vis.glprogram import GLProgram
from klampt.math import vectorops

import sys
import json
from time import time
from itertools import combinations
from random import random, seed, choice

import Polygon as pn
import pyclipper as pc

from util import *

num_buffers = 0
EPSILON = 1


class Circle:
    def __str__(self):
        return 'Circle(x={}, y={}, radius={})'.format(self.center[0], self.center[1], self.radius)

    def __init__(self, x=0, y=0, radius=1):
        self.center = (x, y)
        self.radius = radius
        self.type = 'Circle'

    def contains(self, point):
        return (vectorops.distance(point, self.center) <= self.radius)

    def poly(self, numdivs=12, toint=True):
        pnts = []
        # numdivs = int(ceil(self.radius * pi * 2 / res))
        for i in xrange(numdivs + 1):
            u = float(i) / float(numdivs) * pi * 2
            if toint:
                pnts.append((
                    int(self.center[0] + self.radius * cos(u)),
                    int(self.center[1] + self.radius * sin(u)),
                ))
            else:
                pnts.append((
                    self.center[0] + self.radius * cos(u),
                    self.center[1] + self.radius * sin(u),
                ))
        return pnts

    def drawGL(self, numdivs=12):
        glBegin(GL_TRIANGLE_FAN)
        glVertex2f(*self.center)
        for p in self.poly(numdivs, False):
            glVertex2f(*p)
        glEnd()


class Rectangle:
    def __str__(self):
        return 'Rectangle(x={}, y={}, width={}, height={})'.format(
            self.center[0], self.center[1], self.width, self.height
        )

    def __init__(self, x=0, y=0, width=1, height=1):
        self.center = (x, y)
        self.width = width
        self.height = height
        self.type = 'Rectangle'

    def contains(self, point):
        temp = vectorops.sub(point, self.center)
        return temp[0] >= 0 and temp[1] >= 0 and temp[0] <= self.width and temp[1] <= self.height

    def poly(self):
        return [
            # (self.center[0], self.center[1]),
            # (self.center[0], self.center[1] + self.height),
            # (self.center[0] + self.width, self.center[1] + self.height),
            # (self.center[0] + self.width, self.center[1]),
            (self.center[0], self.center[1]),
            (self.center[0] + self.width, self.center[1]),
            (self.center[0] + self.width, self.center[1] + self.height),
            (self.center[0], self.center[1] + self.height),
        ]

    def drawGL(self):
        glBegin(GL_QUADS)
        for p in self.poly():
            glVertex2f(*p)
        glEnd()


class Poly:
    def __str__(self):
        return 'Poly({})'.format(self.points)

    def __init__(self, points):
        if points:
            if type(points[0][0]) == list:
                self.points = list(sorted(points, key=lambda x: abs(pc.Area(x))))
                self.type = 'C_Poly'
            else:
                self.type = 'S_Poly'
        else:
            self.type = 'Empty'
        self.points = points

    def contains(self, point):
        if self.type == 'C_Poly':
            c = 0
            for cont in self.points:
                if pc.PointInPolygon(point, cont):
                    if pc.Orientation(cont):
                        c += 1
                    else:
                        c -= 1
            return c > 0
        elif self.type == 'S_Poly':
            return bool(pc.PointInPolygon(point, self.points))

    def quasiCenter(self):
        center = (0, 0)
        if self.type == 'C_Poly':
            poly = pn.Polygon()
            for cont in self.points:
                if pc.Orientation(cont):
                    poly += pn.Polygon(cont)
                else:
                    poly -= pn.Polygon(cont)
            if poly.isInside(*poly.center()) > 0:
                center = poly.center()
            else:
                center = poly.sample(o55)
        elif self.type == 'S_Poly':
            poly = pn.Polygon(self.points)
            if pc.PointInPolygon(poly.center(), self.points) > 0:
                center = poly.center()
            else:
                center = poly.sample(o55)
        return (int(center[0]), int(center[1]))

    def sample(self, reachable_to=None):
        if self.type == 'C_Poly':
            poly = pn.Polygon()
            for cont in self.points:
                if pc.Orientation(cont):
                    if reachable_to is None or pc.PointInPolygon(reachable_to, cont):
                        poly += pn.Polygon(cont)
                else:
                    poly -= pn.Polygon(cont)
            if poly:
                return poly.sample(random)
        elif self.type == 'S_Poly':
            if reachable_to is None or pc.PointInPolygon(reachable_to, self.points):
                poly = pn.Polygon(self.points)
                return poly.sample(random)
        return False

    def pathConnected(self, u, v):
        if self.type == 'C_Poly':
            c = 0
            d = 0
            cu = 0
            cv = 0
            for cont in self.points:
                uinc = pc.PointInPolygon(u, cont)
                vinc = pc.PointInPolygon(v, cont)
                # print("in", uinc, vinc)
                if uinc:
                    cu += 1
                if vinc:
                    cv += 1
                if uinc and vinc:
                    c += 1
                    d += 1 if pc.Orientation(cont) else -1
            # print("==", d, cu, cv, c)
            return (d > 0) and (cu == cv == c)
        elif self.type == 'S_Poly':
            # print("==", bool(pc.PointInPolygon(u, self.points)) and bool(pc.PointInPolygon(v, self.points)))
            return bool(pc.PointInPolygon(u, self.points)) and bool(pc.PointInPolygon(v, self.points))

    def drawGL(self, color=(0.5, 0.5, 0.5)):
        if self.type == 'C_Poly':
            for cont in reversed(self.points):
                if pc.Orientation(cont):
                    glColor3f(0.1, 0.5, 0.1)
                else:
                    glColor3f(0.5, 0.1, 0.1)
                for tristrip in pn.Polygon(cont).triStrip():
                    glBegin(GL_TRIANGLE_STRIP)
                    for p in tristrip:
                        glVertex2f(*p)
                    glEnd()

        elif self.type == 'S_Poly':
            glColor3f(*color)
            # glBegin(GL_LINE_LOOP)
            # for p in self.points + [self.points[0]]:
            #     glVertex2f(*p)
            # glEnd()
            for tristrip in pn.Polygon(self.points).triStrip():
                glBegin(GL_TRIANGLE_STRIP)
                for p in tristrip:
                    glVertex2f(*p)
                glEnd()


class DiskCSpace(CSpace):
    def __init__(self, rad=10, poseMap={}, obstacles=[], height=1000, width=1000):
        CSpace.__init__(self)
        #set bounds
        self.bound = [(0, width), (0, height)]
        self.wall = [
            (self.bound[0][0], self.bound[1][0]),
            (self.bound[0][1], self.bound[1][0]),
            (self.bound[0][1], self.bound[1][1]),
            (self.bound[0][0], self.bound[1][1]),
        ]
        #set collision checking resolution
        self.eps = 1

        self.robot = Circle(0, 0, rad)
        self.obstacles = obstacles

        self.mink_obs = None

        self.poseMap = poseMap  # poseMap = {id: Circle,...}
        self.regions = None
        self.pose2reg = None
        self.rGraph = None

    @staticmethod
    def from_json(filename):
        # {
        #   "n": <number of objects>,
        #   "radius": <disk radius>,
        #   "height": <environment height>,
        #   "width": <environment width>,
        #   "starts": [[x0,y0], [x1,y1], ..., [xn,yn]],  # array of coordinates indexed by object number
        #   "goals": [[x0,y0], [x1,y1], ..., [xn,yn]],  # array of coordinates indexed by object number
        #   "obstacles": [ # array of obstacles each specified as a counter clockwise polygon point array
        #     [[x0,y0]...],
        #     [[x0,y0],...],
        #     ...
        #   ]
        # }
        with open(filename) as f:
            data = json.load(f)

            posemap = {'S' + str(i): Circle(p[0], p[1], data['radius']) for i, p in enumerate(data['starts'])}
            posemap.update({'G' + str(i): Circle(p[0], p[1], data['radius']) for i, p in enumerate(data['goals'])})
            posemap.update({key: Circle(p[0], p[1], data['radius']) for key, p in data['buffers'].items()})
            # print(posemap)
            return DiskCSpace(
                data['radius'], posemap, [Poly(poly) for poly in data['obstacles']], data['height'], data['width']
            )

    def setRobotRad(self, rad):
        self.robot.radius = rad

    def saveMinkObs(self):
        return Poly(self.mink_obs.points[:])

    def restoreMinkObs(self, points):
        self.mink_obs = Poly(points[:])

    def computeMinkObs(self):
        # print(self.robot.radius)
        shape = self.robot.poly()
        clip = pc.Pyclipper()
        for o in self.obstacles:
            opoly = o.poly()
            clip.AddPath(opoly, pc.PT_CLIP, True)
            clip.AddPaths(pc.MinkowskiSum(shape, opoly, True), pc.PT_CLIP, True)
        clip.AddPaths(pc.MinkowskiSum(shape, self.wall, True), pc.PT_CLIP, True)
        clip.AddPath(self.wall, pc.PT_SUBJECT, True)
        mink_obs = clip.Execute(pc.CT_DIFFERENCE, pc.PFT_NONZERO, pc.PFT_NONZERO)
        self.mink_obs = Poly(mink_obs)

    def addPose(self, oid, obj):
        self.poseMap[oid] = obj

    def removePose(self, pid):
        return self.poseMap.pop(pid, False)

    def restorePoses(self, poses):
        self.poseMap = poses.copy()

    def savePoses(self, poses):
        return self.poseMap.copy()

    def clearPoses(self):
        self.poseMap.clear()

    def addObstacle(self, obs):
        self.obstacles.append(obs)

    def restoreObstacles(self, obs):
        self.obstacles = obs[:]

    def saveObstacles(self):
        return self.obstacles[:]

    def feasible(self, q):
        if self.mink_obs is None:
            self.computeMinkObs()
        return self.mink_obs.contains(q)

    def drawObstaclesGL(self):
        glColor3f(0.2, 0.2, 0.2)
        for o in self.obstacles:
            o.drawGL()

    def drawMinkGL(self):
        if self.mink_obs:
            self.mink_obs.drawGL()

    def drawRobotGL(self, q, color=(1, 0.5, 1)):
        glColor3f(*color)
        newc = vectorops.add(self.robot.center, q)
        c = Circle(newc[0], newc[1], self.robot.radius)
        c.drawGL()

    def drawPoses(self):
        for pid, pose in self.poseMap.items():
            dd = str(pid).strip('ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz')
            sd = str(pid).strip('0123456789')
            if dd:
                if 'B' in sd:
                    glColor3f(0, 0, 0)
                else:
                    glColor3f(*getColor(int(dd) * 2.0 / len(self.poseMap)))
                pose.drawGL()

    def drawRegionGraphGL(self, drawRegions=True, drawGraph=True):
        if self.regions is None:
            t0 = time()
            self.regionGraph()
            print("RG Time: ", time() - t0)

        if self.RG and drawGraph:
            V, E = self.RG
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            glColor4f(0, 0, 0, 0.9)
            glPointSize(5.0)
            glBegin(GL_POINTS)
            for v in V:
                glVertex2f(*self.regions[v].quasiCenter())
            glEnd()
            glColor4f(0.5, 0.5, 0.5, 0.8)
            glBegin(GL_LINES)
            for (i, j) in E:
                glVertex2f(*self.regions[i].quasiCenter())
                glVertex2f(*self.regions[j].quasiCenter())
            glEnd()
            glDisable(GL_BLEND)

        if drawRegions:
            for rid, r in sorted(self.regions.items(), reverse=True):
                if len(rid) == 1:
                    r.drawGL((0.1, 0.5, 0.1))
                    continue
                colors = []
                for d in rid:
                    dd = str(d).strip('ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz')
                    sd = str(d).strip('0123456789')
                    if dd:
                        # seed(dd)
                        if 'B' in sd:
                            colors.append((0, 0, 0))
                        else:
                            colors.append(getColor(int(dd) * 2.0 / len(self.poseMap)))
                r.drawGL(avgColor(colors))

                # seed(rid[:-1])
                # r.drawGL()
                # r.drawGL(getColor(sum(rid))
                # r.drawGL(choice(getColors(100)))
                # r.drawGL((random(), random(), random()))
                # seed()

    def regionGraph(self, filterfunc=lambda x: True):
        """
        prune_dist =
            0 if you don't want to prune edges
            >0 the hamming distance you want to prune
            <0 will prune using absolute value as the hamming
               distance but always connect to the free space
        """
        if self.mink_obs is None:
            self.computeMinkObs()

        shape = self.robot.poly()
        polysum = []
        regions = {}
        # clip = pc.Pyclipper()
        for i, o in filter(filterfunc, self.poseMap.items()):
            # clip.AddPaths(pc.MinkowskiSum(shape, o.poly(), True), pc.PT_CLIP, True)
            # obj = clip.Execute(pc.CT_UNION, pc.PFT_NONZERO, pc.PFT_NONZERO)
            # obj = pc.MinkowskiSum(shape, o.poly(), True)
            obj = Circle(o.center[0], o.center[1], o.radius + self.robot.radius).poly()
            for rid, r in regions.items():
                rANDobj = polyINTER(r, obj)
                if rANDobj:
                    regions[rid + (i, )] = rANDobj
                    rDIFobj = polyDIFF(r, obj)
                    if rDIFobj:
                        regions[rid] = rDIFobj
                    else:
                        del regions[rid]

            objDIFFpolysum = polyDIFF(polyINTER(self.mink_obs.points, obj), polysum)
            if objDIFFpolysum:
                regions[(i, )] = objDIFFpolysum
            polysum = polyUNION(polysum, obj)

        obj2reg = {}
        for rid, r in regions.items():
            char = 'a'
            for cont in r:
                poly = Poly(cont)
                rid_n = rid + (char, )
                char = chr(ord(char) + 1)
                regions[rid_n] = poly

                for i, p in self.poseMap.items():
                    # if pc.PointInPolygon(p.center, cont):
                    if findNearest(p.center, EPSILON, lambda x: pc.PointInPolygon(x, cont)):
                        # print(i, p)
                        # print(rid_n)
                        obj2reg[i] = rid_n

            del regions[rid]

        cfree = Poly(polyDIFF(self.mink_obs.points, polysum))
        icont = list(enumerate(reversed(cfree.points), 1))
        for i, cont in icont:
            # print(i, pc.Area(cont), pc.Orientation(cont))
            if pc.Orientation(cont):
                poly = cont
                for j, kont in icont[:i - 1]:
                    if not pc.Orientation(kont):
                        poly = polyDIFF(poly, kont)
                regions[(-i, )] = Poly(poly)
            # poly = Poly(pc.PolyTreeToPaths(node))
            # regions[(-i, )] = poly

        self.regions = regions
        self.pose2reg = obj2reg
        # print(self.regions)
        # for rid, r in regions.items():
        #     print(rid, r.type, r.points)

        paths = []
        for rkv1, rkv2 in combinations(regions.items(), 2):
            rid1, r1 = rkv1
            rid2, r2 = rkv2

            s1 = set(rid1[:-1])
            s2 = set(rid2[:-1])
            s1vs2 = s1.intersection(s2)

            ### Pruning Strategy ###
            # prune_dist = 1
            if s1 and s2 and not s1vs2:
                continue
            # if len(s1 ^ s2) == 0:
            #     # print(rid1, s1, rid2, s2)
            #     continue
            # if prune_dist != 0:
            #     if len(s1 ^ s2) > abs(prune_dist):
            #         if s1 and s2 or prune_dist > 0:
            #             continue
            if len(s1 ^ s2) != 1:
                continue

            if r1.type == 'C_Poly' and r2.type == 'C_Poly':
                # print(rid1, rid2)
                continue
            elif r1.type == 'C_Poly':
                c = 0
                for cont in r1.points:
                    #     if polyTOUCH(cont, r2.points):
                    #         if not pc.Orientation(cont):
                    #             c += 1
                    #         else:
                    #             c -= 1
                    # if c > 0:
                    #     paths.append((rid1, rid2))

                    if set([(x, y) for x, y in cont]).intersection(set([(x, y) for x, y in r2.points])):
                        paths.append((rid1, rid2))
                        break
            elif r2.type == 'C_Poly':
                c = 0
                for cont in r2.points:
                    #     if polyTOUCH(cont, r1.points):
                    #         if not pc.Orientation(cont):
                    #             c += 1
                    #         else:
                    #             c -= 1
                    # if c > 0:
                    #     paths.append((rid1, rid2))
                    if set([(x, y) for x, y in cont]).intersection(set([(x, y) for x, y in r1.points])):
                        paths.append((rid1, rid2))
                        break
            else:
                if polyTOUCH(r1.points, r2.points):
                    paths.append((rid1, rid2))

        graph = {}
        for u, v in paths:
            graph[u] = sorted(graph.get(u, []) + [v])
            graph[v] = sorted(graph.get(v, []) + [u])

        self.RG = (graph.keys(), paths)
        self.RGAdj = graph
        # print(self.RG)
        # print(self.RGAdj)
        # print(self.pose2reg)


class DiskCSpaceProgram(GLProgram):
    def __init__(self, space, actions=[], arrangements=[], start=None, goal=None):
        GLProgram.__init__(self)
        self.space = space
        self.staticObs = space.saveObstacles()
        #PRM planner
        # MotionPlan.setOptions(type="prm", knn=10, connectionThreshold=0.1)
        # self.optimizingPlanner = False

        #FMM* planner
        #MotionPlan.setOptions(type="fmm*")
        #self.optimizingPlanner = True

        #RRT planner
        # MotionPlan.setOptions(type="rrt", perturbationRadius=25, bidirectional=True)
        # self.optimizingPlanner = False

        #RRT* planner
        # MotionPlan.setOptions(type="rrt*")
        # self.optimizingPlanner = True

        #random-restart RRT planner
        MotionPlan.setOptions(
            type="rrt",
            perturbationRadius=10,
            bidirectional=True,
            shortcut=True,
            restart=True,
            restartTermCond="{foundSolution:1,maxIters:1000}"
        )
        self.optimizingPlanner = True

        #OMPL planners:
        #Tested to work fine with OMPL's prm, lazyprm, prm*, lazyprm*, rrt, rrt*, rrtconnect, lazyrrt, lbtrrt, sbl, bitstar.
        #Note that lbtrrt doesn't seem to continue after first iteration.
        #Note that stride, pdst, and fmt do not work properly...
        #MotionPlan.setOptions(type="ompl:rrt",suboptimalityFactor=0.1,knn=10,connectionThreshold=0.1)
        #self.optimizingPlanner = True

        self.move_actions = actions
        self.arrangements = arrangements

        self.index = -1
        self.start = start
        self.goal = goal
        self.planner = None
        if self.start is not None and self.goal is not None:
            self.planner = MotionPlan(space)
            self.planner.setEndpoints(start, goal)
        self.path = []
        self.G = None
        self.drawRegions = False
        self.drawRGraph = False
        self.drawPoses = True

    def keyboardfunc(self, key, x, y):
        if key == ' ':
            if self.planner is not None:
                if self.optimizingPlanner or not self.path:
                    print("Planning 1...")
                    self.planner.planMore(1)
                    self.path = self.planner.getPath()
                    self.G = self.planner.getRoadmap()
                    self.refresh()
        elif key == 'p':
            if self.planner is not None:
                if self.optimizingPlanner or not self.path:
                    print("Planning 100...")
                    self.planner.planMore(100)
                    self.path = self.planner.getPath()
                    self.G = self.planner.getRoadmap()
                    self.refresh()
        elif key == 'r':
            self.drawRegions = not self.drawRegions
        elif key == 'g':
            self.drawRGraph = not self.drawRGraph
        elif key == 'l':
            if self.move_actions:
                self.drawPoses = False
                self.index += 1
                if not self.getQuery():
                    self.index -= 1
                    # self.getQuery()
                else:
                    self.path = None
                    self.G = None
        elif key == 'h':
            if self.move_actions:
                self.drawPoses = False
                self.index -= 1
                if not self.getQuery():
                    self.index += 1
                    self.getQuery()
                else:
                    self.path = None
                    self.G = None
        elif key == 'c':
            self.drawPoses = True
            self.space.restoreObstacles(self.staticObs)
            self.space.computeMinkObs()
            self.path = None
            # print(self.space.pose2reg['S1'])
            # print(self.space.pose2reg['G1'])
            # path = BFS(self.space.RGAdj, self.space.pose2reg['S1'], self.space.pose2reg['G1'])
            # self.path = [self.space.regions[p].quasiCenter() for p in path]
            # print(self.path)
            self.G = None
            self.planner = None
            self.index = -1
            self.refresh()

    def getQuery(self):
        if 0 <= self.index < len(self.move_actions):
            pstart = self.move_actions[self.index][1][0]
            pgoal = self.move_actions[self.index][1][1]
            self.space.restoreObstacles(self.staticObs)
            for pid in self.arrangements[self.index]:
                if pid != pstart and pid != pgoal:
                    self.space.addObstacle(self.space.poseMap[pid])
            self.space.computeMinkObs()

            self.planner = MotionPlan(self.space)
            self.start = findNearest(self.space.poseMap[pstart].center, EPSILON, self.space.mink_obs.contains)
            self.goal = findNearest(self.space.poseMap[pgoal].center, EPSILON, self.space.mink_obs.contains)
            self.planner.setEndpoints(self.start, self.goal)
            self.refresh()
            return True
        return False

    def display(self):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        mB = max([y for x, y in self.space.bound])
        glOrtho(0, mB, mB, 0, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        glDisable(GL_LIGHTING)
        if self.path:
            #draw path
            glColor3f(0, 1, 0)
            glBegin(GL_LINE_STRIP)
            for q in self.path:
                glVertex2f(q[0], q[1])
            glEnd()
            for q in self.path:
                self.space.drawRobotGL(q)
        else:
            if self.planner:
                self.space.drawRobotGL(self.start, (1, 0, 1))
                self.space.drawRobotGL(self.goal, (0.5, 0, 0.5))

        if self.G:
            #draw graph
            V, E = self.G
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            glColor4f(0, 0, 0, 0.7)
            glPointSize(3.0)
            glBegin(GL_POINTS)
            for v in V:
                glVertex2f(v[0], v[1])
            glEnd()
            glColor4f(0.5, 0.5, 0.5, 0.6)
            glBegin(GL_LINES)
            for (i, j) in E:
                glVertex2f(V[i][0], V[i][1])
                glVertex2f(V[j][0], V[j][1])
            glEnd()
            glDisable(GL_BLEND)

        self.space.drawObstaclesGL()
        self.space.drawRegionGraphGL(self.drawRegions, self.drawRGraph)
        if not self.drawRegions and self.drawPoses:
            self.space.drawPoses()
        self.space.drawMinkGL()


def loadEnv(filename):
    if filename[-4:] == 'json':
        return DiskCSpace.from_json(filename)
    else:
        with open(filename) as f:
            return eval(f.read())


def genPoses(n, space):
    ### save current obstacles
    staticObstacles = space.saveObstacles()

    for i in range(n):
        ### need to generate both start and goal
        for j in range(2):
            sorg = 'G' if j else 'S'
            ### reset obstacles
            space.restoreObstacles(staticObstacles)
            for pid, pose in space.poseMap.items():
                ### For dense case,
                ### start only checks with starts
                ### goal only checks with goals
                # if j % 2 == pid % 2:
                if pid[0] == sorg:
                    space.addObstacle(pose)
            ### compute cspace
            space.computeMinkObs()
            if space.mink_obs.type != 'Empty':
                ### try to sample point
                # point = space.mink_obs.sample()
                point = findNearest([int(xy) for xy in space.mink_obs.sample()], EPSILON, space.mink_obs.contains)
            else:
                point = None
            if not point:
                if j == 0:
                    print("failed to generate the start of object " + str(i))
                else:
                    print("failed to generate the goal of object " + str(i))
                print("FAIL TO GENERATE THE INITIAL INSTANCE")

                ### restore obstacles
                space.restoreObstacles(staticObstacles)
                space.computeMinkObs()
                return False

            ### Congrats the object's goal/start is accepted
            space.addPose(sorg + str(i), Circle(point[0], point[1], space.robot.radius))
            # space.addPose(2 * i + j, Circle(point[0], point[1], space.robot.radius))

    ### restore obstacles
    space.restoreObstacles(staticObstacles)
    space.computeMinkObs()
    return True


def genBuffers(n, space, occupied, method='random', param1=0, param2=[], count=0, suffix=''):
    num_generated = 0
    staticObstacles = space.saveObstacles()

    ### Random Sampling w/ max overlaps ###
    if method == 'random':
        maxOverlap = param1
        noccupied = occupied[:]
        numOverlapAllowed = 0
        for i in range(n):
            isValid = False
            timeout = 500
            while not isValid and timeout > 0:
                timeout -= 1
                numOverlap = 0
                # point = space.mink_obs.sample()
                point = findNearest([int(xy) for xy in space.mink_obs.sample()], EPSILON, space.mink_obs.contains)
                for pid in noccupied:
                    p = space.poseMap[pid]
                    if Circle(p.center[0], p.center[1], p.radius * 2).contains(point):
                        numOverlap += 1

                # print(numOverlap, p.center)
                if numOverlap <= numOverlapAllowed:
                    isValid = True

            ### reach here either (1) isValid == True (2) timeout <= 0
            if timeout <= 0:
                ### keep failing generating a buffer allowed to overlap with maximum number of objects
                ### increase the numOverlapAllowed
                numOverlapAllowed += 1
                if (numOverlapAllowed > maxOverlap):
                    print("Exceed the maximum limit of numOverlap for buffer generation")
                    break

            ### Otherwise the buffer is accepted
            space.addPose('B' + str(i + count) + suffix, Circle(point[0], point[1], space.robot.radius))
            noccupied.append('B' + str(i + count) + suffix)
            num_generated += 1

    ### Greedy Free Space Sampling ###
    elif method == 'greedy_free':
        for pid in occupied:
            p = space.poseMap[pid]
            space.addObstacle(p)

        for i in range(n):
            space.computeMinkObs()
            if space.mink_obs.type != 'Empty':
                # point = space.mink_obs.sample()
                point = findNearest([int(xy) for xy in space.mink_obs.sample()], EPSILON, space.mink_obs.contains)
            else:
                print("No free space!")
                break
            space.addPose('B' + str(i + count) + suffix, Circle(point[0], point[1], space.robot.radius))
            space.addObstacle(Circle(point[0], point[1], space.robot.radius))
            num_generated += 1

    ### Greedy Boundary Sampling ###
    elif method == 'greedy_boundary':
        for pid in occupied:
            p = space.poseMap[pid]
            space.addObstacle(p)

        space.setRobotRad(space.robot.radius + 5)
        space.computeMinkObs()
        space.setRobotRad(space.robot.radius - 5)

        for i in range(n):
            space.computeMinkObs()
            if space.mink_obs.type != 'Empty':
                # point = space.mink_obs.sample()
                # point = findNearest([int(xy) for xy in space.mink_obs.sample()], EPSILON, space.mink_obs.contains)
                if space.mink_obs.type == 'S_Poly':
                    b_points = space.mink_obs.points
                elif space.mink_obs.type == 'C_Poly':
                    b_points = choice(space.mink_obs.points)
                ind = choice(range(-1, len(b_points) - 1))
                p1 = b_points[ind]
                p2 = b_points[ind + 1]
                point = findNearest(
                    [int(xy) for xy in vectorops.interpolate(p1, p2, random())], EPSILON, space.mink_obs.contains
                )
            else:
                print("No free space!")
                break
            space.addPose('B' + str(i + count) + suffix, Circle(point[0], point[1], space.robot.radius))
            space.addObstacle(Circle(point[0], point[1], space.robot.radius))
            num_generated += 1

    ### Random Boundary Sampling ###
    elif method == 'boundary_random':
        for pid in occupied:
            p = space.poseMap[pid]
            space.addObstacle(p)

        space.setRobotRad(space.robot.radius + 5)
        space.computeMinkObs()
        space.setRobotRad(space.robot.radius - 5)

        maxOverlap = param1
        boccupied = []
        numOverlapAllowed = 0
        for i in range(n):
            isValid = False
            timeout = 500
            while not isValid and timeout > 0:
                timeout -= 1
                numOverlap = 0
                if space.mink_obs.type != 'Empty':
                    if space.mink_obs.type == 'S_Poly':
                        b_points = space.mink_obs.points
                    elif space.mink_obs.type == 'C_Poly':
                        b_points = choice(space.mink_obs.points)
                    ind = choice(range(-1, len(b_points) - 1))
                    p1 = b_points[ind]
                    p2 = b_points[ind + 1]
                    point = findNearest(
                        [int(xy) for xy in vectorops.interpolate(p1, p2, random())], EPSILON, space.mink_obs.contains
                    )
                else:
                    print("No free space!")
                    # set params to break out of outer loop
                    timeout = -1
                    numOverlapAllowed = maxOverlap + 1
                    break
                for pid in boccupied:
                    p = space.poseMap[pid]
                    if Circle(p.center[0], p.center[1], p.radius + space.robot.radius).contains(point):
                        numOverlap += 1

                if numOverlap <= numOverlapAllowed:
                    isValid = True

            if timeout <= 0:
                numOverlapAllowed += 1
                if (numOverlapAllowed > maxOverlap):
                    print("Exceed the maximum limit of numOverlap for buffer generation")
                    break

            space.addPose('B' + str(i + count) + suffix, Circle(point[0], point[1], space.robot.radius))
            boccupied.append('B' + str(i + count) + suffix)
            num_generated += 1

    ### Sample feasible region for given object and ordering ###
    elif method == 'object_feasible':
        # obj_mob = []
        for pid in occupied:
            # if pid[0] != 'G':  # get objects not at goal poses
            #     obj_mob.append(int(pid[1:]))
            p = space.poseMap[pid]
            space.addObstacle(p)

        # obj_sel = param1
        # obj_ord = param2
        # for obj in obj_ord:
        #     if obj == obj_sel:
        #         break
        #     p = space.poseMap['S' + str(obj)]
        #     space.addObstacle(p)
        pose_sel = param1

        space.computeMinkObs()
        for i in range(n):
            if space.mink_obs.type != 'Empty':
                # point = space.mink_obs.sample(space.poseMap[pose_sel].center)
                p = space.mink_obs.sample(space.poseMap[pose_sel].center)
                if p:
                    point = findNearest([int(xy) for xy in p], EPSILON, space.mink_obs.contains)
                else:
                    point = p
            else:
                print("No free space!")
                break
            if point:
                space.addPose('B' + str(i + count) + suffix, Circle(point[0], point[1], space.robot.radius))
                # space.addObstacle(Circle(point[0], point[1], space.robot.radius))
                num_generated += 1
            else:
                print("No feasible space!")

    ### Sample boundary of feasible region for given object and ordering ###
    elif method == 'boundary_feasible':
        for pid in occupied:
            p = space.poseMap[pid]
            space.addObstacle(p)
        space.setRobotRad(space.robot.radius + 5)
        space.computeMinkObs()
        space.setRobotRad(space.robot.radius - 5)

        pose_sel = param1

        b_points = []
        if space.mink_obs.type == 'S_Poly':
            b_points = space.mink_obs.points
        elif space.mink_obs.type == 'C_Poly':
            for c in space.mink_obs.points:
                if pc.Orientation(c) and pc.PointInPolygon(space.poseMap[pose_sel].center, c):
                    b_points = c

        for i in range(n):
            if b_points:
                ind = choice(range(-1, len(b_points) - 1))
                p1 = b_points[ind]
                p2 = b_points[ind + 1]
                point = findNearest(
                    [int(xy) for xy in vectorops.interpolate(p1, p2, random())], EPSILON, space.mink_obs.contains
                )
            else:
                print("No feasible space!")
                break
            space.addPose('B' + str(i + count) + suffix, Circle(point[0], point[1], space.robot.radius))
            # space.addObstacle(Circle(point[0], point[1], space.robot.radius))
            num_generated += 1

    ### reset obstacles
    space.restoreObstacles(staticObstacles)
    space.computeMinkObs()
    return num_generated


if __name__ == '__main__':
    space = None
    numObjs = 5
    rad = 50
    height = 1000
    width = 1000

    if len(sys.argv) > 1:
        if sys.argv[1].isdigit():
            numObjs = int(sys.argv[1])
        else:
            space = loadEnv(sys.argv[1])
            rad = space.robot.radius
            height = space.bound[1][1]
            width = space.bound[0][1]

    if len(sys.argv) > 2:
        if space is None:
            rad = int(sys.argv[2])
        else:
            numObjs = int(sys.argv[2])

    if len(sys.argv) > 3:
        if space is None:
            height = int(sys.argv[3])
        else:
            rad = int(sys.argv[3])

    if len(sys.argv) > 4:
        width = int(sys.argv[4])

    if space is None:
        space = DiskCSpace(rad, {}, [], height, width)
    else:
        space.setRobotRad(rad)

    space.computeMinkObs()
    if space.mink_obs.type == 'S_Poly':
        print(pc.Area(space.mink_obs.points))
    elif space.mink_obs.type == 'C_Poly':
        print([pc.Area(x) for x in space.mink_obs.points])
    else:
        print("WTF?")

    if len(space.poseMap) == 0:
        genPoses(numObjs, space)

    space.regionGraph()
    if num_buffers > 0:
        num_generated = genBuffers(num_buffers, space, space.poseMap.keys(), 'greedy_free')
        print(num_generated)
        num_generated = genBuffers(
            num_buffers - num_generated,
            space,
            space.poseMap.keys(),
            'random',
            len(space.poseMap.keys()),
            count=num_generated
        )
        print(num_generated)
        # genBuffers(num_buffers, space, space.poseMap.keys(), 'random', len(space.poseMap.keys()))
        # genBuffers(num_buffers, space, space.poseMap.keys(), 'greedy_free')
        # genBuffers(num_buffers, space, space.poseMap.keys(), 'greedy_boundary')
        # genBuffers(num_buffers, space, [], 'greedy_free')
        # genBuffers(num_buffers, space, [], 'greedy_boundary')
        # genBuffers(num_buffers, space, [], 'boundary_random', 1)
        # genBuffers(num_buffers, space, space.poseMap.keys(), 'boundary_random', 2)
        # genBuffers(num_buffers, space, filter(lambda x: x[0] == 'S', space.poseMap.keys()), 'boundary_feasible', 'G1')
        # genBuffers(num_buffers, space, filter(lambda x: x[0] == 'S', space.poseMap.keys()), 'object_feasible', 'G1')
        space.regionGraph()

    outfile = sys.stderr
    if len(sys.argv) > 5:
        outfile = open(sys.argv[5], 'w')

    print(
        """DiskCSpace(
    rad={},
    height={},
    width={},""".format(
            rad,
            height,
            width,
        ),
        file=outfile,
    )
    print('    obstacles=[', file=outfile)
    for x in space.obstacles:
        print('        ', x, ',', sep='', file=outfile)
    print('    ],', file=outfile)

    print('    poseMap={', file=outfile)
    for k, v in space.poseMap.items():
        print("        '", k, "': ", v, ',', sep='', file=outfile)
    print('    },\n)', file=outfile)

    if outfile is not sys.stderr:
        outfile.close()

    program = DiskCSpaceProgram(space)
    program.view.w = program.view.h = 1080
    program.name = "Motion planning test"
    program.run()
