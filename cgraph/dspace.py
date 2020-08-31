from __future__ import print_function
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from klampt.plan.cspace import CSpace, MotionPlan
from klampt.vis.glprogram import GLProgram
from klampt.math import vectorops

import sys
import math
from time import time
from copy import deepcopy
from itertools import combinations
from random import random, seed, choice

import Polygon as pn
import pyclipper as pc

from util import *


def interpolate(a, b, u):
    """Interpolates linearly between a and b"""
    return vectorops.madd(a, vectorops.sub(b, a), u)


class Circle:
    def __init__(self, x=0, y=0, radius=1):
        self.center = (x, y)
        self.radius = radius
        self.type = 'Circle'

    def contains(self, point):
        return (vectorops.distance(point, self.center) <= self.radius)

    def poly(self, res=55, toint=True):
        pnts = []
        numdivs = int(math.ceil(self.radius * math.pi * 2 / res))
        for i in xrange(numdivs + 1):
            u = float(i) / float(numdivs) * math.pi * 2
            if toint:
                pnts.append(
                    (
                        int(self.center[0] + self.radius * math.cos(u)),
                        int(self.center[1] + self.radius * math.sin(u)),
                    )
                )
            else:
                pnts.append((
                    self.center[0] + self.radius * math.cos(u),
                    self.center[1] + self.radius * math.sin(u),
                ))
        return pnts

    def drawGL(self, res=55):
        glBegin(GL_TRIANGLE_FAN)
        glVertex2f(*self.center)
        for p in self.poly(res, False):
            glVertex2f(*p)
        glEnd()


class Rectangle:
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
            (self.center[0], self.center[1]),
            (self.center[0], self.center[1] + self.height),
            (self.center[0] + self.width, self.center[1] + self.height),
            (self.center[0] + self.width, self.center[1]),
        ]

    def drawGL(self):
        glBegin(GL_QUADS)
        for p in self.poly():
            glVertex2f(*p)
        glEnd()


class Poly:
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
        c = 0
        for cont in self.points:
            if pc.PointInPolygon(point, cont):
                if pc.Orientation(cont):
                    c += 1
                else:
                    c -= 1
        return c > 0

    def quasiCenter(self):
        if self.type == 'C_Poly':
            poly = pn.Polygon()
            for cont in self.points:
                if pc.Orientation(cont):
                    poly += pn.Polygon(cont)
                else:
                    poly -= pn.Polygon(cont)
            if poly.isInside(*poly.center()) > 0:
                return poly.center()
            else:
                return poly.sample(o55)
        elif self.type == 'S_Poly':
            poly = pn.Polygon(self.points)
            if pc.PointInPolygon(poly.center(), self.points) > 0:
                return poly.center()
            else:
                return poly.sample(o55)

    def sample(self):
        if self.type == 'C_Poly':
            poly = pn.Polygon()
            for cont in self.points:
                if pc.Orientation(cont):
                    poly += pn.Polygon(cont)
                else:
                    poly -= pn.Polygon(cont)
            else:
                return poly.sample(random)
        elif self.type == 'S_Poly':
            poly = pn.Polygon(self.points)
            return poly.sample(random)

    def drawGL(self, color=(0.5, 0.5, 0.5)):
        if self.type == 'C_Poly':
            # glColor3f(*color)
            # for cont in self.points:
            for cont in reversed(self.points):
                if pc.Orientation(cont):
                    glColor3f(0.1, 0.5, 0.1)
                else:
                    glColor3f(0.5, 0.1, 0.1)
                # glBegin(GL_LINE_LOOP)
                # for p in cont:
                #     glVertex2f(*p)
                # glEnd()
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

    def setRobotRad(self, rad):
        self.robot.radius = rad

    def addObstacle(self, obs):
        self.obstacles.append(obs)

    def computeMinkObs(self):
        # print(self.robot.radius)
        shape = self.robot.poly()
        clip = pc.Pyclipper()
        for o in self.obstacles:
            clip.AddPaths(pc.MinkowskiSum(shape, o.poly(), True), pc.PT_CLIP, True)
        clip.AddPaths(pc.MinkowskiSum(shape, self.wall, True), pc.PT_CLIP, True)
        clip.AddPath(self.wall, pc.PT_SUBJECT, True)
        mink_obs = clip.Execute(pc.CT_DIFFERENCE, pc.PFT_NONZERO, pc.PFT_NONZERO)
        self.mink_obs = Poly(mink_obs)

    def addPose(self, oid, obj):
        self.poseMap[oid] = obj

    def removePose(self, oid, obj):
        return self.poseMap.pop(oid, False)

    def setPoses(self, occ):
        self.occupied = occ

    def clearPoses(self, obj):
        self.poseMap.clear()

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

    def drawOccupiedGL(self):
        glColor3f(0.4, 0.1, 0.1)
        for o in self.occupied:
            o.drawGL()

    def drawRobotGL(self, q):
        glColor3f(0, 0, 1)
        newc = vectorops.add(self.robot.center, q)
        c = Circle(newc[0], newc[1], self.robot.radius)
        c.drawGL()

    def drawRegionGraphGL(self):
        if self.regions is None:
            t0 = time()
            self.regionGraph()
            print("RG Time: ", time() - t0)

        if self.RG:
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

        for rid, r in sorted(self.regions.items(), reverse=True):
            colors = []
            for d in rid:
                dd = str(d).strip('ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz')
                if dd:
                    # seed(dd)
                    colors.append(getColor(int(dd) * 2.0 / len(self.poseMap)))
            r.drawGL(avgColor(colors))

            # seed(rid[:-1])
            # r.drawGL()
            # r.drawGL(getColor(sum(rid))
            # r.drawGL(choice(getColors(100)))
            # r.drawGL((random(), random(), random()))
            # seed()

    def regionGraph(self, prune_dist=0):
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
        for i, o in self.poseMap.items():
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
                    if pc.PointInPolygon(p.center, cont):
                        obj2reg[i] = rid_n

            del regions[rid]

        cfree = Poly(polyDIFF(self.mink_obs.points, polysum))
        icont = list(enumerate(reversed(cfree.points), 1))
        for i, cont in icont:
            print(i, pc.Area(cont), pc.Orientation(cont))
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
        # for rid, r in regions.items():
        #     print(rid, r.type, r.points)

        paths = []
        for rkv1, rkv2 in combinations(regions.items(), 2):
            rid1, r1 = rkv1
            rid2, r2 = rkv2

            s1 = set(rid1[:-1])
            s2 = set(rid2[:-1])
            s1vs2 = s1.intersection(s2)

            # if s1 and s2 and not s1vs2:
            #     continue
            # if prune_dist != 0:
            #     if len(s1 ^ s2) > abs(prune_dist):
            #         if s1 and s2 or prune_dist > 0:
            #             continue

            if r1.type == 'C_Poly' and r2.type == 'C_Poly':
                print(rid1, rid2)
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


class DiskCSpaceProgram(GLProgram):
    def __init__(self, space, start=None, goal=None):
        GLProgram.__init__(self)
        self.space = space
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

        self.start = start
        self.goal = goal
        self.planner = None
        if start is not None and goal is not None:
            self.planner = MotionPlan(space)
            self.planner.setEndpoints(start, goal)
        self.path = []
        self.G = None

    def keyboardfunc(self, key, x, y):
        if self.planner is not None:
            if key == ' ':
                if self.optimizingPlanner or not self.path:
                    print("Planning 1...")
                    self.planner.planMore(1)
                    self.path = self.planner.getPath()
                    self.G = self.planner.getRoadmap()
                    self.refresh()
            elif key == 'p':
                if self.optimizingPlanner or not self.path:
                    print("Planning 100...")
                    self.planner.planMore(100)
                    self.path = self.planner.getPath()
                    self.G = self.planner.getRoadmap()
                    self.refresh()
            # elif key == 'q':
            #     self.

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
                self.space.drawRobotGL(self.start)
                self.space.drawRobotGL(self.goal)

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
        self.space.drawRegionGraphGL()
        self.space.drawMinkGL()


def loadEnv(filename):
    with open(filename) as f:
        return eval(f.read())


def genPoses(n, space=DiskCSpace()):
    staticObstacles = space.obstacles[:]
    for i in range(n):
        ### need to generate both start and goal
        for j in range(2):
            sorg = 'G' if j else 'S'
            ### reset obstacles
            space.obstacles = staticObstacles[:]
            for pid, pose in space.poseMap.items():
                ### For dense case,
                ### start only checks with starts
                ### goal only checks with goals
                # if j % 2 == pid % 2:
                if pid[-1] == sorg:
                    space.addObstacle(pose)
            ### compute cspace
            space.computeMinkObs()
            try:
                ### try to sample point
                point = space.mink_obs.sample()
            except:
                if j == 0:
                    print("failed to generate the start of object " + str(i))
                else:
                    print("failed to generate the goal of object " + str(i))
                print("FAIL TO GENERATE THE INITIAL INSTANCE")
                return

            ### Congrats the object's goal/start is accepted
            space.poseMap[str(i) + sorg] = Circle(point[0], point[1], space.robot.radius)

    ### reset obstacles
    space.obstacles = staticObstacles[:]
    space.computeMinkObs()


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

    if len(sys.argv) > 2:
        if space is None:
            rad = int(sys.argv[2])
        else:
            numObjs = int(sys.argv[2])

    if len(sys.argv) > 3:
        height = int(sys.argv[3])

    if len(sys.argv) > 4:
        width = int(sys.argv[4])

    if space is None:
        space = DiskCSpace(rad, {}, [], height, width)

    if len(space.poseMap) == 0:
        genPoses(numObjs, space)

    # space = DiskCSpace(rad=50, poseMap=poseMap)
    # space.addObstacle(Circle(700, 500, 120))
    # space.addObstacle(Rectangle(295, 400, 5, 300))
    # space.addObstacle(Rectangle(295, 400, 300, 5))
    # space.addObstacle(Rectangle(595, 700, -300, -5))
    # space.addObstacle(Rectangle(595, 700, -5, -300))
    # space.computeMinkObs()
    # print(space.mink_obs.points, space.mink_obs.type, space.mink_obs.sample())
    # start = (150, 150)
    # goal = (850, 850)
    program = DiskCSpaceProgram(space)
    program.view.w = program.view.h = 1080
    program.name = "Motion planning test"
    program.run()

    # program = DiskCSpaceProgram(space)
    # program.view.w = program.view.h = 1080
    # program.name = "Motion planning test2"
    # program.run()

    print("TEST")
