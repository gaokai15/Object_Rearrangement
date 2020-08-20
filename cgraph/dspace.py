from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from klampt.plan.cspace import CSpace, MotionPlan
from klampt.vis.glprogram import GLProgram
from klampt.math import vectorops

import sys
import math
from copy import deepcopy

import Polygon as pn
import pyclipper as pc

from util import *


def interpolate(a, b, u):
    """Interpolates linearly between a and b"""
    return vectorops.madd(a, vectorops.sub(b, a), u)


class Circle:
    def __init__(self, x=0, y=0, radius=1, ind=-1):
        self.center = (x, y)
        self.radius = radius
        self.type = 'Circle'
        self.id = ind

    def contains(self, point):
        return (vectorops.distance(point, self.center) <= self.radius)

    def poly(self, res=20, toint=True):
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

    def drawGL(self, res=20):
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
        self.points = list(sorted(points, key=lambda x: abs(pc.Area(x))))
        self.type = 'Poly'

    def contains(self, point):
        c = 0
        for cont in self.points:
            if pc.PointInPolygon(point, cont):
                if pc.Orientation(cont):
                    c += 1
                else:
                    c -= 1
        return c > 0

    def drawGL(self):
        for cont in self.points:
            if pc.Orientation(cont):
                glColor3f(0.1, 0.5, 0.1)
            else:
                glColor3f(0.5, 0.1, 0.1)
            # glBegin(GL_POLYGON)
            glBegin(GL_LINE_LOOP)
            for p in cont:
                glVertex2f(*p)
            glEnd()


class DiskCSpace(CSpace):
    def __init__(self, rad=10, height=1000, width=1000):
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
        #setup a robot with radius 10
        self.robot = Circle(0, 0, rad)
        #set obstacles here
        self.obstacles = []
        self.occupied = []

        self.mink_obs = None

    def setRobotRad(self, rad):
        self.robot.radius = rad

    def addObstacle(self, obs):
        self.obstacles.append(obs)

    def computeMinkObs(self):
        print(self.robot.radius)
        shape = self.robot.poly()
        clip = pc.Pyclipper()
        for o in self.obstacles + self.occupied:
            clip.AddPaths(pc.MinkowskiSum(shape, o.poly(), True), pc.PT_CLIP, True)
        clip.AddPaths(pc.MinkowskiSum(shape, self.wall, True), pc.PT_CLIP, True)
        clip.AddPath(self.wall, pc.PT_SUBJECT, True)
        mink_obs = clip.Execute(pc.CT_DIFFERENCE, pc.PFT_NONZERO, pc.PFT_NONZERO)
        self.mink_obs = Poly(mink_obs)

    def addOccupied(self, obj):
        self.occupied.append(obj)

    def setOccupied(self, occ):
        self.occupied = occ

    def clearOccupied(self, obj):
        self.occupied = []

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

    def regionGraph(self, poseMap):  # poseMap = {id: Circle,...}
        regions = {}
        shape = self.robot.poly()
        clip = pc.Pyclipper()
        clipS = pc.Pyclipper()
        for i, o in poseMap.items():
            obj = pc.MinkowskiSum(shape, o.poly(), True)
            clipS.AddPaths(obj, pc.PT_CLIP, True)
            for rid, r in regions.items():
                clip.cl
                rANDobj = r & obj
                if rANDobj:
                    regions[rind + (i, )] = rANDobj
                    rDIFobj = r - obj
                    if rDIFobj:
                        regions[rind] = rDIFobj
                    else:
                        del regions[rind]

            objDIFFpolysum = wall_mink & obj - polysum
            # print(objDIFFpolysum, bool(objDIFFpolysum))
            if objDIFFpolysum:
                regions[(i, )] = objDIFFpolysum
            polysum += obj

        obj2reg = {}
        for rind, r in regions.items():
            # if len(r) > 1:
            char = 'a'
            for cont in r:
                poly = pn.Polygon(cont)
                rind_n = rind + (char, )
                char = chr(ord(char) + 1)

                if poly.isInside(*poly.center()):
                    regions[rind_n] = (poly, poly.center())
                else:
                    regions[rind_n] = (poly, poly.sample(random))

                for i, p in enumerate(all_points):
                    if poly.isInside(*p):
                        obj2reg[i] = rind_n

            del regions[rind]

        cfree = wall_mink - polysum
        for i, pfree in enumerate(pu.fillHoles(cfree), 1):
            r = pn.Polygon(pfree)
            for isHole, cont in zip(cfree.isHole(), cfree):
                if isHole: r -= pn.Polygon(cont)
            if r.isInside(*r.center()):
                regions[(-i, )] = (r, r.center())
            else:
                regions[(-i, )] = (r, r.sample(random))

        return regions, obj2reg


class DiskCSpaceObstacleProgram(GLProgram):
    def __init__(self, space, start, goal):
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

        self.planner = MotionPlan(space)
        self.start = start
        self.goal = goal
        self.planner.setEndpoints(start, goal)
        self.path = []
        self.G = None

    def keyboardfunc(self, key, x, y):
        if key == ' ':
            if self.optimizingPlanner or not self.path:
                print "Planning 1..."
                self.planner.planMore(1)
                self.path = self.planner.getPath()
                self.G = self.planner.getRoadmap()
                self.refresh()
        elif key == 'p':
            if self.optimizingPlanner or not self.path:
                print "Planning 100..."
                self.planner.planMore(100)
                self.path = self.planner.getPath()
                self.G = self.planner.getRoadmap()
                self.refresh()

    def display(self):
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        mB = max([y for x, y in self.space.bound])
        glOrtho(0, mB, mB, 0, -1, 1)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()

        glDisable(GL_LIGHTING)
        self.space.drawObstaclesGL()
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
            self.space.drawRobotGL(self.start)
            self.space.drawRobotGL(self.goal)

        if self.G:
            #draw graph
            V, E = self.G
            glEnable(GL_BLEND)
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
            glColor4f(0, 0, 0, 0.5)
            glPointSize(3.0)
            glBegin(GL_POINTS)
            for v in V:
                glVertex2f(v[0], v[1])
            glEnd()
            glColor4f(0.5, 0.5, 0.5, 0.5)
            glBegin(GL_LINES)
            for (i, j) in E:
                glVertex2f(V[i][0], V[i][1])
                glVertex2f(V[j][0], V[j][1])
            glEnd()
            glDisable(GL_BLEND)

        self.space.drawMinkGL()


if __name__ == '__main__':
    space = None
    start = None
    goal = None
    space = DiskCSpace(50)
    space.addObstacle(Circle(700, 500, 120))
    space.addObstacle(Rectangle(295, 400, 5, 300))
    space.addObstacle(Rectangle(295, 400, 300, 5))
    space.addObstacle(Rectangle(595, 700, -300, -5))
    space.addObstacle(Rectangle(595, 700, -5, -300))
    start = (150, 150)
    goal = (850, 850)
    program = DiskCSpaceObstacleProgram(space, start, goal)
    program.view.w = program.view.h = 1080
    program.name = "Motion planning test"
    program.run()
