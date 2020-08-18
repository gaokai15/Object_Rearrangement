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
    def __init__(self, x=0, y=0, radius=1):
        self.center = (x, y)
        self.radius = radius
        self.type = 'Circle'

    def contains(self, point):
        return (vectorops.distance(point, self.center) <= self.radius)

    def poly(self, res=20):
        pnts = []
        numdivs = int(math.ceil(self.radius * math.pi * 2 / res))
        for i in xrange(numdivs + 1):
            u = float(i) / float(numdivs) * math.pi * 2
            pnts.append((self.center[0] + self.radius * math.cos(u), self.center[1] + self.radius * math.sin(u)))
        return pnts

    def drawGL(self, res=20):
        numdivs = int(math.ceil(self.radius * math.pi * 2 / res))
        glBegin(GL_TRIANGLE_FAN)
        glVertex2f(*self.center)
        for i in xrange(numdivs + 1):
            u = float(i) / float(numdivs) * math.pi * 2
            glVertex2f(self.center[0] + self.radius * math.cos(u), self.center[1] + self.radius * math.sin(u))
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
        # glVertex2f(self.center[0] - self.width / 2.0, self.center[1] - self.height / 2.0)
        # glVertex2f(self.center[0] - self.width / 2.0, self.center[1] + self.height / 2.0)
        # glVertex2f(self.center[0] + self.width / 2.0, self.center[1] + self.height / 2.0)
        # glVertex2f(self.center[0] + self.width / 2.0, self.center[1] - self.height / 2.0)
        glVertex2f(self.center[0], self.center[1])
        glVertex2f(self.center[0], self.center[1] + self.height)
        glVertex2f(self.center[0] + self.width, self.center[1] + self.height)
        glVertex2f(self.center[0] + self.width, self.center[1])
        glEnd()


class Poly:
    def __init__(self, points):
        self.points = list(sorted(points, key=lambda x: abs(pc.Area(x))))
        self.type = 'Poly'

    def drawGL(self):
        for cont in self.points:
            if pc.Orientation(cont):
                glColor3f(0.1, 0.5, 0.1)
            else:
                glColor3f(0.5, 0.1, 0.1)
            glBegin(GL_POLYGON)
            for p in cont:
                glVertex2f(*p)
            glEnd()


class DiskCSpace(CSpace):
    def __init__(self):
        CSpace.__init__(self)
        #set bounds
        self.bound = [(0, 1000), (0, 1000)]
        self.wall = [
            (self.bound[0][0], self.bound[1][0]),
            (self.bound[0][1], self.bound[1][0]),
            (self.bound[0][1], self.bound[1][1]),
            (self.bound[0][0], self.bound[1][1]),
        ]
        #set collision checking resolution
        self.eps = 1
        #setup a robot with radius 10
        self.robot = Circle(0, 0, 10)
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
        clipM = pc.Pyclipper()
        for o in self.obstacles:
            # clip.AddPaths(pc.MinkowskiSum(shape, o.poly(), True), pc.PT_CLIP, True)
            for cont in pc.MinkowskiSum(shape, o.poly(), True):
                if pc.Orientation(cont):
                    clipM.AddPath(cont, pc.PT_SUBJECT, True)
                else:
                    clipM.AddPath(cont, pc.PT_CLIP, True)
            clip.AddPaths(clipM.Execute(pc.CT_DIFFERENCE, pc.PFT_POSITIVE, pc.PFT_POSITIVE), pc.PT_CLIP, True)

        all_obs = clip.Execute(pc.CT_UNION, pc.PFT_POSITIVE, pc.PFT_POSITIVE)
        clip.Clear()
        clip.AddPaths(all_obs, pc.PT_CLIP, True)
        clip.AddPath(self.wall, pc.PT_SUBJECT, True)
        mink_obs = clip.Execute(pc.CT_DIFFERENCE, pc.PFT_POSITIVE, pc.PFT_POSITIVE)
        self.mink_obs = Poly(mink_obs)

    def addOccupied(self, obj):
        self.occupied.append(obj)

    def setOccupied(self, occ):
        self.occupied = occ

    def clearOccupied(self, obj):
        self.occupied = []

    def feasible(self, q):
        #bounds test
        for wp in self.wall:
            qo = vectorops.add(q, vectorops.mul(vectorops.unit(vectorops.sub(wp, q)), self.robot.radius))
            if not CSpace.feasible(self, qo): return False

        for o in self.obstacles + self.occupied:
            if o.type == 'Circle':
                qo = vectorops.add(q, vectorops.mul(vectorops.unit(vectorops.sub(o.center, q)), self.robot.radius))
                if o.contains(qo): return False
            elif o.type == 'Rectangle':
                qo = vectorops.add(q, vectorops.mul(vectorops.unit(vectorops.sub(o.center, q)), self.robot.radius))
                if o.contains(qo): return False
                ocenter = (o.center[0], o.center[1] + o.height)
                qo = vectorops.add(q, vectorops.mul(vectorops.unit(vectorops.sub(ocenter, q)), self.robot.radius))
                if o.contains(qo): return False
                ocenter = (o.center[0] + o.width, o.center[1] + o.height)
                qo = vectorops.add(q, vectorops.mul(vectorops.unit(vectorops.sub(ocenter, q)), self.robot.radius))
                if o.contains(qo): return False
                ocenter = (o.center[0] + o.width, o.center[1])
                qo = vectorops.add(q, vectorops.mul(vectorops.unit(vectorops.sub(ocenter, q)), self.robot.radius))
            else:
                print("Error! Invalid Obstacle Type")
                sys.exit(-1)
        return True

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

    def regions(self):
        regions = {}
        polysum = pn.Polygon()
        for i, obj in enumerate(all_minkowskis):
            for rind, r in regions.items():
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
            perturbationRadius=25,
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
    space = DiskCSpace()
    space.setRobotRad(50)
    space.addObstacle(Circle(500, 500, 100))
    space.addObstacle(Rectangle(295, 400, 5, 300))
    space.addObstacle(Rectangle(295, 400, 300, 5))
    space.addObstacle(Rectangle(595, 700, -300, -5))
    space.addObstacle(Rectangle(595, 700, -5, -300))
    space.computeMinkObs()
    start = (150, 150)
    goal = (850, 850)
    program = DiskCSpaceObstacleProgram(space, start, goal)
    program.view.w = program.view.h = 1080
    program.name = "Motion planning test"
    program.run()
