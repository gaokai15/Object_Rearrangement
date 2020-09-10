from __future__ import print_function
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from klampt.plan.cspace import CSpace, MotionPlan
from klampt.vis.glprogram import GLProgram
from klampt.math import vectorops

import sys
from time import time
from itertools import combinations
from random import random, seed, choice

import Polygon as pn
import pyclipper as pc

from util import *


def interpolate(a, b, u):
    """Interpolates linearly between a and b"""
    return vectorops.madd(a, vectorops.sub(b, a), u)


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
                pnts.append(
                    (
                        int(self.center[0] + self.radius * cos(u)),
                        int(self.center[1] + self.radius * sin(u)),
                    )
                )
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
            return pc.PointInPolygon(point, self.points)

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

    def sample(self):
        sampled = (0, 0)
        if self.type == 'C_Poly':
            poly = pn.Polygon()
            for cont in self.points:
                if pc.Orientation(cont):
                    poly += pn.Polygon(cont)
                else:
                    poly -= pn.Polygon(cont)
            else:
                sampled = poly.sample(random)
        elif self.type == 'S_Poly':
            poly = pn.Polygon(self.points)
            sampled = poly.sample(random)
        return (int(sampled[0]), int(sampled[1]))

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

    def removePose(self, oid, obj):
        return self.poseMap.pop(oid, False)

    def setPoses(self, occ):
        self.poseMap = occ

    def clearPoses(self, obj):
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

    def drawRobotGL(self, q):
        glColor3f(0, 0, 1)
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

    def drawRegionGraphGL(self, drawRegions=True):
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

    def regionGraph(self):
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
                    # print(i, p)
                    if pc.PointInPolygon(p.center, cont):
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
        self.drawRegions = True

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
        if key == 'r':
            self.drawRegions = not self.drawRegions

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
        self.space.drawRegionGraphGL(self.drawRegions)
        if not self.drawRegions:
            self.space.drawPoses()
        self.space.drawMinkGL()


def loadEnv(filename):
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
            try:
                ### try to sample point
                point = space.mink_obs.sample()
            except:
                if j == 0:
                    print("failed to generate the start of object " + str(i))
                else:
                    print("failed to generate the goal of object " + str(i))
                print("FAIL TO GENERATE THE INITIAL INSTANCE")
                return False

            ### Congrats the object's goal/start is accepted
            space.addPose(sorg + str(i), Circle(point[0], point[1], space.robot.radius))
            # space.addPose(2 * i + j, Circle(point[0], point[1], space.robot.radius))

    ### restore obstacles
    space.restoreObstacles(staticObstacles)
    space.computeMinkObs()
    return True


# def genBuffers(n, space=DiskCSpace(), maxOverlap, method='random'):
def genBuffers(n, space, maxOverlap, method='random'):
    staticObstacles = space.saveObstacles()

    ### Random Generation ###
    ### The idea is we randomly generate a buffer in the space, hoping it will
    ### overlap with nothing.  If it could not achieve after several trials, we
    ### increment the number of object poses it can overlap.  We keep incrementing
    ### until we find enough buffers.
    if method == 'random':
        # V, E = space.RG
        numOverlapAllowed = 0
        for i in range(n):
            isValid = False
            timeout = 500
            while not isValid and timeout > 0:
                timeout -= 1
                ### try to sample point
                numOverlap = 0
                # print(space.obstacles)
                # print(space.mink_obs.points)
                point = space.mink_obs.sample()
                for p in space.poseMap.values():
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
                    return

            ### Otherwise the buffer is accepted
            space.addPose('B' + str(i), Circle(point[0], point[1], space.robot.radius))
            # space.addPose(len(space.poseMap) + 1, Circle(point[0], point[1], space.robot.radius))
            # print(space.poseMap)

    ### Hueristic Generation ###
    elif method == 'simple_heuristic':

        # b_points = set()
        # if space.mink_obs.type == 'S_Poly':
        #     for x in space.mink_obs.points:
        #         b_points.update(x)
        # elif space.mink_obs.type == 'C_Poly':
        #     for x in space.mink_obs.points:
        #         for y in x:
        #             b_points.update(x)
        # print(b_points)

        # numBuffers = len(b_points)
        for i in range(n):
            # point = choice(list(b_points))
            # b_points.remove(point)
            point = polysum.sample(random)
            buffer_points.append(point)
            buffers.append(pn.Polygon(polygon + point))
            mink_obj = 2 * polygon + point  ### grown_shape buffer
            minkowski_buffers.append(pn.Polygon(mink_obj))

    ### Better Hueristic Generation ###
    elif method == 'better_heuristic':
        numBuffPerObj = 1
        obj_ind = range(len(mink_objs))
        for si, gi in zip(obj_ind[::2], obj_ind[1::2]):
            ind_obs = set(obj_ind) - set([si, gi])
            polysum = wall_mink - sum([mink_objs[x] for x in ind_obs], pn.Polygon())
            for i in range(numBuffPerObj):
                point = polysum.sample(random)
                buffer_points.append(point)
                buffers.append(pn.Polygon(polygon + point))
                mink_obj = 2 * polygon + point  ### grown_shape buffer
                minkowski_buffers.append(pn.Polygon(mink_obj))

        return buffer_points, buffers, minkowski_buffers

    ### reset obstacles
    space.restoreObstacles(staticObstacles)
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
            rad = space.robot.radius

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

    space.regionGraph()
    genBuffers(5, space, 4)
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