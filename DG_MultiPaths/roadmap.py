from random import uniform  # , choice
from util import *


class Roadmap(object):
    def __init__(
        self, height, width, robot, obstacles=None, edgeCollider=None, pointCollider=None
    ):
        self.height = height
        self.width = width
        self.adjListMap = {}
        self.points = {}
        self.robot = robot
        self.obstacles = obstacles
        self.edgeCollider = edgeCollider
        if edgeCollider is None:
            self.notEdgeCollides = lambda u, v: True
        else:
            self.notEdgeCollides = lambda u, v: edgeCollider(self.robot, (u, v), self.obstacles)
        self.pointCollider = pointCollider
        if pointCollider is None:
            self.notCollides = lambda u: True
        else:
            self.notCollides = lambda u: pointCollider(self.robot, u, self.obstacles)

    def copyBut(self, obstacles, edgeCollider, pointCollider):
        print(obstacles)
        print(self.obstacles)
        toRet = Roadmap(
            self.height, self.width, self.robot, obstacles, edgeCollider, pointCollider
        )
        toRet.adjListMap = self.adjListMap.copy()
        toRet.points = self.points.copy()
        return toRet

    def addPoint(self, point):
        index = len(self.points) + 1
        self.points[index] = point
        if index not in self.adjListMap:
            self.adjListMap[index] = []
        return index

    def point(self, index):
        return self.points[index]

    def addEdge(self, index1, index2):
        self.adjListMap[index1].append(index2)
        self.adjListMap[index2].append(index1)

    def delEdge(self, index1, index2):
        self.adjListMap[index1].remove(index2)

    def getUndirected(self):
        retAdjListMap = self.adjListMap.copy()
        # for u, e in retAdjListMap.items():
        #     print("adj: ", len(e))
        #     for v in e:
        #         print("1:", u, v)
        #         retAdjListMap[v].append(u)
        #         print("2:", v, u)

        retPoints = self.points.copy()

        return retPoints, retAdjListMap

    def r(self, gamma):
        n = len(self.points)
        return gamma * (log(n) / n)**2

    def tryGrow(self, gamma=100, r=r, tryPoint=None):
        if tryPoint is None:
            tryPoint = (uniform(0, self.width), uniform(0, self.height))

        if self.notCollides(tryPoint):
            v = self.addPoint(tryPoint)
            for u, pu in self.points.items():
                if dist(tryPoint, pu) < r(self, gamma):
                    if self.notEdgeCollides(tryPoint, pu):
                        self.addEdge(u, v)

            return v

        return False
