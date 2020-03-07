from random import uniform  # , choice
from util import *


class RRTree(object):
    def __init__(
        self, height, width, robot=None, obstacles=None, startPoint=None, edgeCollider=None
    ):
        self.height = height
        self.width = width
        self.adjListMap = {}
        self.points = {}
        if startPoint is not None:
            self.addPoint(startPoint)
        self.robot = robot
        self.obstacles = obstacles
        if edgeCollider is None:
            self.notEdgeCollides = lambda u, v: True
        else:
            self.notEdgeCollides = lambda u, v: edgeCollider(self.robot, (u, v), self.obstacles)

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

    def delEdge(self, index1, index2):
        self.adjListMap[index1].remove(index2)

    def getUndirected(self):
        retAdjListMap = self.adjListMap.copy()
        for u, e in retAdjListMap.items():
            for v in e:
                retAdjListMap[v].append(u)

        retPoints = self.points.copy()

        return retPoints, retAdjListMap

    def tryGrow(self, radius=None, tryPoint=None):
        if tryPoint is None:
            tryPoint = (uniform(0, self.width), uniform(0, self.height))

        if len(self.points) == 0:
            if self.notEdgeCollides(tryPoint, tryPoint):
                self.addPoint(tryPoint)
                return tuple(tryPoint)

        dmin = np.inf
        emin = None
        pmin = 1
        for u, e in self.adjListMap.items():
            for v in e:
                d, dp = dist_to_segment(tryPoint, self.point(u), self.point(v))
                if d < dmin:
                    dmin = d
                    emin = [u, v]
                    pmin = dp

        if isinstance(pmin, tuple):
            # if radius is not None:
            #     tryPoint = uniform(0.1, radius) * np.subtract(tryPoint, pmin
            #                                                   ) / dist(tryPoint, pmin) + pmin
            if self.notEdgeCollides(pmin, tryPoint):
                npt = self.addPoint(pmin)
                self.delEdge(emin[0], emin[1])
                self.addEdge(emin[0], npt)
                self.addEdge(npt, emin[1])
                if dmin > 0:
                    self.addEdge(npt, self.addPoint(tryPoint))
                return tuple(tryPoint)
        elif isinstance(pmin, int):
            if emin is None:
                src = pmin
            else:
                src = emin[pmin]
            # if radius is not None:
            #     tryPoint = uniform(0.1, radius) * np.subtract(tryPoint,
            #                                                   src) / dist(tryPoint, src) + src
            if self.notEdgeCollides(self.point(src), tryPoint):
                self.addEdge(src, self.addPoint(tryPoint))
                return tuple(tryPoint)
        else:
            print("Error!", pmin, type(pmin), file=sys.stderr)
            exit(-1)

        return False
