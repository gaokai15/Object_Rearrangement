from math import *
from collections import deque

import numpy as np
import Polygon as pn
import Polygon.Utils as pu


def norm(u):
    return sqrt(sum(np.power(u, 2)))


def dist(a, b):
    return norm(np.subtract(a, b))


def polysCollide(poly1, poly2):
    cpoly1 = pn.Polygon(poly1)
    cpoly2 = pn.Polygon(poly2)
    return cpoly1.overlaps(cpoly2)


def poly_disk(center, radius, resolution=1):
    poly = []
    for i in range(360, 0, -resolution):
        rad = radians(i)
        x = center[0] + radius * cos(rad)
        y = center[1] + radius * sin(rad)
        poly.append([x, y])
    return poly


def bfs(tree, start, goal):
    path = []
    backtrace = {start: start}
    explore = deque([start])
    while goal not in backtrace:
        try:
            p = explore.pop()
        except Exception:
            return []
        for c in tree[p]:
            if c not in backtrace:
                backtrace[c] = p
                explore.append(c)

    path.append(goal)
    while backtrace[path[-1]] != path[-1]:
        path.append(backtrace[path[-1]])
    path.reverse()

    return path
