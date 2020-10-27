from math import *
from collections import deque

import numpy as np
import Polygon as pn
import Polygon.Utils as pu


def norm(u):
    return sqrt(sum(np.power(u, 2)))


def dist(a, b):
    return norm(np.subtract(a, b))


### find the bounding box for a polygon ###
def bbox(poly):
    xmin = np.Inf
    xmax = -np.Inf
    ymin = np.Inf
    ymax = -np.Inf
    for p in poly:
        xmin = min(xmin, p[0])
        xmax = max(xmax, p[0])
        ymin = min(ymin, p[1])
        ymax = max(ymax, p[1])
    return xmin, xmax, ymin, ymax


### build the shape of the object based on resolution ###
def poly_disk(center, radius, resolution):
    poly = []
    for i in range(360, 0, -resolution):
        rad = radians(i)
        x = center[0] + radius * cos(rad)
        y = center[1] + radius * sin(rad)
        poly.append([x, y])
    return poly


### check if two polygons overlap ###
def polysCollide(poly1, poly2):
    cpoly1 = pn.Polygon(poly1)
    cpoly2 = pn.Polygon(poly2)
    return cpoly1.overlaps(cpoly2)


### collision checker2 ###
def collisionCheck(objects):
    for i in range(len(objects) - 1):
        for poly in objects[i + 1:]:
            if polysCollide(poly, objects[i]):
                return False
    return True


### collision checker ###
def isCollisionFree(object_shape, center, obstacles):
    object_location = np.add(center, object_shape)
    ### Now check if the new object collides with all other objects (as obstacles)
    for poly in obstacles:
        if polysCollide(poly, object_location):
            return False

    return True


### collision checker with buffers ###
def countNumOverlap(object_shape, center, obstacles, buffer_obs, numOverlapAllowed):
    buffer_polygon = np.add(center, object_shape)
    ### Now check if the new buffer collides with all other objects (as obstacles)
    numOverlap = 0
    for poly in obstacles:
        if polysCollide(poly, buffer_polygon):
            numOverlap += 1
        if (numOverlap > numOverlapAllowed):
            return numOverlap
    for poly in buffer_obs:
        if polysCollide(poly, buffer_polygon):
            numOverlap += 3
        if (numOverlap > numOverlapAllowed):
            return numOverlap
    ### reach here since numOverlap <= numOverlapAllowed
    return numOverlap


### BFS search on the connectivity graph ###
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


def BFS(graphAdj, start, goal, condition=lambda x: True):
    path = []
    backtrace = {start: start}
    explore = deque([start])
    while goal not in backtrace:
        if len(explore) > 0:
            u = explore.pop()
        else:
            return []
        for v in filter(condition, graphAdj[u]):
            if v not in backtrace:
                backtrace[v] = u
                explore.appendleft(v)

    path.append(goal)
    while backtrace[path[-1]] != path[-1]:
        path.append(backtrace[path[-1]])
    path.reverse()

    return path


def checkBitStatusAtPos(n, k):
    #This code is contributed by Gitanjali (GeeksforGeeks)
    new_num = n >> k
    return (new_num & 1)
