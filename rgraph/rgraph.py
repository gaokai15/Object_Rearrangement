from math import *
from time import time
from random import random
from itertools import combinations, chain

import numpy as np
import Polygon as pn
import visilibity as vis
from region import region

epsilon = 2**-8


def norm(u):
    return sqrt(sum(np.power(u, 2)))


def dist(a, b):
    return norm(np.subtract(a, b))


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


def regionsCollide(r1, r2):
    cr1 = region(r1, True)
    cr2 = region(r2, True)
    return not cr1.intersection(cr2).is_empty()


def poly_disk(center, radius, resolution=1):
    poly = []
    for i in range(0, 360, resolution):
        rad = radians(i)
        x = center[0] + radius * cos(rad)
        y = center[1] + radius * sin(rad)
        poly.append([int(x), int(y)])
        # poly.append([x, y])
    return poly


def objects2regions(minkowski_objs, mink_boundary, regions={}):
    """
    minkowski_objs = the object discs after a minkowski sum
    mink_boundary = the boundary of the configuration space
    regions = optionaly pass previous structure to build on top of
    """
    polysum = sum(regions.values(), region())
    for i, obj in enumerate(minkowski_objs):
        for rind, r in regions.items():
            rANDobj = r & obj
            if not rANDobj.is_empty():
                regions[rind + (i, )] = rANDobj
                rDIFobj = r - obj
                if rDIFobj.is_empty():
                    del regions[rind]
                else:
                    regions[rind] = rDIFobj

        objDIFFpolysum = mink_boundary & obj - polysum
        if not objDIFFpolysum.is_empty():
            regions[(i, )] = objDIFFpolysum
        polysum += obj
    return regions, polysum


def regions2graph(ref_regions, mink_boundary, polysum, points=[], prune_dist=0):
    """
    ref_regions = regions dictionary to create a graph from
    mink_boundary = the boundary of the configuration space
    points = a set of points to query for region containment
    prune_dist = 0 if you don't want to prune edges
                >0 the hamming distance you want to prune
                <0 will prune using absolute value as the hamming
                   distance but always connect to the free space
    """
    t0 = time()
    regions = ref_regions.copy()
    point2regs = {}
    for rind, r in regions.items():
        if r.is_empty():
            print("Empty: ", rind)
        char = 'a'
        for comp in r.get_components():
            rind_n = rind + (char, )
            regions[rind_n] = comp
            char = chr(ord(char) + 1)
            for i, p in enumerate(points):
                if comp.contains(*p):
                    point2regs[i] = rind_n

        del regions[rind]

    cfree = mink_boundary - polysum
    for i, rfree in enumerate(cfree.get_components(), 1):
        regions[(-i, )] = rfree
    print("Comps Time: ", time() - t0)

    t0 = time()
    paths = []
    for rkv1, rkv2 in combinations(regions.items(), 2):
        rind1, r1 = rkv1
        rind2, r2 = rkv2

        # t1 = time()
        s1 = set(rind1[:-1])
        s2 = set(rind2[:-1])
        s1vs2 = s1.intersection(s2)

        if s1 and s2 and not s1vs2:
            continue
        if prune_dist != 0:
            if len(s1 ^ s2) > abs(prune_dist):
                if s1 and s2 or prune_dist > 0:
                    continue
        # print("Prune Time: ", time() - t1)

        # interR = set(chain(*r1.to_list())) & set(chain(*r2.to_list()))
        # r1Ur2 = r1 + r2
        # t0 = time()
        # maybe = r1Ur2.disconnected()
        # print("Maybe0: ", time() - t0)

        # t0 = time()
        # print(len(s1 & s2), (s1 & s2 == s1 and s1 | s2 == s2) or (s1 & s2 == s2 and s1 | s2 == s1))
        # print("Maybe1: ", time() - t0)

        # # print(rind1, rind2, r1 == r2)
        # if maybe != (len(interR) == 0):
        #     print(maybe, interR)
        #     if len(interR) == 0:
        #         print(r1.to_list())
        #         print(r2.to_list())
        # t0 = time()
        # print("num", r1Ur2.num_connected_components() <= r1.num_connected_components() + r2.num_connected_components())
        # print(time() - t0)

        # t0 = time()
        # print("maybe", r1Ur2.disconnected())
        # print(time() - t0)

        # if s1 < s2 or s2 < s1:
        #     print(len(s1 ^ s2) - len(s1 & s2), s1, s2, s1 ^ s2, s1 & s2, s1 | s2)

        ### Run this on dense case for testing
        # if not ((not (r1.closure() & r2.closure()).is_empty() and not maybe) == (len(r1Ur2.get_components()) == 1)):
        #     print((r1.closure() & r2.closure()).is_empty())
        #     print(maybe)
        #     print(len(r1Ur2.get_components()) == 1)
        #     print(str(r1))
        #     print(str(r2))
        #     print(r1.to_list(), r2.to_list())

        # t1 = time()
        if not (r1.closure() & r2.closure()).is_empty():
            # if len(r1Ur2.get_components()) == 1:
            if not (r1 + r2).disconnected():
                paths.append((rind1, rind2))
        # print("Connect Time: ", time() - t1)

    print("Connect Edges Time: ", time() - t0)
    t0 = time()

    graph = {}
    for u, v in paths:
        graph[u] = sorted(graph.get(u, []) + [v])
        graph[v] = sorted(graph.get(v, []) + [u])

    if points:
        return graph, regions, point2regs
    else:
        return graph, regions


def o55():
    return 0.55


def regionPath(r1, r2, seed=o55, debug=None):
    lr1 = r1.to_list()
    pr1 = pn.Polygon(lr1[0]) - sum([pn.Polygon(l) for l in lr1[1:]], pn.Polygon())
    if pr1.area() > 0:
        if pr1.isInside(*pr1.center()):
            pstart = pr1.center()
        else:
            pstart = pr1.sample(seed)
    else:
        pstart = lr1[0][len(lr1[0]) // 2]

    lr2 = r2.to_list()
    pr2 = pn.Polygon(lr2[0]) - sum([pn.Polygon(l) for l in lr2[1:]], pn.Polygon())
    if pr2.area() > 0:
        if pr2.isInside(*pr2.center()):
            pgoal = pr2.center()
        else:
            pgoal = pr2.sample(seed)
    else:
        pgoal = lr2[0][len(lr2[0]) // 2]

    line = region([list(pstart), list(pgoal)], False)
    r1Ar2 = r1 + r2
    hasDirectPath = line < r1Ar2

    if debug:
        debug(line, r1, r2)
        print('Direct?: ', hasDirectPath)
        print(pr1.area(), pr2.area())
        print(r1Ar2.get_components())

    if not hasDirectPath:
        interR = set(chain(*r1.to_list())) & set(chain(*r2.to_list()))
        if len(interR) == 0:
            interR = set(chain(*r1.to_list())).union(set(chain(*r2.to_list())))
        interR = min(interR, key=lambda x: dist(pstart, x) + dist(x, pgoal))

        # Start to boundary
        wall_mink_poly = r1  # .regularization()
        wall_mink_list = r1.to_list()

        env_polys_vis = [vis.Polygon([vis.Point(*p) for p in wall_mink_list[0]])]
        for hole in wall_mink_list[1:]:
            env_polys_vis.append(vis.Polygon([vis.Point(*p) for p in reversed(hole)]))
        env = vis.Environment(env_polys_vis)
        if not env.is_valid(epsilon):
            if debug:
                debug(wall_mink_poly)

        start = vis.Point(*pstart)
        goal = vis.Point(*interR)

        start.snap_to_boundary_of(env, epsilon)
        start.snap_to_vertices_of(env, epsilon)

        t0 = time()
        ppath = env.shortest_path(start, goal, epsilon)
        print(time() - t0)

        path = [(p.x(), p.y()) for p in ppath.path()]

        # Boundary to goal
        wall_mink_poly = r2  # .regularization()
        wall_mink_list = r2.to_list()

        env_polys_vis = [vis.Polygon([vis.Point(*p) for p in wall_mink_list[0]])]
        for hole in wall_mink_list[1:]:
            env_polys_vis.append(vis.Polygon([vis.Point(*p) for p in reversed(hole)]))
        env = vis.Environment(env_polys_vis)
        if not env.is_valid(epsilon):
            if debug:
                debug(wall_mink_poly)

        start = vis.Point(*interR)
        goal = vis.Point(*pgoal)

        start.snap_to_boundary_of(env, epsilon)
        start.snap_to_vertices_of(env, epsilon)

        t0 = time()
        ppath = env.shortest_path(start, goal, epsilon)
        print(time() - t0)

        path += [(p.x(), p.y()) for p in ppath.path()][1:]
    else:
        path = [pstart, pgoal]

    return path
