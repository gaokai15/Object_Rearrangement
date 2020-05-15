from math import *
from time import time
from itertools import combinations

import numpy as np
import Polygon as pn
import Polygon.Utils as pu
import Polygon.Shapes as ps
import visilibity as vis

epsilon = pn.getTolerance()


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


def objects2regions(minkowski_objs, mink_boundary, regions={}):
    """
    minkowski_objs = the object discs after a minkowski sum
    mink_boundary = the boundary of the configuration space
    regions = optionaly pass previous structure to build on top of
    """
    polysum = sum(regions.values(), pn.Polygon())
    for i, obj in enumerate(minkowski_objs):
        for rind, r in regions.items():
            rANDobj = r & obj
            if rANDobj:
                regions[rind + (i, )] = rANDobj
                rDIFobj = r - obj
                if rDIFobj:
                    regions[rind] = rDIFobj
                else:
                    del regions[rind]

        objDIFFpolysum = mink_boundary & obj - polysum
        # print(objDIFFpolysum, bool(objDIFFpolysum))
        if objDIFFpolysum:
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
    regions = ref_regions.copy()
    point2regs = {}
    for rind, r in regions.items():
        # if len(r) > 1:
        char = 'a'
        for cont in r:
            poly = pn.Polygon(cont)
            rind_n = rind + (char, )
            regions[rind_n] = poly
            char = chr(ord(char) + 1)
            for i, p in enumerate(points):
                if poly.isInside(*p):
                    point2regs[i] = rind_n

        del regions[rind]

    cfree = mink_boundary - polysum
    for i, pfree in enumerate(pu.fillHoles(cfree), 1):
        r = pn.Polygon(pfree)
        for isHole, cont in zip(cfree.isHole(), cfree):
            if isHole: r -= pn.Polygon(cont)
        regions[(-i, )] = r

    paths = []
    for rkv1, rkv2 in combinations(regions.items(), 2):
        rind1, r1 = rkv1
        rind2, r2 = rkv2

        s1 = set(rind1[:-1])
        s2 = set(rind2[:-1])
        s1vs2 = s1.intersection(s2)

        if s1 and s2 and not s1vs2:
            continue
        if prune_dist != 0:
            if len(s1 ^ s2) > abs(prune_dist):
                if s1 and s2 or prune_dist > 0:
                    continue

        interR = set(pu.pointList(r1)) & set(pu.pointList(r2))
        # print(rind1, r1, rind2, r2)
        if len(interR) > 0:
            paths.append((rind1, rind2))

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
    if r1.isInside(*r1.center()):
        pstart = r1.center()
    else:
        pstart = r1.sample(seed)

    if r2.isInside(*r2.center()):
        pgoal = r2.center()
    else:
        pgoal = r2.sample(seed)

    rect = ps.Rectangle(dist(pstart, pgoal), 1)
    vsg = np.subtract(pgoal, pstart)
    rect.rotate(np.angle(vsg[0] + 1j * vsg[1]))
    rect.warpToBox(*bbox([pstart, pgoal]))
    r1Ar2 = r1 + r2
    hasDirectPath = (r1Ar2).covers(rect) if len(r1Ar2) == 1 else False

    if debug:
        debug(rect, r1, r2)
        print('Direct?: ', hasDirectPath)
        print(r1.area(), r2.area())

    if not hasDirectPath:
        interR = set(pu.pointList(r1)) & set(pu.pointList(r2))
        interR = min(interR, key=lambda x: dist(pstart, x) + dist(x, pgoal))

        # Start to boundary
        wall_mink_poly = pu.fillHoles(r1)

        env_polys_vis = [vis.Polygon([vis.Point(*p) for p in reversed(pu.pointList(wall_mink_poly))])]
        for isHole, cont in zip(r1.isHole(), r1):
            if isHole: env_polys_vis += [vis.Polygon([vis.Point(*p) for p in reversed(cont)])]
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
        wall_mink_poly = pu.fillHoles(r2)

        env_polys_vis = [vis.Polygon([vis.Point(*p) for p in reversed(pu.pointList(wall_mink_poly))])]
        for isHole, cont in zip(r2.isHole(), r2):
            if isHole: env_polys_vis += [vis.Polygon([vis.Point(*p) for p in reversed(cont)])]
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
