from __future__ import division

from time import time
from random import uniform, random
from itertools import combinations
import numpy as np
import visilibity as vis
import Polygon as pn
import Polygon.Utils as pu
import Polygon.Shapes as ps

from util import *


class RegionGraphGenerator(object):
    def __init__(self, instance, visualTool, wall_mink):
        self.epsilon = 2**-8
        self.wall_mink = wall_mink
        self.regions, self.obj2reg = self.genRegionGraph(instance, visualTool)
        self.paths = self.connectRegionGraph(instance, visualTool)
        self.graph = self.getGraphFromPaths()

        ### After cgraph generation, let's see if we want to display or save it
        ### region graph without connections
        # visualTool.drawRegionGraph({}, self.regions.values(), label=False)
        # ### region graph with connections
        # visualTool.drawRegionGraph(self.paths, self.regions.values(), label=False)
        # ### connectivity graph
        # visualTool.drawConGraph(self.paths, instance.points, instance.objects, instance.buffers)

    def genRegionGraph(self, instance, visualTool):
        ### This function generates a graph with different regions, specifying neighboring regions
        all_points = instance.points + instance.buffer_points
        # print(len(all_points))
        all_minkowskis = instance.minkowski_objs + instance.minkowski_buffers
        # print(len(all_minkowskis))

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

            objDIFFpolysum = self.wall_mink & obj - polysum
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

                try:
                    if poly.isInside(*poly.center()):
                        regions[rind_n] = (poly, poly.center())
                    else:
                        regions[rind_n] = (poly, poly.sample(random))
                except pn.cPolygon.Error:
                    # print(cont)
                    regions[rind_n] = (poly, cont[len(cont) // 2])

                for i, p in enumerate(all_points):
                    if (p[0], p[1]) in cont:  # buffer gen fix
                        print("AHA", i, len(all_points))
                        # obj2reg[i] = rind_n
                    if poly.isInside(*p):
                        obj2reg[i] = rind_n

            del regions[rind]

        cfree = self.wall_mink - polysum
        for i, pfree in enumerate(pu.fillHoles(cfree), 1):
            r = pn.Polygon(pfree)
            for isHole, cont in zip(cfree.isHole(), cfree):
                if isHole: r -= pn.Polygon(cont)
            if r.isInside(*r.center()):
                regions[(-i, )] = (r, r.center())
            else:
                regions[(-i, )] = (r, r.sample(random))

        return regions, obj2reg

    def connectRegionGraph(self, instance, visualTool):
        ### This function figures out the connectivity between different regions
        paths = {}

        for rkv1, rkv2 in combinations(self.regions.items(), 2):
            rind1, r1 = rkv1
            rind2, r2 = rkv2
            # print(rind1, r1, rind2, r2)
            if rind1[0] > 0 and rind2[0] > 0:
                s1 = set(rind1[:-1])
                s2 = set(rind2[:-1])
                if len(s1 - s2) + len(s2 - s1) > 1:
                    continue

            r1, pstart = r1
            r2, pgoal = r2
            r1Ar2 = r1 + r2
            pointStart = pstart + instance.polygon * 0.1
            pointGoal = pgoal + instance.polygon * 0.1

            interR = set(pu.pointList(r1)) & set(pu.pointList(r2))
            if len(interR) > 0:
                rect = ps.Rectangle(dist(pstart, pgoal), 1)
                vsg = np.subtract(pgoal, pstart)
                rect.rotate(np.angle(vsg[0] + 1j * vsg[1]))
                rect.warpToBox(*bbox([pstart, pgoal]))
                hasDirectPath = (r1Ar2).covers(rect) if len(r1Ar2) == 1 else False

                # # collides = False
                if visualTool.display and not hasDirectPath:
                    interR = min(interR, key=lambda x: dist(pstart, x) + dist(x, pgoal))
                    wall_mink_poly = pu.fillHoles(r1)
                    env_polys_vis = [vis.Polygon([vis.Point(*p) for p in reversed(pu.pointList(wall_mink_poly))])]
                    for isHole, cont in zip(r1.isHole(), r1):
                        if isHole: env_polys_vis += [vis.Polygon([vis.Point(*p) for p in reversed(cont)])]
                    env = vis.Environment(env_polys_vis)
                    if not env.is_valid(self.epsilon):
                        print("there is error caused by self.epsilon validity")

                    start = vis.Point(*pstart)
                    goal = vis.Point(*interR)

                    start.snap_to_boundary_of(env, self.epsilon)
                    start.snap_to_vertices_of(env, self.epsilon)

                    t0 = time()
                    ppath = env.shortest_path(start, goal, self.epsilon)
                    # print(time() - t0)

                    path = [(p.x(), p.y()) for p in ppath.path()]

                    wall_mink_poly = pu.fillHoles(r2)
                    env_polys_vis = [vis.Polygon([vis.Point(*p) for p in reversed(pu.pointList(wall_mink_poly))])]
                    for isHole, cont in zip(r2.isHole(), r2):
                        if isHole: env_polys_vis += [vis.Polygon([vis.Point(*p) for p in reversed(cont)])]
                    env = vis.Environment(env_polys_vis)
                    if not env.is_valid(self.epsilon):
                        print("there is error caused by self.epsilon validity")

                    start = vis.Point(*interR)
                    goal = vis.Point(*pgoal)

                    start.snap_to_boundary_of(env, self.epsilon)
                    start.snap_to_vertices_of(env, self.epsilon)

                    t0 = time()
                    ppath = env.shortest_path(start, goal, self.epsilon)
                    # print(time() - t0)

                    path += [(p.x(), p.y()) for p in ppath.path()][1:]
                else:
                    path = [pstart, pgoal]

                # if not collides:
                paths[(rind1, rind2)] = path
                #     # paths[(rind2, rind1)] = list(reversed(path))
                #     color = 'blue'
                # else:
                #     color = 'red'

        return paths

    def getGraphFromPaths(self):
        graph = {}
        for uv, p in self.paths.items():
            u, v = uv
            if p is not None:
                graph[u] = sorted(graph.get(u, []) + [v])
                graph[v] = sorted(graph.get(v, []) + [u])

        return graph
