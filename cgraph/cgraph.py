from __future__ import division
import sys
import json
from time import time
from random import uniform, random, choice
from collections import deque
from itertools import combinations

import numpy as np
from util import *
import Polygon as pn
import Polygon.Utils as pu
import Polygon.Shapes as ps


def genBuffers(numBuffs, wall_mink, polygon, objects, mink_objs, trials, maximumOverlap):
    ### The idea is we randomly generate a buffer in the space, hoping it will overlap with nothing
    ### if it could not achieve after several trials, we increment the number of object poses it can overlap
    ### we keep incrementing until we find enough buffers
    buffer_points = []  ### center of the buffers
    buffers = []  ### polygons of the buffers
    minkowski_buffers = []  ### minkowski sum of the buffers

    ### Random Generation ###
    if False:
        numOverlapAllowed = 0
        for i in range(numBuffs):
            isValid = False
            timeout = 500
            while not isValid and timeout > 0:
                timeout -= 1
                ### generate the center of an object with uniform distribution
                # point = (
                #     uniform(0 - min(polygon[:, 0]), WIDTH - max(polygon[:, 0])),
                #     uniform(0 - min(polygon[:, 1]), HEIGHT - max(polygon[:, 1])),
                # )
                point = wall_mink.sample(random)
                numOverlap = countNumOverlap(polygon, point, objects, buffers, numOverlapAllowed)
                if numOverlap <= numOverlapAllowed:
                    isValid = True
            ### reach here either (1) isValid == True (2) timeout <= 0
            if timeout <= 0:
                ### keep failing generating a buffer allowed to overlap with maximum number of objects
                ### increase the numOverlapAllowed
                numOverlapAllowed += 1
                if (numOverlapAllowed > maximumOverlap):
                    print "Exceed the maximum limit of numOverlap for buffer generation"
                    return buffer_points, buffers, minkowski_buffers

            ### Otherwise the buffer is accepted
            buffer_points.append(point)
            buffers.append(pn.Polygon(polygon + point))
            mink_obj = 2 * polygon + point  ### grown_shape buffer
            minkowski_buffers.append(pn.Polygon(mink_obj))
            # print "successfully generating buffer " + str(i) + " overlapping with " + str(numOverlap) + " poses"

        return buffer_points, buffers, minkowski_buffers

    ### Hueristic Generation ###
    if False:
        polysum = wall_mink - sum(mink_objs, pn.Polygon())
        b_points = set()
        for x in polysum:
            b_points.update(x)
        # print(b_points)

        # numBuffers = len(b_points)
        for i in range(numBuffs):
            # point = choice(list(b_points))
            # b_points.remove(point)
            point = polysum.sample(random)
            buffer_points.append(point)
            buffers.append(pn.Polygon(polygon + point))
            mink_obj = 2 * polygon + point  ### grown_shape buffer
            minkowski_buffers.append(pn.Polygon(mink_obj))

    ### Better Hueristic Generation ###
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


def genInstance(numObjs, HEIGHT, WIDTH, polygon):

    points = []  ### center of the objects
    objects = []  ### polygons of the objects
    minkowski_objs = []  ### minkowski sum of the objects

    for i in range(numObjs):
        ### need to generate both start and goal
        for j in range(2):
            isfree = False
            timeout = 1000
            while not isfree and timeout > 0:
                timeout -= 1
                ### generate the center of an object with uniform distribution
                point = (
                    uniform(0 - min(polygon[:, 0]), WIDTH - max(polygon[:, 0])),
                    uniform(0 - min(polygon[:, 1]), HEIGHT - max(polygon[:, 1])),
                )
                ### For dense case,
                ### start only checks with starts
                ### goal only checks with goals
                isfree = isCollisionFree(polygon, point, objects[j % 2::2])

            if timeout <= 0:
                if j == 0:
                    print "failed to generate the start of object " + str(i)
                else:
                    print "failed to generate the goal of object " + str(i)
                print "FAIL TO GENERATE THE INITIAL INSTANCE"
                return False, False, False

            ### Congrats the object's goal/start is accepted
            points.append(point)
            objects.append(pn.Polygon(polygon + point))
            mink_obj = 2 * polygon + point  ### grown_shape object
            minkowski_objs.append(pn.Polygon(mink_obj))

    return points, objects, minkowski_objs


def genRegionGraph(
    HEIGHT,
    WIDTH,
    numObjs,
    RAD,
    example_index,
    wall_mink,
    points,
    minkowski_objs,
    buffer_points,
    minkowski_buffers,
    displayMore=False
):

    all_points = points + buffer_points
    all_minkowskis = minkowski_objs + minkowski_buffers

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

        ### displayMore: for checking purpose (a long iteration)###
        if displayMore:
            drawRegionGraph(
                HEIGHT, WIDTH, numObjs, RAD, example_index, {}, regions.values(), saveimage=False, label=True
            )

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


def connectRegionGraph(
    HEIGHT,
    WIDTH,
    numObjs,
    RAD,
    example_index,
    epsilon,
    regions,
    polygon,
    points,
    display=False,
    displayMore=False,
    savefile=False
):
    paths = {}

    for rkv1, rkv2 in combinations(regions.items(), 2):
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
        pointStart = pstart + polygon * 0.1
        pointGoal = pgoal + polygon * 0.1

        interR = set(pu.pointList(r1)) & set(pu.pointList(r2))
        if len(interR) > 0:
            rect = ps.Rectangle(dist(pstart, pgoal), 1)
            vsg = np.subtract(pgoal, pstart)
            rect.rotate(np.angle(vsg[0] + 1j * vsg[1]))
            rect.warpToBox(*bbox([pstart, pgoal]))
            hasDirectPath = (r1Ar2).covers(rect) if len(r1Ar2) == 1 else False

            if displayMore:
                drawConGraph(
                    HEIGHT,
                    WIDTH,
                    numObjs,
                    RAD,
                    example_index, {
                        0: pu.pointList(pu.fillHoles(r1)),
                        1: pu.pointList(pu.fillHoles(r2)),
                        2: pu.pointList(rect)
                    },
                    regions.values(),
                    saveimage=False,
                    label=False
                )
                print('Direct?: ', hasDirectPath)

            # # collides = False
            if display and not hasDirectPath:
                interR = min(interR, key=lambda x: dist(pstart, x) + dist(x, pgoal))
                wall_mink_poly = pu.fillHoles(r1)
                env_polys_vis = [vis.Polygon([vis.Point(*p) for p in reversed(pu.pointList(wall_mink_poly))])]
                for isHole, cont in zip(r1.isHole(), r1):
                    if isHole: env_polys_vis += [vis.Polygon([vis.Point(*p) for p in reversed(cont)])]
                env = vis.Environment(env_polys_vis)
                if not env.is_valid(epsilon):
                    displayMore = True
                    drawProblem(
                        HEIGHT, WIDTH, numObjs, RAD, wall_mink_poly, regions.values(), color_pool, points,
                        example_index, False, None, pointStart, pointGoal
                    )
                    if savefile:
                        savefile += ".env_error"
                    else:
                        savefile = "polys.json" + str(time()) + ".env_error"

                start = vis.Point(*pstart)
                goal = vis.Point(*interR)

                start.snap_to_boundary_of(env, epsilon)
                start.snap_to_vertices_of(env, epsilon)

                t0 = time()
                ppath = env.shortest_path(start, goal, epsilon)
                # print(time() - t0)

                path = [(p.x(), p.y()) for p in ppath.path()]

                wall_mink_poly = pu.fillHoles(r2)
                env_polys_vis = [vis.Polygon([vis.Point(*p) for p in reversed(pu.pointList(wall_mink_poly))])]
                for isHole, cont in zip(r2.isHole(), r2):
                    if isHole: env_polys_vis += [vis.Polygon([vis.Point(*p) for p in reversed(cont)])]
                env = vis.Environment(env_polys_vis)
                if not env.is_valid(epsilon):
                    displayMore = True
                    drawProblem(
                        HEIGHT, WIDTH, numObjs, RAD, wall_mink_poly, regions.values(), color_pool, points,
                        example_index, False, None, pointStart, pointGoal
                    )
                    if savefile:
                        savefile += ".env_error"
                    else:
                        savefile = "polys.json" + str(time()) + ".env_error"

                start = vis.Point(*interR)
                goal = vis.Point(*pgoal)

                start.snap_to_boundary_of(env, epsilon)
                start.snap_to_vertices_of(env, epsilon)

                t0 = time()
                ppath = env.shortest_path(start, goal, epsilon)
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

            if display and displayMore:
                drawConGraph(
                    HEIGHT,
                    WIDTH,
                    numObjs,
                    RAD,
                    example_index, {0: path},
                    regions.values(),
                    saveimage=False,
                    label=False
                )

    return paths


def genDenseCGraph(numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, example_index, debug=False):

    ### get all preliminary information ###
    color_pool = getColorMap(numObjs)
    epsilon = EPSILON
    polygon = np.array(poly_disk([0, 0], RAD, 30))  ### object shape
    wall_pts = pn.Polygon([(0, 0), (WIDTH, 0), (WIDTH, HEIGHT), (0, HEIGHT)])  ### workspace
    ### inner boundary to limit the center the object
    wall_mink = pn.Polygon([(RAD, RAD), (WIDTH - RAD, RAD), (WIDTH - RAD, HEIGHT - RAD), (RAD, HEIGHT - RAD)])

    points, objects, minkowski_objs = genInstance(numObjs, HEIGHT, WIDTH, polygon)

    # finish generating the starts and goals
    print "Display the original problem without extra buffers"
    # if display:
    #     drawProblem(
    #         HEIGHT, WIDTH, numObjs, RAD, wall_pts, objects, points, color_pool, example_index, None, saveimage=True
    #     )

    ## Now let's generate some buffers
    # numBuffs = numObjs
    numBuffs = 5
    buffer_points, buffers, minkowski_buffers = genBuffers(
        numBuffs, wall_mink, polygon, objects, minkowski_objs, trials=5, maximumOverlap=numObjs
    )

    ### finish generating the buffers
    print "Finish generating " + str(len(buffer_points)) + " buffers"
    print "Display the original problem with extra buffers"
    if display:
        drawProblem(
            HEIGHT, WIDTH, numObjs, RAD, wall_pts, objects, points, color_pool, example_index, buffers, saveimage=True
        )

    ############ Decomposition of the workspace into different regions ################
    regions, obj2reg = genRegionGraph(
        HEIGHT, WIDTH, numObjs, RAD, example_index, wall_mink, points, minkowski_objs, buffer_points, minkowski_buffers
    )

    ## this is a complete decomposition graph with no paths ###
    # print "display space decomposition"
    # if display:
    #     drawRegionGraph(HEIGHT, WIDTH, numObjs, RAD, example_index, {}, regions.values(), saveimage=True, label=False)

    ##############################################################################################

    ############ Build connections on the region graphs ################
    paths = connectRegionGraph(HEIGHT, WIDTH, numObjs, RAD, example_index, epsilon, regions, polygon, points)

    # if display:
    #     # display decomposition with paths
    #     print "display space decomposition with paths"
    #     drawRegionGraph(HEIGHT, WIDTH, numObjs, RAD, example_index,
    #         paths, regions.values(), saveimage=True, label=True)
    #     ### display decomposition with original object
    #     print "display connectivity graph with original objects"
    #     drawConGraph(HEIGHT, WIDTH, numObjs, RAD, example_index,
    #         paths, color_pool, points, objects, buffers,
    #             saveimage=True
    #     )

    graph = {}
    for uv, p in paths.items():
        u, v = uv
        if p is not None:
            graph[u] = sorted(graph.get(u, []) + [v])
            graph[v] = sorted(graph.get(v, []) + [u])
    # print(graph)

    if savefile:
        with open(savefile, 'w') as output:
            json.dump(
                {
                    'numObjs': numObjs,
                    'RAD': RAD,
                    'HEIGHT': HEIGHT,
                    'WIDTH': WIDTH,
                    'points': points,
                    'objects': np.array([pu.pointList(p) for p in objects]).tolist(),
                    # staticObs = []
                    'graph': {str(k): v
                              for k, v in graph.items()},
                    'path': {str(k): v
                             for k, v in paths.items()},
                },
                output,
            )

    if debug:
        return graph, paths, objects, wall_pts, color_pool, points, polygon, obj2reg
        # return graph, paths, objects, obj2reg, regions, polygon
    return graph, paths, objects + buffers, wall_pts, color_pool, points + buffer_points, polygon, obj2reg
