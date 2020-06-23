from __future__ import division
import os
import sys
import json
from time import time
from collections import deque
from random import uniform, random
from itertools import combinations

import IPython
import numpy as np
import matplotlib
# matplotlib.use('agg')
import matplotlib.cm as cmx
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.colors as colors
import matplotlib.patches as patches

from rgraph_poly import *

EPSILON = 2**-8

# def setupPlot(HEIGHT, WIDTH):
#     fig = plt.figure(num=None, figsize=(5, 5), dpi=120, facecolor='w', edgecolor='k')
#     ax = fig.subplots()
#     ax.set_axisbelow(True)
#     scale = max(HEIGHT, WIDTH)
#     ax.set_ylim(-0.1 * scale, scale * 1.1)
#     ax.set_xlim(-0.1 * scale, scale * 1.1)
#     ax.grid(which='minor', linestyle=':', alpha=0.2)
#     ax.grid(which='major', linestyle=':', alpha=0.5)
#     return fig, ax


def createPolygonPatch(polygon, color, zorder=1):
    verts = []
    codes = []
    for v in range(0, len(polygon)):
        verts.append(polygon[v])
        if v == 0:
            codes.append(Path.MOVETO)
        else:
            codes.append(Path.LINETO)
    verts.append(verts[0])
    codes.append(Path.CLOSEPOLY)
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor=color, lw=0.5, zorder=zorder)
    return patch


def createPolygonPatch_distinct(polygon, color, isGoal, zorder=1):
    verts = []
    codes = []
    for v in range(0, len(polygon)):
        verts.append(polygon[v])
        if v == 0:
            codes.append(Path.MOVETO)
        else:
            codes.append(Path.LINETO)

    verts.append(verts[0])
    codes.append(Path.CLOSEPOLY)
    path = Path(verts, codes)
    if isGoal:
        patch = patches.PathPatch(path, linestyle='--', edgecolor=color, facecolor="white", lw=1, zorder=zorder)
    else:
        patch = patches.PathPatch(path, facecolor=color, lw=0.5, zorder=zorder)

    return patch


def drawProblem(
    HEIGHT,
    WIDTH,
    numObjs,
    RAD,
    wall,
    polygons,
    color_pool,
    points,
    example_index,
    saveimage,
    path=None,
    robotStart=None,
    robotGoal=None
):
    _, ax = setupPlot(HEIGHT, WIDTH)
    if robotStart is not None:
        patch = createPolygonPatch(robotStart, 'green', zorder=3)
        ax.add_patch(patch)
    if robotGoal is not None:
        patch = createPolygonPatch(robotGoal, 'red', zorder=3)
        ax.add_patch(patch)

    for walls in wall:
        # walls = pu.pointList(cont)
        wallx = [p[0] for p in walls + [walls[0]]]
        wally = [p[1] for p in walls + [walls[0]]]
        plt.plot(wallx, wally, 'blue')

    for i in range(len(polygons)):
        obj_idx = i // 2
        isGoal = i % 2
        poly = polygons[i]
        if type(poly) == tuple:
            poly, _ = poly
        for cont in poly:
            # patch = createPolygonPatch_distinct(cont, color, p % 2)
            patch = createPolygonPatch_distinct(cont, color_pool[obj_idx], isGoal)
            ax.add_patch(patch)

    ### label these polygon
    for i in range(len(points)):
        obj_idx = i // 2
        ax.text(points[i][0], points[i][1], str(obj_idx), fontweight='bold', fontsize=10, zorder=3)

    # for poly in polygons:
    #     patch = createPolygonPatch(pu.pointList(poly), 'gray')
    #     ax.add_patch(patch)

    if path is not None:
        pts, color = path
        pathx = [p[0] for p in pts]
        pathy = [p[1] for p in pts]
        plt.plot(pathx, pathy, color)
    if saveimage:
        path = os.getcwd() + "/figures/"
        plt.savefig(
            path + str(numObjs) + "_" + str(int(RAD)) + "_" + str(HEIGHT) + "_" + str(WIDTH) + "_" +
            str(example_index) + "_object_deployment.png"
        )
    plt.show()


def drawConGraph(HEIGHT, WIDTH, paths, color_pool, polygons=None, label=True):
    _, ax = setupPlot(HEIGHT, WIDTH)
    scale = max(HEIGHT, WIDTH)

    wallx = [0, WIDTH, WIDTH, 0, 0]
    wally = [0, 0, HEIGHT, HEIGHT, 0]
    plt.plot(wallx, wally, 'blue')

    if polygons is not None:
        c = 0
        for p in range(0, len(polygons)):
            if label:
                if p % 2 == 0:
                    c += 2.0 / len(polygons)
            else:
                c += 1.0 / len(polygons)
            color = patches.colors.hsv_to_rgb((c, 1, 1))
            poly = polygons[p]
            if type(poly) == tuple:
                poly, _ = poly
            for cont in poly:
                patch = createPolygonPatch_distinct(cont, color, label and p % 2)
                ax.add_patch(patch)
                if label:
                    ax.annotate(str(p), xy=(0, 0), xytext=pn.Polygon(cont).center())

    for path in paths.values():
        color = 'black'
        for i in range(1, len(path)):
            ax.annotate("", xy=path[i], xytext=path[i - 1], arrowprops=dict(arrowstyle="-", color=color))

            circ = patches.Circle(path[i - 1], scale / 400.0, color=color, zorder=3)
            ax.add_patch(circ)
            circ = patches.Circle(path[i], scale / 400.0, color=color, zorder=3)
            ax.add_patch(circ)

        circ = patches.Circle(path[0], scale / 200.0, color=color, zorder=3)
        ax.add_patch(circ)
        circ = patches.Circle(path[-1], scale / 200.0, color=color, zorder=3)
        ax.add_patch(circ)

    plt.show()
    return

    if saveimage:
        path = os.getcwd() + "/figures/"
        plt.savefig(
            path + str(numObjs) + "_" + str(int(RAD)) + "_" + str(HEIGHT) + "_" + str(WIDTH) + "_" +
            str(example_index) + "_cgraph.png"
        )
    plt.show()
    return


def setupPlot(HEIGHT, WIDTH):
    fig = plt.figure(num=None, figsize=(int(5 * WIDTH / HEIGHT), 5), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()
    return fig, ax


def animatedMotions(
    HEIGHT, WIDTH, numObjs, RAD, rpaths, color_pool, points, object_ordering, example_index, objectShape=None
):

    # n_steps = 5

    if objectShape is None:
        print("Fail to create the instances. Not to mention to animate... Try it again")
        return

    ### set the canvas
    fig, ax = setupPlot(HEIGHT, WIDTH)
    wallx = [0, WIDTH, WIDTH, 0, 0]
    wally = [0, 0, HEIGHT, HEIGHT, 0]
    ax.plot(wallx, wally, 'blue')
    plt.show(block=False)

    ### give a prompt to start animation
    raw_input("Press <ENTER> to start animation")

    ### separate the points (center of the objects) into (1) current points and (2) goal points
    current_pts = points[0::2]
    start_pts = points[0::2]
    goal_pts = points[1::2]
    isGoalReached = [False for i in range(numObjs)]

    for fake_object in object_ordering:
        if fake_object >= numObjs:
            obj = fake_object - numObjs
            for key in rpaths.keys():
                if key[0] == 2 * obj + 1:
                    buffer = key[1]
                    break
            key0 = 2 * obj + 1
            key1 = buffer
        else:
            obj = fake_object
            for key in rpaths.keys():
                if key[0] == 2 * obj:
                    buffer = key[1]
                    break
            key0 = 2 * obj
            key1 = buffer

        for wpt_idx in range(len(rpaths[(key0, key1)]) - 1):
            pt1 = rpaths[(key0, key1)][wpt_idx]
            pt2 = rpaths[(key0, key1)][wpt_idx + 1]
            step1 = int(abs(pt1[0] - pt2[0]) / 50.0)
            step2 = int(abs(pt1[1] - pt2[1]) / 50.0)
            if step1 == 0 and step2 == 0:
                n_steps = 1
            else:
                n_steps = max(step1, step2)
            for step in range(n_steps + 1):
                ### update current_pts
                current_pts[obj] = (
                    pt1[0] + (pt2[0] - pt1[0]) / n_steps * step, pt1[1] + (pt2[1] - pt1[1]) / n_steps * step
                )
                ### check if the current object reaches the goal
                if current_pts[obj] == goal_pts[obj]:
                    isGoalReached[obj] = True

                ax.cla()
                ax.plot(wallx, wally, 'blue')

                ### plot the objects
                for nn in range(len(current_pts)):
                    ### start
                    polygon = pn.Polygon(objectShape + start_pts[nn])
                    patch = createPolygonPatch_distinct(pu.pointList(polygon), color_pool[nn], isGoal=True, zorder=1)
                    ax.add_patch(patch)
                    ax.text(start_pts[nn][0], start_pts[nn][1], "S" + str(nn), fontweight='bold', fontsize=10, zorder=1)

                    ### current
                    polygon = pn.Polygon(objectShape + current_pts[nn])
                    patch = createPolygonPatch_distinct(pu.pointList(polygon), color_pool[nn], isGoal=False, zorder=3)
                    ax.add_patch(patch)
                    ax.text(
                        current_pts[nn][0],
                        current_pts[nn][1],
                        "Obj" + str(nn),
                        fontweight='bold',
                        fontsize=10,
                        zorder=3
                    )
                    ### goal
                    if not isGoalReached[nn]:
                        polygon = pn.Polygon(objectShape + goal_pts[nn])
                        patch = createPolygonPatch_distinct(
                            pu.pointList(polygon), color_pool[nn], isGoal=True, zorder=1
                        )
                        ax.add_patch(patch)
                        ax.text(
                            goal_pts[nn][0], goal_pts[nn][1], "G" + str(nn), fontweight='bold', fontsize=10, zorder=1
                        )

                plt.pause(0.05)

    plt.show()
    return


def drawMotions(HEIGHT, WIDTH, numObjs, RAD, paths, color_pool, points, example_index, saveimage, polygons=None):
    _, ax = setupPlot(HEIGHT, WIDTH)
    scale = max(HEIGHT, WIDTH)

    wallx = [0, WIDTH, WIDTH, 0, 0]
    wally = [0, 0, HEIGHT, HEIGHT, 0]
    plt.plot(wallx, wally, 'blue')

    if polygons is not None:
        for i in range(len(polygons)):
            obj_idx = i // 2
            isGoal = i % 2
            patch = createPolygonPatch_distinct(pu.pointList(polygons[i]), color_pool[obj_idx], isGoal)
            ax.add_patch(patch)
        ### label these polygons
        for i in range(len(points)):
            obj_idx = i // 2
            ax.text(points[i][0], points[i][1], str(obj_idx), fontweight='bold', fontsize=10, zorder=1)

    # if polygons is not None:
    #     c = 0
    #     for p in range(0, len(polygons)):
    #         if p % 2 == 0:
    #             c += 2.0 / len(polygons)
    #         # else:
    #         #     pox = [pp[0] for pp in pu.pointList(polygons[p])]
    #         #     poy = [pp[1] for pp in pu.pointList(polygons[p])]
    #         #     pox.append(pox[0])
    #         #     poy.append(poy[1])
    #         #     color = patches.colors.hsv_to_rgb((c, 1, 1))
    #         #     plt.plot(pox, poy, color)
    #         color = patches.colors.hsv_to_rgb((c, 1, 1 - 0.5 * (p % 2)))
    #         patch = createPolygonPatch(pu.pointList(polygons[p]), color, zorder=1)
    #         ax.add_patch(patch)

    rads = {}
    # cc = 0.0
    for obj, path in paths.items():
        # cc += 1.0 / len(paths)
        # color = patches.colors.hsv_to_rgb((cc, 1, 1))
        # path = paths[(2 * j, 2 * j + 1)]
        obj_idx = obj[0] // 2
        color = color_pool[obj_idx]
        for i in range(1, len(path)):
            vfrom = path[i - 1]
            vto = path[i]
            rad = rads.get((vfrom, vto), -0.2) + 0.2
            rads[(vfrom, vto)] = rad
            rads[(vto, vfrom)] = rad

            ax.annotate(
                "",
                xy=vto,
                xytext=vfrom,
                arrowprops=dict(arrowstyle="simple", connectionstyle="arc3,rad=" + str(rad), color=color)
            )
            # ax.annotate(
            #     "",
            #     xy=vto,
            #     xytext=vfrom,
            #     arrowprops=dict(
            #         arrowstyle="->", connectionstyle="arc3,rad=" + str(rad), color=color
            #     )
            # )
            ax.annotate(
                "",
                xy=vto,
                xytext=vfrom,
                arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=" + str(rad), color="black")
            )

            pcolor = "black"
            circ = patches.Circle(path[i - 1], scale / 200.0, color=pcolor, zorder=3)
            ax.add_patch(circ)
            circ = patches.Circle(path[i], scale / 200.0, color=pcolor, zorder=3)
            ax.add_patch(circ)
    if saveimage:
        path = os.getcwd() + "/figures/"
        plt.savefig(
            path + str(numObjs) + "_" + str(int(RAD)) + "_" + str(HEIGHT) + "_" + str(WIDTH) + "_" +
            str(example_index) + "_paths.png"
        )
    plt.show()
    return


def isCollisionFree(robot, point, obstacles):
    robotAt = np.add(point, robot)
    for poly in obstacles:
        if polysCollide(poly, robotAt):
            return False

    return True


def isEdgeCollisionFree(robot, edge, obstacles):
    cpoly1 = pn.Polygon(np.add(robot, edge[0]))
    cpoly2 = pn.Polygon(np.add(robot, edge[1]))
    rpoly = pu.pointList(pu.convexHull(cpoly1 + cpoly2))
    for poly in obstacles:
        if polysCollide(poly, rpoly):
            return False

    return True


def getColorMap(numObjs):
    # set colormap
    gist_ncar = plt.get_cmap('gist_ncar')
    cNorm = colors.Normalize(vmin=0, vmax=1)
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=gist_ncar)

    color_values = np.linspace(0, 0.9, numObjs)
    color_pool = [scalarMap.to_rgba(color_values[ii]) for ii in xrange(numObjs)]

    return color_pool


def genDenseCGraph(numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, example_index, debug=False):

    ### get some colors from colormap
    color_pool = getColorMap(numObjs)

    polygon = np.array(poly_disk([0, 0], RAD, 30))
    wall_pts = pn.Polygon([(0, 0), (WIDTH, 0), (WIDTH, HEIGHT), (0, HEIGHT)])
    wall_mink = pn.Polygon([(RAD, RAD), (WIDTH - RAD, RAD), (WIDTH - RAD, HEIGHT - RAD), (RAD, HEIGHT - RAD)])

    points = []
    objects = []
    # staticObs = []
    minkowski_objs = []
    for i in range(numObjs):

        for i in range(2):
            isfree = False
            timeout = 1000
            while not isfree and timeout > 0:
                timeout -= 1
                point = (
                    uniform(0 - min(polygon[:, 0]),
                            WIDTH - max(polygon[:, 0])), uniform(0 - min(polygon[:, 1]), HEIGHT - max(polygon[:, 1]))
                )
                isfree = isCollisionFree(polygon, point, objects[i % 2::2])
                # isfree = isCollisionFree(polygon, point, objects)

            if timeout <= 0:
                # print("Failed to generate!")
                return False, False, False

            points.append(point)
            objects.append(pn.Polygon(polygon + point))
            mink_obj = 2 * polygon + point

            minkowski_objs.append(pn.Polygon(mink_obj))

    if display:
        drawProblem(HEIGHT, WIDTH, numObjs, RAD, wall_pts, objects, color_pool, points, example_index, saveimage)

    paths = {}

    less_regions, polysum = objects2regions(minkowski_objs, wall_mink)
    graph, regions, obj2reg = regions2graph(less_regions, wall_mink, polysum, points)

    if display:
        drawConGraph(HEIGHT, WIDTH, {}, color_pool, regions.values(), False)

    def drawDebug(rect, r1=None, r2=None):
        if r1 and r2:
            drawConGraph(
                HEIGHT, WIDTH, {
                    0: pu.pointList(pu.fillHoles(r1)),
                    1: pu.pointList(pu.fillHoles(r2)),
                    2: pu.pointList(rect)
                }, color_pool, regions.values(), False
            )
        else:
            drawConGraph(HEIGHT, WIDTH, {
                0: pu.pointList(rect),
            }, color_pool, regions.values(), False)

    if display:
        for rind1, r1adj in graph.items():
            for rind2 in r1adj:
                r1 = regions[rind1]
                r2 = regions[rind2]

                if displayMore:
                    path = regionPath(r1, r2, debug=drawDebug)
                else:
                    path = regionPath(r1, r2)
                paths[(rind1, rind2)] = path

                if displayMore:
                    drawConGraph(HEIGHT, WIDTH, {0: path}, color_pool, regions.values(), False)

    if display:
        drawConGraph(HEIGHT, WIDTH, paths, color_pool, regions.values(), False)
        drawConGraph(HEIGHT, WIDTH, paths, color_pool, objects)

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
        return graph, paths, objects, obj2reg, regions, polygon
    return graph, paths, objects, color_pool, points, polygon, obj2reg


def loadDenseCGraph(savefile, repath, display, displayMore):
    with open(savefile, 'r') as finput:
        data = json.load(finput)

    numObjs = data['numObjs']
    RAD = data['RAD']
    HEIGHT = data['HEIGHT']
    WIDTH = data['WIDTH']
    points = data['points']
    objects = [pn.Polygon(o) for o in data['objects']]
    graph = {eval(k): v for k, v in data['graph'].items()}
    paths = {eval(k): v for k, v in data['path'].items()}

    epsilon = EPSILON
    polygon = np.array(poly_disk([0, 0], RAD, 30))
    wall_pts = pn.Polygon([(0, 0), (WIDTH, 0), (WIDTH, HEIGHT), (0, HEIGHT)])
    wall_mink = pn.Polygon([(RAD, RAD), (WIDTH - RAD, RAD), (WIDTH - RAD, HEIGHT - RAD), (RAD, HEIGHT - RAD)])

    # staticObs = []
    minkowski_objs = []
    # minkowski_poly = []
    for point in points:
        mink_obj = 2 * polygon + point
        minkowski_objs.append(pn.Polygon(mink_obj))

    if display:
        drawProblem(HEIGHT, WIDTH, wall_pts, objects)

    if repath:
        paths = {}
        regions = {}
        polysum = pn.Polygon()
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

            objDIFFpolysum = wall_mink & obj - polysum
            # print(objDIFFpolysum, bool(objDIFFpolysum))
            if objDIFFpolysum:
                regions[(i, )] = objDIFFpolysum
            polysum += obj

            if displayMore:
                drawConGraph(HEIGHT, WIDTH, {}, color_pool, regions.values(), False)

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

                for i, p in enumerate(points):
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
        # freeInd = 0
        # for strip in (wall_mink - polysum).triStrip():
        #     freeInd -= 1
        #     slen = len(strip) - 2
        #     for i in range(slen):
        #         ptri = pn.Polygon(strip[i:i + 3])
        #         regions[(freeInd, i - slen // 2)] = ptri

        # print(sorted(regions))

        # for r, p in regions.items():
        #     paths[r] = [p.center()] * 2

        if display:
            drawConGraph(HEIGHT, WIDTH, {}, color_pool, regions.values(), False)

        for rkv1, rkv2 in combinations(regions.items(), 2):
            rind1, r1 = rkv1
            rind2, r2 = rkv2
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
                rect.warpToBox(*pn.Polygon([pstart, pgoal]).boundingBox())
                hasDirectPath = (r1Ar2).covers(rect) if len(r1Ar2) == 1 else False

                if displayMore:
                    drawConGraph(
                        HEIGHT, WIDTH, {
                            0: pu.pointList(pu.fillHoles(r1)),
                            1: pu.pointList(pu.fillHoles(r2)),
                            2: pu.pointList(rect)
                        }, color_pool, regions.values(), False
                    )
                    print('Direct?: ', hasDirectPath)
                    # drawProblem(
                    #     HEIGHT, WIDTH, wall_mink, regions.values(), None, pu.pointList(pu.fillHoles(r1)),
                    #     pu.pointList(pu.fillHoles(r2))
                    # )

                # collides = False
                if display and not hasDirectPath:
                    interR = min(interR, key=lambda x: dist(pstart, x) + dist(x, pgoal))
                    wall_mink_poly = pu.fillHoles(r1)
                    # if displayMore:
                    #     # drawProblem(HEIGHT, WIDTH, wall_mink_poly, regions.values(), None, pointStart, pointGoal)
                    #     drawConGraph(HEIGHT, WIDTH, {0: pointStart, 1: pointGoal}, regions.values(), False)
                    env_polys_vis = [vis.Polygon([vis.Point(*p) for p in reversed(pu.pointList(wall_mink_poly))])]
                    for isHole, cont in zip(r1.isHole(), r1):
                        if isHole: env_polys_vis += [vis.Polygon([vis.Point(*p) for p in reversed(cont)])]
                    env = vis.Environment(env_polys_vis)
                    if not env.is_valid(epsilon):
                        displayMore = True
                        drawProblem(HEIGHT, WIDTH, wall_mink_poly, regions.values(), None, pointStart, pointGoal)
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
                    print(time() - t0)

                    path = [(p.x(), p.y()) for p in ppath.path()]

                    wall_mink_poly = pu.fillHoles(r2)
                    # if displayMore:
                    #     # drawProblem(HEIGHT, WIDTH, wall_mink_poly, regions.values(), None, pointStart, pointGoal)
                    #     drawConGraph(HEIGHT, WIDTH, {0: pointStart, 1: pointGoal}, regions.values(), False)
                    env_polys_vis = [vis.Polygon([vis.Point(*p) for p in reversed(pu.pointList(wall_mink_poly))])]
                    for isHole, cont in zip(r2.isHole(), r2):
                        if isHole: env_polys_vis += [vis.Polygon([vis.Point(*p) for p in reversed(cont)])]
                    env = vis.Environment(env_polys_vis)
                    if not env.is_valid(epsilon):
                        displayMore = True
                        drawProblem(HEIGHT, WIDTH, wall_mink_poly, regions.values(), None, pointStart, pointGoal)
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
                    print(time() - t0)

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
                    # drawProblem(HEIGHT, WIDTH, r1 + r2, regions.values(), (path, color), pointStart, pointGoal)
                    drawConGraph(HEIGHT, WIDTH, {0: path}, color_pool, regions.values(), False)

    if display:
        drawConGraph(HEIGHT, WIDTH, paths, color_pool, regions.values(), False)
        drawConGraph(HEIGHT, WIDTH, paths, color_pool, objects)

    if repath:
        new_graph = {}
        for uv, p in paths.items():
            u, v = uv
            if p is not None:
                new_graph[u] = sorted(new_graph.get(u, []) + [v])
                new_graph[v] = sorted(new_graph.get(v, []) + [u])

        # assert (new_graph == graph)
        graph = new_graph

    return numObjs, RAD, HEIGHT, WIDTH, points, objects, graph, paths, obj2reg


if __name__ == "__main__":
    if (len(sys.argv) < 5):
        print(
            '''Error: deptree.py: <# objects> <height> <width> <radius>
            [display?: (y/n)] [display more?: (y/n)] [save file] [load save?: (y/n)]'''
        )
        exit()

    try:
        numObjs = int(sys.argv[1])
        RAD = float(sys.argv[2])
        HEIGHT = int(sys.argv[3])
        WIDTH = int(sys.argv[4])
    except ValueError:
        print(
            '''Error: deptree.py: <# objects> <height> <width> <radius>
            [display?: (y/n)] [display more?: (y/n)] [save file] [load save?: (y/n)]'''
        )
        exit()

    display = False
    if (len(sys.argv) > 5):
        display = sys.argv[5] not in ('n', 'N')

    displayMore = False
    if (len(sys.argv) > 6):
        displayMore = sys.argv[6] not in ('n', 'N')

    savefile = False
    if (len(sys.argv) > 7):
        savefile = sys.argv[7]

    loadSave = False
    if (len(sys.argv) > 8):
        loadSave = sys.argv[8] not in ('n', 'N')

    if loadSave:
        print(loadDenseCGraph(savefile, True, display, displayMore))
    else:
        print(genDenseCGraph(numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, None, None))
