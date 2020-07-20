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

from rgraph import *

EPSILON = 2**-8


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

    # for walls in wall:
    walls = wall.tolist() + [wall[0].tolist()]
    wallx = [p[0] for p in walls]
    wally = [p[1] for p in walls]
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
                if len(cont) > 1:
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
                    polygon = objectShape + start_pts[nn]
                    patch = createPolygonPatch_distinct(polygon, color_pool[nn], isGoal=True, zorder=1)
                    ax.add_patch(patch)
                    ax.text(start_pts[nn][0], start_pts[nn][1], "S" + str(nn), fontweight='bold', fontsize=10, zorder=1)

                    ### current
                    polygon = objectShape + current_pts[nn]
                    patch = createPolygonPatch_distinct(polygon, color_pool[nn], isGoal=False, zorder=3)
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
                        polygon = objectShape + goal_pts[nn]
                        patch = createPolygonPatch_distinct(polygon, color_pool[nn], isGoal=True, zorder=1)
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
            for cont in polygons[i]:
                patch = createPolygonPatch_distinct(cont, color_pool[obj_idx], isGoal)
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
    robotAt = np.add(point, robot).tolist()
    for poly in obstacles:
        if regionsCollide(poly[0], robotAt):
            return False

    return True


# def isEdgeCollisionFree(robot, edge, obstacles):
#     rpoly1 = region(np.add(robot, edge[0]))
#     rpoly2 = region(np.add(robot, edge[1]))
#     rpoly = pu.pointList(pu.convexHull(cpoly1 + cpoly2))
#     for poly in obstacles:
#         if polysCollide(poly.tolist(), rpoly.tolist()):
#             return False

#     return True


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
    wall_pts = np.array([(0, 0), (WIDTH, 0), (WIDTH, HEIGHT), (0, HEIGHT)])
    wall_mink = region([[RAD, RAD], [WIDTH - RAD, RAD], [WIDTH - RAD, HEIGHT - RAD], [RAD, HEIGHT - RAD]], False)

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
                    int(uniform(0 - min(polygon[:, 0]), WIDTH - max(polygon[:, 0]))),
                    int(uniform(0 - min(polygon[:, 1]), HEIGHT - max(polygon[:, 1]))),
                    # uniform(0 - min(polygon[:, 0]), WIDTH - max(polygon[:, 0])),
                    # uniform(0 - min(polygon[:, 1]), HEIGHT - max(polygon[:, 1])),
                )
                isfree = isCollisionFree(polygon, point, objects[i % 2::2])
                # isfree = isCollisionFree(polygon, point, objects)

            if timeout <= 0:
                # print("Failed to generate!")
                return False, False, False

            points.append(point)
            obj = polygon + point
            objects.append([obj.tolist()])
            mink_obj = 2 * polygon + point
            # print(mink_obj)
            minkowski_objs.append(region(mink_obj.tolist(), True))

    if display:
        drawProblem(HEIGHT, WIDTH, numObjs, RAD, wall_pts, objects, color_pool, points, example_index, saveimage)

    paths = {}

    less_regions, polysum = objects2regions(minkowski_objs, wall_mink)

    # if display:
    #     drawConGraph(HEIGHT, WIDTH, {}, color_pool, [x.to_list() for x in polysum.get_components()], False)
    # cfree = wall_mink - polysum
    # ccomps = cfree.get_components()
    # print(ccomps)
    # if display:
    #     for x in ccomps:
    #         drawConGraph(HEIGHT, WIDTH, {}, color_pool, [x.to_list()], False)

    for i, x in less_regions.items():
        # print(i, x.get_components())
        # if len(x.get_components()) == 0:
        #     print(x)
        #     raw_input("Test")
        #     print(x.complement().to_list())
        #     print(x.to_list())
        # else:
        if displayMore:
            drawConGraph(HEIGHT, WIDTH, {}, color_pool, [x.to_list()], False)

    t0 = time()
    graph, regions, obj2reg = regions2graph(less_regions, wall_mink, polysum, points, 0)
    print("R2G Time: ", time() - t0)

    regLists = []
    for i, x in regions.items():
        # print(i, x.get_components()[0] == x)
        regLists.append(x.to_list())
        # if display:
        #     drawConGraph(HEIGHT, WIDTH, {}, color_pool, [x.to_list()], False)

    if display:
        drawConGraph(HEIGHT, WIDTH, {}, color_pool, regLists, False)

    def drawDebug(rect, r1=None, r2=None):
        if r1 is not None and r2 is not None:
            ind = 0
            trace = {}
            for x in r1.to_list() + r2.to_list() + rect.to_list():
                trace[ind] = x
                ind += 1
            drawConGraph(HEIGHT, WIDTH, trace, color_pool, regLists, False)
        else:
            drawConGraph(HEIGHT, WIDTH, {i: x for i, x in enumerate(rect.to_list())}, color_pool, regLists, False)

    print(graph)

    if display:
        for rind1, r1adj in graph.items():
            for rind2 in r1adj:
                r1 = regions[rind1]
                r2 = regions[rind2]
                if displayMore:
                    drawConGraph(HEIGHT, WIDTH, {}, color_pool, [r1.to_list()], False)
                    drawConGraph(HEIGHT, WIDTH, {}, color_pool, [r2.to_list()], False)

                if displayMore:
                    path = regionPath(r1, r2, debug=drawDebug)
                else:
                    path = regionPath(r1, r2)
                paths[(rind1, rind2)] = path

                if displayMore:
                    drawConGraph(HEIGHT, WIDTH, {0: path}, color_pool, regLists, False)

    if display:
        drawConGraph(HEIGHT, WIDTH, paths, color_pool, regLists, False)
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
                    'objects': objects,
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
    objects = data['objects']
    # staticObs = data['staticObs']
    graph = {eval(k): v for k, v in data['graph'].items()}
    paths = {eval(k): v for k, v in data['path'].items()}

    ### get some colors from colormap
    color_pool = getColorMap(numObjs)

    polygon = np.array(poly_disk([0, 0], RAD, 30))
    wall_pts = np.array([(0, 0), (WIDTH, 0), (WIDTH, HEIGHT), (0, HEIGHT)])
    wall_mink = region([[RAD, RAD], [WIDTH - RAD, RAD], [WIDTH - RAD, HEIGHT - RAD], [RAD, HEIGHT - RAD]], False)

    # staticObs = []
    minkowski_objs = []
    # minkowski_poly = []
    for point in points:
        mink_obj = 2 * polygon + point
        minkowski_objs.append(region(mink_obj.tolist(), True))

    if display:
        drawProblem(HEIGHT, WIDTH, numObjs, RAD, wall_pts, objects, color_pool, points, None, False)

    if repath:
        less_regions, polysum = objects2regions(minkowski_objs, wall_mink)

        for i, x in less_regions.items():
            # print(i, x.get_components())
            # if len(x.get_components()) == 0:
            #     print(x)
            #     raw_input("Test")
            #     print(x.complement().to_list())
            #     print(x.to_list())
            # else:
            if displayMore:
                drawConGraph(HEIGHT, WIDTH, {}, color_pool, [x.to_list()], False)

        t0 = time()
        graph, regions, obj2reg = regions2graph(less_regions, wall_mink, polysum, points, 0)
        print("R2G Time: ", time() - t0)

        regLists = []
        for i, x in regions.items():
            # print(i, x.get_components()[0] == x)
            regLists.append(x.to_list())
            # if display:
            #     drawConGraph(HEIGHT, WIDTH, {}, color_pool, [x.to_list()], False)

        if display:
            drawConGraph(HEIGHT, WIDTH, {}, color_pool, regLists, False)

        def drawDebug(rect, r1=None, r2=None):
            if r1 is not None and r2 is not None:
                ind = 0
                trace = {}
                for x in r1.to_list() + r2.to_list() + rect.to_list():
                    trace[ind] = x
                    ind += 1
                drawConGraph(HEIGHT, WIDTH, trace, color_pool, regLists, False)
            else:
                drawConGraph(HEIGHT, WIDTH, {i: x for i, x in enumerate(rect.to_list())}, color_pool, regLists, False)

        print(graph)

        if display:
            for rind1, r1adj in graph.items():
                for rind2 in r1adj:
                    r1 = regions[rind1]
                    r2 = regions[rind2]
                    if displayMore:
                        drawConGraph(HEIGHT, WIDTH, {}, color_pool, [r1.to_list()], False)
                        drawConGraph(HEIGHT, WIDTH, {}, color_pool, [r2.to_list()], False)

                    if displayMore:
                        path = regionPath(r1, r2, debug=drawDebug)
                    else:
                        path = regionPath(r1, r2)
                    paths[(rind1, rind2)] = path

                    if displayMore:
                        drawConGraph(HEIGHT, WIDTH, {0: path}, color_pool, regLists, False)

        if display:
            drawConGraph(HEIGHT, WIDTH, paths, color_pool, regLists, False)

    if display:
        drawConGraph(HEIGHT, WIDTH, paths, color_pool, objects)

    if repath:
        return graph, paths, objects, color_pool, points, polygon, obj2reg
    return graph, paths, objects, obj2reg, regions, polygon
    # return numObjs, RAD, HEIGHT, WIDTH, points, objects, graph, paths


if __name__ == "__main__":
    if (len(sys.argv) < 5):
        print(
            '''Error: graph.py: <# objects> <radius> <height> <width>
            [display?: (y/n)] [display more?: (y/n)] [save file] [load save?: (y/n)]'''
        )
        exit()

    try:
        numObjs = int(sys.argv[1])
        RAD = int(sys.argv[2])
        HEIGHT = int(sys.argv[3])
        WIDTH = int(sys.argv[4])
    except ValueError:
        print(
            '''Error: graph.py: <# objects> <radius> <height> <width>
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
    repath = False
    if (len(sys.argv) > 8):
        loadSave = sys.argv[8] not in ('n', 'N')
        repath = False if len(sys.argv[8]) == 1 else True

    if loadSave:
        print(loadDenseCGraph(savefile, repath, display, displayMore))
    else:
        print(genDenseCGraph(numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, None, None))
