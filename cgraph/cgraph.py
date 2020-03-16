import sys
from time import time
import numpy as np
from collections import deque
from itertools import combinations
from random import uniform  # , randint
import cPickle as pickle

from matplotlib.lines import Line2D
from matplotlib.path import Path
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from util import *
import visilibity as vis
from DisjointSets import DS


def setupPlot(HEIGHT, WIDTH):
    fig = plt.figure(num=None, figsize=(5, 5), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()
    ax.set_axisbelow(True)
    ax.set_ylim(-1, max(HEIGHT, WIDTH) + 1)
    ax.set_xlim(-1, max(HEIGHT, WIDTH) + 1)
    ax.grid(which='minor', linestyle=':', alpha=0.2)
    ax.grid(which='major', linestyle=':', alpha=0.5)
    return fig, ax


def createPolygonPatch(polygon, color):
    verts = []
    codes = []
    for v in range(0, len(polygon)):
        xy = polygon[v]
        verts.append((xy[0], xy[1]))
        if v == 0:
            codes.append(Path.MOVETO)
        else:
            codes.append(Path.LINETO)
    verts.append(verts[0])
    codes.append(Path.CLOSEPOLY)
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor=color, lw=1)

    return patch


def drawProblem(
    HEIGHT, WIDTH, wall, polygons, graph=None, path=None, robotStart=None, robotGoal=None
):
    _, ax = setupPlot(HEIGHT, WIDTH)
    if robotStart is not None:
        patch = createPolygonPatch(robotStart, 'green')
        ax.add_patch(patch)
    if robotGoal is not None:
        patch = createPolygonPatch(robotGoal, 'red')
        ax.add_patch(patch)

    wallx = [p.x() for p in wall + [wall[0]]]
    wally = [p.y() for p in wall + [wall[0]]]
    plt.plot(wallx, wally, 'blue')

    for p in range(0, len(polygons)):
        patch = createPolygonPatch(polygons[p], 'gray')
        ax.add_patch(patch)

    if graph is not None:
        for u, e in graph:
            for v in e:
                xpts = (u.x, v.x)
                ypts = (u.y, v.y)
                l = Line2D(xpts, ypts, color="black", zorder=1)
                ax.add_line(l)

    if path is not None:
        pathx = [p.x() for p in path.path()]
        pathy = [p.y() for p in path.path()]
        plt.plot(pathx, pathy, 'blue')

    plt.show()


def basicSearch(tree, start, goal):
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


def drawConGraph(HEIGHT, WIDTH, paths, polygons=None):
    _, ax = setupPlot(HEIGHT, WIDTH)
    if polygons is not None:
        for p in range(0, len(polygons)):
            patch = createPolygonPatch(polygons[p], 'gray')
            ax.add_patch(patch)

    c = 0
    for path in paths.values():
        # print(path)
        c += 1 / len(paths)
        color = patches.colors.hsv_to_rgb((c, 1, 1))
        if path is not None:
            pathx = [p.x() for p in path.path()]
            pathy = [p.y() for p in path.path()]
            plt.plot(pathx, pathy, color)

        # for i in range(1, len(path)):
        #     xpts = (points[path[i - 1]][0], points[path[i]][0])
        #     ypts = (points[path[i - 1]][1], points[path[i]][1])
        #     l = Line2D(xpts, ypts, color=color, zorder=2)
        #     ax.add_line(l)
        #     # ax.annotate(
        #     #     "",
        #     #     xy=points[path[i]],
        #     #     xytext=points[path[i - 1]],
        #     #     arrowprops=dict(arrowstyle="->")
        #     # )

        #     circ = patches.Circle(points[path[i - 1]], 0.3, color=color, zorder=3)
        #     ax.add_patch(circ)
        #     circ = patches.Circle(points[path[i]], 0.3, color=color, zorder=3)
        #     ax.add_patch(circ)

    plt.show()
    return


def depSearch(paths, objs, start, goal):
    deps = []
    if len(paths) > 0:
        backtrace = {start: start}
        explore = deque([start])
        while goal not in backtrace:
            try:
                p = explore.pop()
            except Exception:
                return []
            for c in range(objs):
                if p != c:
                    if len(paths[tuple(sorted((p, c)))][2]) > 0:
                        # print("isedge", sorted((p, c)), len(paths[tuple(sorted((p, c)))]))
                        if c not in backtrace:
                            backtrace[c] = p
                            explore.append(c)
                            # print("added", explore, backtrace)

        deps.append(goal)
        while backtrace[deps[-1]] != deps[-1]:
            deps.append(backtrace[deps[-1]])
        deps.reverse()

    return deps


def isCollisionFree(robot, point, obstacles):
    robotAt = np.add(point, robot)
    for poly in obstacles:
        if polysCollide(poly, robotAt):
            return False

    return True


def isCollisionFreeEdge(robot, edge, obstacles):
    for poly in obstacles:
        for i in range(len(robot) - 1):
            roboPart = [robot[i], robot[i], robot[i + 1], robot[i + 1]]
            roboMask = [edge[0], edge[1], edge[1], edge[0]]
            robotEdge = np.add(roboPart, roboMask)
            if polysCollide(poly, robotEdge):
                return False

    return True


def genCGraph(numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile):
    epsilon = 0.00000001
    # polygon = np.array([[0, 0], [1, 1], [2, 0], [1, -1]]) * RAD
    polygon = np.array([[0, 0], [0, 1], [1, 1], [1, 0]]) * RAD
    wall_pts = [
        vis.Point(0, 0),
        vis.Point(WIDTH, 0),
        vis.Point(WIDTH, HEIGHT),
        vis.Point(0, HEIGHT)
    ]
    wall_mink_pts = [
        vis.Point(0, 0),
        vis.Point(WIDTH - RAD, 0),
        vis.Point(WIDTH - RAD, HEIGHT - RAD),
        vis.Point(0, HEIGHT - RAD),
    ]
    # wall_mink_pts = [
    #     vis.Point(0, RAD / 2),
    #     vis.Point(WIDTH - RAD, RAD / 2),
    #     vis.Point(WIDTH - RAD, HEIGHT - RAD / 2),
    #     vis.Point(0, HEIGHT - RAD / 2),
    # ]
    walls = vis.Polygon(wall_pts)

    points = []
    objects = []
    # staticObs = []
    minkowski_objs = []
    # minkowski_poly = []
    for i in range(numObjs):

        for i in range(2):
            isfree = False
            while not isfree:
                point = (
                    int(uniform(0, WIDTH - max(polygon[:, 0]))),
                    int(uniform(0 - min(polygon[:, 1]), HEIGHT - max(polygon[:, 1])))
                )
                isfree = isCollisionFree(polygon, point, objects)  # [:len(objects) - i])
            points.append(point)
            objects.append(polygon + point)
            # mink_obj = 2 * (polygon - [RAD, 0]) + point
            mink_obj = 2 * (polygon - [RAD / 2, RAD / 2]) + point

            delta = epsilon * 0 + 2
            for i in range(len(mink_obj)):
                if mink_obj[i][0] < 0:
                    mink_obj[i][0] = delta
                elif mink_obj[i][0] > WIDTH - RAD:
                    mink_obj[i][0] = WIDTH - RAD - delta

                if mink_obj[i][1] < 0:
                    mink_obj[i][1] = delta
                elif mink_obj[i][1] > HEIGHT - RAD:
                    mink_obj[i][1] = HEIGHT - RAD - delta

                # if mink_obj[i][1] < RAD / 2:
                #     mink_obj[i][1] = RAD / 2 + delta
                # elif mink_obj[i][1] > HEIGHT - RAD / 2:
                #     mink_obj[i][1] = HEIGHT - RAD / 2 - delta

            minkowski_objs.append(mink_obj)
            # minkowski_poly.append(vis.Polygon([vis.Point(*p) for p in mink_obj.astype(float)])) # noqa
            # print(minkowski_poly[-1].is_in_standard_form())
            # print(minkowski_poly[-1].is_simple())

    if display:
        drawProblem(HEIGHT, WIDTH, wall_pts, objects)

    paths = {}
    for indStart, indGoal in combinations(range(numObjs * 2), 2):
        obstacles = objects[:indStart] + objects[indStart + 1:indGoal] + objects[indGoal + 1:]
        minkowski_obs = minkowski_objs[:indStart] + minkowski_objs[
            indStart + 1:indGoal] + minkowski_objs[indGoal + 1:]
        # minkowski_poly_obs = minkowski_poly[:indStart] + minkowski_poly[
        #     indStart + 1:indGoal] + minkowski_poly[indGoal + 1:]

        # i = 0
        # while i < len(minkowski_obs):
        #     j = 0
        #     while j < len(minkowski_obs):
        #         if i == j:
        #             j += 1
        #             continue
        #         try:
        #             pi = minkowski_obs[i]
        #             pj = minkowski_obs[j]
        #         except IndexError:
        #             break
        #         print(i, j, pi, pj, polysCollide(pi, pj))
        #         if polysCollide(pi, pj):
        #             pm = mergePolys(pi, pj)
        #             print("merge: ", pm)
        #             minkowski_obs[i] = np.array(pm)
        #             minkowski_obs.pop(j)

        #         else:
        #             j += 1
        #     i += 1

        ds = DS(range(len(minkowski_obs)))
        for i in range(len(minkowski_obs)):
            for j in range(len(minkowski_obs)):
                if i == j: continue
                pi = minkowski_obs[i]
                pj = minkowski_obs[j]
                if polysCollide(pi, pj):
                    ds.union(i, j)

        # print(ds.getSets())
        mergemink_obs = []
        for dset in ds.getSets():
            pm = mergePolys([minkowski_obs[i] for i in dset], True)
            # print(pm)
            mergemink_obs.append(pm)

        minkowski_poly_obs = []
        for pn in mergemink_obs:
            po = vis.Polygon([vis.Point(*p) for p in pn])  # .astype(float)])
            # print(po.is_in_standard_form())
            # print(po.is_simple())
            minkowski_poly_obs.append(po)

        robotStart = objects[indStart]
        robotGoal = objects[indGoal]
        pointStart = points[indStart] + polygon * 0.1
        pointGoal = points[indGoal] + polygon * 0.1

        # print(minkowski_objs)
        env = vis.Environment([walls] + minkowski_poly_obs)
        print('Environment is valid : ', env.is_valid(epsilon))
        if not env.is_valid(epsilon):
            displayMore = True
            if savefile:
                savefile += ".env_error"
            else:
                savefile = "polys.pkl" + str(time) + ".env_error"

        start = vis.Point(*points[indStart])
        goal = vis.Point(*points[indGoal])

        start.snap_to_boundary_of(env, epsilon)
        start.snap_to_vertices_of(env, epsilon)

        if displayMore:
            drawProblem(
                HEIGHT, WIDTH, wall_pts, obstacles, robotStart=robotStart, robotGoal=robotGoal
            )
            drawProblem(
                HEIGHT,
                WIDTH,
                wall_mink_pts,
                mergemink_obs,
                robotStart=pointStart,
                robotGoal=pointGoal
            )

        path = env.shortest_path(start, goal, epsilon)
        # print(indStart, indGoal, [(p.x(), p.y()) for p in path.path()])
        for p1, p2 in zip(path.path()[:-1], path.path()[1:]):
            if not isCollisionFreeEdge(polygon,
                                       ((p1.x(), p1.y()), (p2.x(), p2.y())), obstacles):
                path = None
                break
        paths[(indStart, indGoal)] = path

        if displayMore:
            drawProblem(HEIGHT, WIDTH, wall_pts, obstacles, None, path, robotStart, robotGoal)

    if display:
        drawConGraph(HEIGHT, WIDTH, paths, objects)

    graph = {}
    for uv, p in paths.items():
        u, v = uv
        if p is not None:
            graph[u] = sorted(graph.get(u, []) + [v])
            graph[v] = sorted(graph.get(v, []) + [u])

    if savefile:
        with open(savefile, 'w') as output:
            pickle.dump(
                (
                    numObjs,
                    RAD,
                    HEIGHT,
                    WIDTH,
                    points,
                    objects,
                    # staticObs = []
                    graph,
                    paths,
                ),
                output,
                pickle.HIGHEST_PROTOCOL
            )

    return graph, paths
    #     # GREEDY
    # depgraph = {}
    # for obj in range(numObjs):
    #     indStart = obj * 2  # * randint(0, numObjs - 1)
    #     indGoal = indStart + 1
    #     deps = depSearch(paths, 2 * numObjs, indStart, indGoal)
    #     depsPath = []
    #     for i in range(1, len(deps)):
    #         # nodes, adjListMap, path = paths[tuple(sorted([deps[i - 1], deps[i]]))]
    #         # depsPath += [nodes[x] for x in path]
    #         depsPath += [paths[tuple(sorted([deps[i - 1], deps[i]]))]]

    #     depgraph[indStart] = (deps, depsPath)
    #     print(indStart, deps)

    # if display:
    # drawSolution(HEIGHT, WIDTH, depgraph, objects + staticObs)


if __name__ == "__main__":
    if (len(sys.argv) < 5):
        print(
            '''Error: deptree.py: <# objects> <height> <width> <radius>
            [display?: (y/n)] [display more?: (y/n)] [save file]'''
        )
        exit()

    try:
        numObjs = int(sys.argv[1])
        RAD = int(sys.argv[2])
        HEIGHT = int(sys.argv[3])
        WIDTH = int(sys.argv[4])
    except ValueError:
        print(
            '''Error: deptree.py: <# objects> <height> <width> <radius>
            [display?: (y/n)] [display more?: (y/n)] [save file]'''
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

    genCGraph(numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile)
