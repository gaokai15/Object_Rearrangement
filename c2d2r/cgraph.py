import sys
import json
from time import time
from random import uniform
from collections import deque
from itertools import combinations

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

from util import *
import visilibity as vis
import Polygon as pn
import Polygon.Utils as pu

EPSILON = 2**-8


def setupPlot(HEIGHT, WIDTH):
    fig = plt.figure(num=None, figsize=(5, 5), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()
    ax.set_axisbelow(True)
    scale = max(HEIGHT, WIDTH)
    ax.set_ylim(-0.1 * scale, scale * 1.1)
    ax.set_xlim(-0.1 * scale, scale * 1.1)
    ax.grid(which='minor', linestyle=':', alpha=0.2)
    ax.grid(which='major', linestyle=':', alpha=0.5)
    return fig, ax


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
    patch = patches.PathPatch(path, facecolor=color, lw=1, zorder=zorder)
    return patch


def drawProblem(HEIGHT, WIDTH, wall, polygons, path=None, robotStart=None, robotGoal=None):
    _, ax = setupPlot(HEIGHT, WIDTH)
    if robotStart is not None:
        patch = createPolygonPatch(robotStart, 'green', zorder=3)
        ax.add_patch(patch)
    if robotGoal is not None:
        patch = createPolygonPatch(robotGoal, 'red', zorder=3)
        ax.add_patch(patch)

    walls = pu.pointList(wall)
    wallx = [p[0] for p in walls + [walls[0]]]
    wally = [p[1] for p in walls + [walls[0]]]
    plt.plot(wallx, wally, 'blue')

    for poly in polygons:
        patch = createPolygonPatch(pu.pointList(poly), 'gray')
        ax.add_patch(patch)

    if path is not None:
        pts, color = path
        pathx = [p[0] for p in pts]
        pathy = [p[1] for p in pts]
        plt.plot(pathx, pathy, color)

    plt.show()


def drawConGraph(HEIGHT, WIDTH, paths, polygons=None):
    _, ax = setupPlot(HEIGHT, WIDTH)
    scale = max(HEIGHT, WIDTH)

    wallx = [0, WIDTH, WIDTH, 0, 0]
    wally = [0, 0, HEIGHT, HEIGHT, 0]
    plt.plot(wallx, wally, 'blue')

    if polygons is not None:
        c = 0
        for p in range(0, len(polygons)):
            if p % 2 == 0:
                c += 2.0 / len(polygons)
            color = patches.colors.hsv_to_rgb((c, 1, 1))
            patch = createPolygonPatch(pu.pointList(polygons[p]), color)
            ax.add_patch(patch)

    for path in paths.values():
        color = 'black'
        for i in range(1, len(path)):
            ax.annotate(
                "",
                xy=path[i],
                xytext=path[i - 1],
                arrowprops=dict(arrowstyle="-", color=color)
            )

            circ = patches.Circle(path[i - 1], scale / 200.0, color=color, zorder=3)
            ax.add_patch(circ)
            circ = patches.Circle(path[i], scale / 200.0, color=color, zorder=3)
            ax.add_patch(circ)

    plt.show()
    return


def drawMotions(HEIGHT, WIDTH, paths, polygons=None):
    _, ax = setupPlot(HEIGHT, WIDTH)
    scale = max(HEIGHT, WIDTH)

    wallx = [0, WIDTH, WIDTH, 0, 0]
    wally = [0, 0, HEIGHT, HEIGHT, 0]
    plt.plot(wallx, wally, 'blue')

    if polygons is not None:
        c = 0
        for p in range(0, len(polygons)):
            if p % 2 == 0:
                c += 2.0 / len(polygons)
            color = patches.colors.hsv_to_rgb((c, 1, 1))
            patch = createPolygonPatch(pu.pointList(polygons[p]), color, zorder=1)
            ax.add_patch(patch)

    rads = {}
    cc = 0.0
    for path in paths.values():
        cc += 1.0 / len(paths)
        color = patches.colors.hsv_to_rgb((cc, 1, 1))
        # path = paths[(2 * j, 2 * j + 1)]
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
                arrowprops=dict(
                    arrowstyle="simple", connectionstyle="arc3,rad=" + str(rad), color=color
                )
            )
            ax.annotate(
                "",
                xy=vto,
                xytext=vfrom,
                arrowprops=dict(
                    arrowstyle="->", connectionstyle="arc3,rad=" + str(rad), color="black"
                )
            )

            pcolor = "black"
            circ = patches.Circle(path[i - 1], scale / 200.0, color=pcolor, zorder=3)
            ax.add_patch(circ)
            circ = patches.Circle(path[i], scale / 200.0, color=pcolor, zorder=3)
            ax.add_patch(circ)

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


def genCGraph(numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile):
    epsilon = EPSILON
    polygon = np.array(poly_disk([0, 0], RAD, 30))
    wall_pts = pn.Polygon([(0, 0), (WIDTH, 0), (WIDTH, HEIGHT), (0, HEIGHT)])
    wall_mink = pn.Polygon(
        [(RAD, RAD), (WIDTH - RAD, RAD), (WIDTH - RAD, HEIGHT - RAD), (RAD, HEIGHT - RAD)]
    )

    points = []
    objects = []
    # staticObs = []
    minkowski_objs = []
    # minkowski_poly = []
    for i in range(numObjs):

        for i in range(2):
            isfree = False
            timeout = 1000
            while not isfree and timeout > 0:
                timeout -= 1
                point = (
                    uniform(0 - min(polygon[:, 0]), WIDTH - max(polygon[:, 0])),
                    uniform(0 - min(polygon[:, 1]), HEIGHT - max(polygon[:, 1]))
                )
                isfree = isCollisionFree(
                    polygon * (1 + np.sqrt(0.502)),
                    point,
                    objects  # [:len(objects) - i]
                )

            if timeout <= 0:
                # print("Failed to generate!")
                return False, False, False

            points.append(point)
            objects.append(pn.Polygon(polygon + point))
            mink_obj = 2 * polygon + point

            # delta = epsilon * 0
            # for i in range(len(mink_obj)):
            #     # if mink_obj[i][0] < 0:
            #     #     mink_obj[i][0] = delta
            #     # elif mink_obj[i][0] > WIDTH - RAD:
            #     #     mink_obj[i][0] = WIDTH - RAD - delta

            #     if mink_obj[i][0] < RAD:
            #         mink_obj[i][0] = RAD + delta
            #     elif mink_obj[i][0] > WIDTH - RAD:
            #         mink_obj[i][0] = WIDTH - RAD - delta

            #     # if mink_obj[i][1] < 0:
            #     #     mink_obj[i][1] = delta
            #     # elif mink_obj[i][1] > HEIGHT - RAD:
            #     #     mink_obj[i][1] = HEIGHT - RAD - delta

            #     if mink_obj[i][1] < RAD:
            #         mink_obj[i][1] = RAD + delta
            #     elif mink_obj[i][1] > HEIGHT - RAD:
            #         mink_obj[i][1] = HEIGHT - RAD - delta

            minkowski_objs.append(pn.Polygon(mink_obj))

    if display:
        drawProblem(HEIGHT, WIDTH, wall_pts, objects)

    paths = {}
    for indStart, indGoal in combinations(range(numObjs * 2), 2):

        robotStart = pu.pointList(objects[indStart])
        robotGoal = pu.pointList(objects[indGoal])
        pointStart = points[indStart] + polygon * 0.1
        pointGoal = points[indGoal] + polygon * 0.1
        obstacles = objects[:indStart] + objects[indStart + 1:indGoal] + objects[indGoal + 1:]
        minkowski_obs = minkowski_objs[:indStart]
        minkowski_obs += minkowski_objs[indStart + 1:indGoal]
        minkowski_obs += minkowski_objs[indGoal + 1:]

        if displayMore:
            drawProblem(HEIGHT, WIDTH, wall_pts, obstacles, None, robotStart, robotGoal)

        pm = pn.Polygon(wall_mink)
        pm -= sum([pn.Polygon(obj) for obj in minkowski_obs], pn.Polygon())
        pm.simplify()

        wall_mink_poly = None
        env_polys = []
        for poly in sorted([pn.Polygon(p) for p in pm], key=lambda p: p.area()):
            if wall_mink_poly is None:
                if poly.isInside(*points[indStart]) and poly.isInside(*points[indGoal]):
                    wall_mink_poly = poly
                    continue
            env_polys.append(poly)

        if wall_mink_poly is None:
            if displayMore:
                drawProblem(HEIGHT, WIDTH, wall_mink, env_polys, None, pointStart, pointGoal)
            continue

        collides = False
        if display:

            if displayMore:
                drawProblem(
                    HEIGHT, WIDTH, wall_mink_poly, env_polys, None, pointStart, pointGoal
                )

            env_polys.insert(0, wall_mink_poly)

            env_polys_vis = []
            for poly in filter(lambda p: wall_mink_poly.covers(p), env_polys):
                po = vis.Polygon([vis.Point(*p) for p in reversed(pu.pointList(poly))])
                env_polys_vis.append(po)

            env = vis.Environment(env_polys_vis)
            if not env.is_valid(epsilon):
                displayMore = True
                drawProblem(
                    HEIGHT, WIDTH, wall_mink_poly, env_polys, None, pointStart, pointGoal
                )
                if savefile:
                    savefile += ".env_error"
                else:
                    savefile = "polys.json" + str(time()) + ".env_error"

            start = vis.Point(*points[indStart])
            goal = vis.Point(*points[indGoal])

            start.snap_to_boundary_of(env, epsilon)
            start.snap_to_vertices_of(env, epsilon)

            t0 = time()
            path = env.shortest_path(start, goal, epsilon)
            print(time() - t0)

            # for p1, p2 in zip(path.path()[:-1], path.path()[1:]):
            #     collides |= not isEdgeCollisionFree(
            #         polygon, [[p1.x(), p1.y()], [p2.x(), p2.y()]], obstacles
            #     )
            #     if collides:
            #         break
            # for p in list(path.path()):
            #     collides |= not isCollisionFree(polygon, [[p.x(), p.y()]], obstacles)
            #     if collides:
            #         break

            path = [(p.x(), p.y()) for p in path.path()]
        else:
            path = []

        if not collides:
            paths[(indStart, indGoal)] = path
            color = 'blue'
        else:
            color = 'red'

        if display and displayMore:
            drawProblem(
                HEIGHT, WIDTH, wall_mink_poly, env_polys, (path, color), pointStart, pointGoal
            )
            drawProblem(
                HEIGHT, WIDTH, wall_pts, obstacles, (path, color), robotStart, robotGoal
            )

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
            json.dump(
                {
                    'numObjs': numObjs,
                    'RAD': RAD,
                    'HEIGHT': HEIGHT,
                    'WIDTH': WIDTH,
                    'points': points,
                    'objects': np.array([pu.pointList(p) for p in objects]).tolist(),
                    # staticObs = []
                    'graph': graph,
                    'path': {str(k): v
                             for k, v in paths.items()},
                },
                output,
            )

    return graph, paths, objects


def loadCGraph(savefile, repath, display, displayMore):
    with open(savefile, 'r') as finput:
        data = json.load(finput)

    numObjs = data['numObjs']
    RAD = data['RAD']
    HEIGHT = data['HEIGHT']
    WIDTH = data['WIDTH']
    points = data['points']
    objects = [pn.Polygon(o) for o in data['objects']]
    graph = {int(k): v for k, v in data['graph'].items()}
    paths = {eval(k): v for k, v in data['path'].items()}

    epsilon = EPSILON
    polygon = np.array(poly_disk([0, 0], RAD, 30))
    wall_pts = pn.Polygon([(0, 0), (WIDTH, 0), (WIDTH, HEIGHT), (0, HEIGHT)])
    wall_mink = pn.Polygon(
        [(RAD, RAD), (WIDTH - RAD, RAD), (WIDTH - RAD, HEIGHT - RAD), (RAD, HEIGHT - RAD)]
    )

    # staticObs = []
    minkowski_objs = []
    # minkowski_poly = []
    for point in points:
        mink_obj = 2 * polygon + point
        minkowski_objs.append(pn.Polygon(mink_obj))

    if display:
        drawProblem(HEIGHT, WIDTH, wall_pts, objects)

    if repath:
        for indStart, indGoal in combinations(range(numObjs * 2), 2):

            robotStart = pu.pointList(objects[indStart])
            robotGoal = pu.pointList(objects[indGoal])
            pointStart = points[indStart] + polygon * 0.1
            pointGoal = points[indGoal] + polygon * 0.1
            obstacles = objects[:indStart] + objects[indStart + 1:indGoal] + objects[indGoal +
                                                                                     1:]
            minkowski_obs = minkowski_objs[:indStart]
            minkowski_obs += minkowski_objs[indStart + 1:indGoal]
            minkowski_obs += minkowski_objs[indGoal + 1:]

            if displayMore:
                drawProblem(HEIGHT, WIDTH, wall_pts, obstacles, None, robotStart, robotGoal)

            pm = pn.Polygon(wall_mink)
            pm -= sum([pn.Polygon(obj) for obj in minkowski_obs], pn.Polygon())
            pm.simplify()

            wall_mink_poly = None
            env_polys = []
            for poly in sorted([pn.Polygon(p) for p in pm], key=lambda p: p.area()):
                if wall_mink_poly is None:
                    if poly.isInside(*points[indStart]) and poly.isInside(*points[indGoal]):
                        wall_mink_poly = poly
                        continue
                env_polys.append(poly)

            if wall_mink_poly is None:
                if displayMore:
                    drawProblem(
                        HEIGHT, WIDTH, wall_mink, env_polys, None, pointStart, pointGoal
                    )
                continue

            collides = False
            if display or repath:

                if displayMore:
                    drawProblem(
                        HEIGHT, WIDTH, wall_mink_poly, env_polys, None, pointStart, pointGoal
                    )

                env_polys.insert(0, wall_mink_poly)

                env_polys_vis = []
                for poly in filter(lambda p: wall_mink_poly.covers(p), env_polys):
                    po = vis.Polygon([vis.Point(*p) for p in reversed(pu.pointList(poly))])
                    env_polys_vis.append(po)

                env = vis.Environment(env_polys_vis)
                if not env.is_valid(epsilon):
                    displayMore = True
                    drawProblem(
                        HEIGHT, WIDTH, wall_mink_poly, env_polys, None, pointStart, pointGoal
                    )

                start = vis.Point(*points[indStart])
                goal = vis.Point(*points[indGoal])

                start.snap_to_boundary_of(env, epsilon)
                start.snap_to_vertices_of(env, epsilon)

                t0 = time()
                path = env.shortest_path(start, goal, epsilon)
                print(time() - t0)

                # for p1, p2 in zip(path.path()[:-1], path.path()[1:]):
                #     collides |= not isEdgeCollisionFree(
                #         polygon, [[p1.x(), p1.y()], [p2.x(), p2.y()]], obstacles
                #     )
                #     if collides:
                #         break
                # for p in list(path.path()):
                #     collides |= not isCollisionFree(polygon, [[p.x(), p.y()]], obstacles)
                #     if collides:
                #         break

                path = [(p.x(), p.y()) for p in path.path()]
            else:
                path = [points[indStart], 'exists', points[indGoal]]

            if not collides:
                paths[(indStart, indGoal)] = path
                color = 'blue'
            else:
                color = 'red'

            if display and displayMore:
                drawProblem(
                    HEIGHT, WIDTH, wall_mink_poly, env_polys, (path, color), pointStart,
                    pointGoal
                )
                drawProblem(
                    HEIGHT, WIDTH, wall_pts, obstacles, (path, color), robotStart, robotGoal
                )

    if display:
        drawConGraph(HEIGHT, WIDTH, paths, objects)

    if repath:
        new_graph = {}
        for uv, p in paths.items():
            u, v = uv
            if p is not None:
                new_graph[u] = sorted(new_graph.get(u, []) + [v])
                new_graph[v] = sorted(new_graph.get(v, []) + [u])

        assert (new_graph == graph)
        # graph = new_graph

    return numObjs, RAD, HEIGHT, WIDTH, points, objects, graph, paths


if __name__ == "__main__":
    if (len(sys.argv) < 5):
        print(
            '''Error: deptree.py: <# objects> <height> <width> <radius>
            [display?: (y/n)] [display more?: (y/n)] [save file]'''
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
