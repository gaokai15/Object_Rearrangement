import sys
import numpy as np
from collections import deque
from itertools import combinations
from random import uniform  # , randint

from matplotlib.lines import Line2D
from matplotlib.path import Path
import matplotlib.pyplot as plt
import matplotlib.patches as patches

from util import *
from rrtree import RRTree
from roadmap import Roadmap


def setupPlot(HEIGHT, WIDTH):
    '''
    Set up matplotlib to create a plot with an empty square
    '''
    fig = plt.figure(num=None, figsize=(5, 5), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()
    ax.set_axisbelow(True)
    ax.set_ylim(-1, max(HEIGHT, WIDTH) + 1)
    ax.set_xlim(-1, max(HEIGHT, WIDTH) + 1)
    ax.grid(which='minor', linestyle=':', alpha=0.2)
    ax.grid(which='major', linestyle=':', alpha=0.5)
    return fig, ax


def createPolygonPatch(polygon, color):
    '''
    Make a patch for a single pology
    '''
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


def drawProblem(HEIGHT, WIDTH, polygons, robotStart=None, robotGoal=None):
    '''
    Render the problem
    '''
    _, ax = setupPlot(HEIGHT, WIDTH)
    if robotStart is not None:
        patch = createPolygonPatch(robotStart, 'green')
        ax.add_patch(patch)
    if robotGoal is not None:
        patch = createPolygonPatch(robotGoal, 'red')
        ax.add_patch(patch)
    for p in range(0, len(polygons)):
        patch = createPolygonPatch(polygons[p], 'gray')
        ax.add_patch(patch)
    plt.show()


def basicSearch(tree, start, goal):
    '''
    Perform basic search
    '''
    path = []
    if len(tree) > 0:
        backtrace = {start: start}
        explore = deque([start])
        while goal not in backtrace:
            p = explore.pop()
            for c in tree[p]:
                if c not in backtrace:
                    backtrace[c] = p
                    explore.append(c)

        path.append(goal)
        while backtrace[path[-1]] != path[-1]:
            path.append(backtrace[path[-1]])
        path.reverse()

    return path


def displayPathInGraph(
    HEIGHT, WIDTH, points, adjListMap, path=[], robotStart=None, robotGoal=None, polygons=None
):
    '''
    Display the RRT and Path
    '''
    _, ax = setupPlot(HEIGHT, WIDTH)
    if robotStart is not None:
        patch = createPolygonPatch(robotStart, 'green')
        ax.add_patch(patch)
    if robotGoal is not None:
        patch = createPolygonPatch(robotGoal, 'red')
        ax.add_patch(patch)
    if polygons is not None:
        for p in range(0, len(polygons)):
            patch = createPolygonPatch(polygons[p], 'gray')
            ax.add_patch(patch)

    for k, p in points.items():
        circ = patches.Circle(p, 0.03, color="black", zorder=1)
        ax.add_patch(circ)

    for u, e in adjListMap.items():
        for v in e:
            xpts = (points[u][0], points[v][0])
            ypts = (points[u][1], points[v][1])
            l = Line2D(xpts, ypts, color="black", zorder=1)
            ax.add_line(l)

    for i in range(1, len(path)):
        xpts = (points[path[i - 1]][0], points[path[i]][0])
        ypts = (points[path[i - 1]][1], points[path[i]][1])
        l = Line2D(xpts, ypts, color="orange", zorder=2)
        ax.add_line(l)

    plt.show()
    return


def displayRoadmap(HEIGHT, WIDTH, paths, polygons=None):
    '''
    Display roadmap
    '''
    _, ax = setupPlot(HEIGHT, WIDTH)
    # if robotStart is not None:
    #     patch = createPolygonPatch(robotStart, 'green')
    #     ax.add_patch(patch)
    # if robotGoal is not None:
    #     patch = createPolygonPatch(robotGoal, 'red')
    #     ax.add_patch(patch)
    if polygons is not None:
        for p in range(0, len(polygons)):
            patch = createPolygonPatch(polygons[p], 'gray')
            ax.add_patch(patch)

    # for k, p in points.items():
    #     circ = patches.Circle(p, 0.03, color="black", zorder=1)
    #     ax.add_patch(circ)

    # for u, e in adjListMap.items():
    #     for v in e:
    #         xpts = (points[u][0], points[v][0])
    #         ypts = (points[u][1], points[v][1])
    #         l = Line2D(xpts, ypts, color="black", zorder=1)
    #         ax.add_line(l)
    color = "black"
    for points, adjListMap, path in paths.values():
        for i in range(1, len(path)):
            xpts = (points[path[i - 1]][0], points[path[i]][0])
            ypts = (points[path[i - 1]][1], points[path[i]][1])
            l = Line2D(xpts, ypts, color=color, zorder=2)
            ax.add_line(l)

            circ = patches.Circle(points[path[i - 1]], 0.03, color=color, zorder=3)
            ax.add_patch(circ)
            circ = patches.Circle(points[path[i]], 0.03, color=color, zorder=3)
            ax.add_patch(circ)

    plt.show()
    return


def displaySolution(HEIGHT, WIDTH, depgraph, polygons=None):
    '''
    Display solution Path
    '''
    _, ax = setupPlot(HEIGHT, WIDTH)
    # if robotStart is not None:
    #     patch = createPolygonPatch(robotStart, 'green')
    #     ax.add_patch(patch)
    # if robotGoal is not None:
    #     patch = createPolygonPatch(robotGoal, 'red')
    #     ax.add_patch(patch)
    if polygons is not None:
        for p in range(0, len(polygons)):
            patch = createPolygonPatch(polygons[p], 'gray')
            ax.add_patch(patch)

    # for k, p in points.items():
    #     circ = patches.Circle(p, 0.03, color="black", zorder=1)
    #     ax.add_patch(circ)

    # for u, e in adjListMap.items():
    #     for v in e:
    #         xpts = (points[u][0], points[v][0])
    #         ypts = (points[u][1], points[v][1])
    #         l = Line2D(xpts, ypts, color="black", zorder=1)
    #         ax.add_line(l)

    c = 0
    for deps, deppaths in depgraph.values():
        c += 1 / len(depgraph)
        color = patches.colors.hsv_to_rgb((c, 1, 1))
        for points, adjListMap, path in deppaths:
            for i in range(1, len(path)):
                xpts = (points[path[i - 1]][0], points[path[i]][0])
                ypts = (points[path[i - 1]][1], points[path[i]][1])
                l = Line2D(xpts, ypts, color=color, zorder=2)
                ax.add_line(l)
                # ax.annotate(
                #     "",
                #     xy=points[path[i]],
                #     xytext=points[path[i - 1]],
                #     arrowprops=dict(arrowstyle="->")
                # )

                circ = patches.Circle(points[path[i - 1]], 0.03, color=color, zorder=3)
                ax.add_patch(circ)
                circ = patches.Circle(points[path[i]], 0.03, color=color, zorder=3)
                ax.add_patch(circ)

    plt.show()
    return


def isCollisionFree(robot, point, obstacles):
    '''
    Collision checking
    '''

    robotAt = np.add(point, robot)
    for poly in obstacles:
        if polysCollide(poly, robotAt):
            return False

    return True


def isCollisionFreeEdge(robot, edge, obstacles):
    '''
    Collision checking
    '''

    # robotEdge = symPolyConvexHull(robot, *edge)
    for poly in obstacles:
        for i in range(len(robot) - 1):
            roboPart = [robot[i], robot[i], robot[i + 1], robot[i + 1]]
            roboMask = [edge[0], edge[1], edge[1], edge[0]]
            robotEdge = np.add(roboPart, roboMask)
            # print(robotEdge)
            if polysCollide(poly, robotEdge):
                return False

    return True


def growPRMwObs(HEIGHT, WIDTH, robot, obstacles):
    '''
    Grow a simple PRM with collision checks
    '''

    radius = 100
    n = 50
    roadmap = Roadmap(HEIGHT, WIDTH, robot, obstacles, isCollisionFreeEdge, isCollisionFree)

    while n > 0:
        newPoint = roadmap.tryGrow(gamma=radius)
        # print(n)
        if not newPoint:
            continue
        n -= 1

        # if iters % 10 == 0:
        #     displayPathInGraph(
        #         rrtree.points, rrtree.adjListMap, [], np.add(robot, newPoint),
        #         np.add(robot, goalPoint), obstacles
        #     )

    return roadmap


def PRM(basemap, obstacles, startPoint, goalPoint):
    '''
    The full PRM algorithm
    '''

    path = []

    roadmap = basemap.copyBut(obstacles, isCollisionFreeEdge, None)
    startInd = roadmap.tryGrow(tryPoint=startPoint, r=lambda y, x: x, gamma=100)
    goalInd = roadmap.tryGrow(tryPoint=goalPoint, r=lambda y, x: x, gamma=100)
    print(startInd, goalInd)

    samples, graph = roadmap.getUndirected()

    path = basicSearch(graph, startInd, goalInd)

    return samples, graph, path


def growRRTwObs(HEIGHT, WIDTH, robot, obstacles, startPoint, goalPoint):
    '''
    Grow a simple RRT with collision checks
    '''

    radius = 10
    iters = 300
    rrtree = RRTree(HEIGHT, WIDTH, robot, obstacles, startPoint, isCollisionFreeEdge)
    if dist(startPoint, goalPoint) < radius:
        if rrtree.tryGrow(tryPoint=goalPoint):
            return rrtree.getUndirected()

    while iters > 0:
        iters -= 1
        # print(iters)
        newPoint = rrtree.tryGrow(radius)
        if not newPoint:
            continue

        # print(newPoint)

        # if iters % 10 == 0:
        #     displayPathInGraph(
        #         rrtree.points, rrtree.adjListMap, [], np.add(robot, newPoint),
        #         np.add(robot, goalPoint), obstacles
        #     )

        if dist(newPoint, goalPoint) < radius:
            if rrtree.tryGrow(tryPoint=goalPoint):
                return rrtree.getUndirected()

    return (dict().copy(), dict().copy())


def RRT(HEIGHT, WIDTH, robot, obstacles, startPoint, goalPoint):
    '''
    The full RRT algorithm
    '''

    points = dict()
    tree = dict()
    path = []

    points, tree = growRRTwObs(HEIGHT, WIDTH, robot, obstacles, startPoint, goalPoint)
    # displayPathInGraph(points, tree, path)
    path = basicSearch(tree, 1, len(points))

    return points, tree, path


def depSearch(paths, objs, start, goal):
    '''
    Perform basic search
    '''
    deps = []
    if len(paths) > 0:
        backtrace = {start: start}
        explore = deque([start])
        while goal not in backtrace:
            p = explore.pop()
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


def main(numObjs, display, displayMore, HEIGHT, WIDTH):
    # Read data and parse polygons

    points = []
    shapes = []
    objects = []
    for i in range(numObjs):
        polygon = np.array([[0, 0], [0.4, 0.4], [0.8, 0], [0.4, -0.4]])
        # for p in range(0, len(xys)):
        #     xy = xys[p].split(',')
        #     polygon.append((float(xy[0]), float(xy[1])))
        shapes.append(polygon)

        for i in range(2):
            isfree = False
            while not isfree:
                point = (
                    uniform(0, WIDTH - max(polygon[:, 0])),
                    uniform(0 - min(polygon[:, 1]), HEIGHT - max(polygon[:, 1]))
                )
                isfree = isCollisionFree(polygon, point, objects)  # [:len(objects) - i])
            points.append(point)
            objects.append(polygon + point)

    objects.append(
        [
            [-0.1, -0.1], [-0.1, HEIGHT + 0.1], [WIDTH + 0.1, HEIGHT + 0.1],
            [WIDTH + 0.1, HEIGHT], [0, HEIGHT], [0, -0.1]
        ]
    )
    objects.append(
        [
            [0, 0], [WIDTH, 0], [WIDTH, HEIGHT], [WIDTH + 0.1, HEIGHT], [WIDTH + 0.1, -0.1],
            [0, -0.1]
        ]
    )

    # print("Objects:")
    # for obj in objects:
    #     print(obj)
    # print()

    if display:
        drawProblem(HEIGHT, WIDTH, objects)

    basemap = growPRMwObs(HEIGHT, WIDTH, polygon, objects)
    # print("edges: ", len(basemap.adjListMap))
    samples, adjList = basemap.getUndirected()

    displayPathInGraph(HEIGHT, WIDTH, samples, adjList, polygons=objects)

    paths = {}
    for indStart, indGoal in combinations(range(numObjs * 2), 2):
        obstacles = objects[:indStart] + objects[indStart + 1:indGoal] + objects[indGoal + 1:]

        robotStart = objects[indStart]
        robotGoal = objects[indGoal]

        if displayMore:
            drawProblem(HEIGHT, WIDTH, obstacles, robotStart, robotGoal)

        nodes, adjListMap, path = PRM(basemap, obstacles, points[indStart], points[indGoal])
        paths[(indStart, indGoal)] = (nodes, adjListMap, path)
        print(indStart, indGoal, path)
        # print(points[indGoal], nodes[path[-1]])

        if displayMore:
            displayPathInGraph(
                HEIGHT, WIDTH, nodes, adjListMap, path, robotStart, robotGoal, obstacles
            )

    if display:
        displayRoadmap(HEIGHT, WIDTH, paths, objects)

    # GREEDY
    depgraph = {}
    for obj in range(numObjs):
        indStart = obj * 2  # * randint(0, numObjs - 1)
        indGoal = indStart + 1
        deps = depSearch(paths, 2 * numObjs, indStart, indGoal)
        depsPath = []
        for i in range(1, len(deps)):
            # nodes, adjListMap, path = paths[tuple(sorted([deps[i - 1], deps[i]]))]
            # depsPath += [nodes[x] for x in path]
            depsPath += [paths[tuple(sorted([deps[i - 1], deps[i]]))]]

        depgraph[indStart] = (deps, depsPath)
        print(indStart, deps)

    if display:
        displaySolution(HEIGHT, WIDTH, depgraph, objects)


if __name__ == "__main__":
    if (len(sys.argv) < 4):
        print(
            '''
            Error:
            deptree.py: <# objects> <height> <width> [display?: (y/n)] [display more?: (y/n)]
            '''
        )
        exit()

    try:
        numObjs = int(sys.argv[1])
        HEIGHT = int(sys.argv[2])
        WIDTH = int(sys.argv[3])
    except ValueError:
        print(
            '''
            Error:
            deptree.py: <# objects> <height> <width> [display?: (y/n)] [display more?: (y/n)]
            '''
        )
        exit()

    display = False
    if (len(sys.argv) > 4):
        display = sys.argv[4] not in ('n', 'N')

    displayMore = False
    if (len(sys.argv) > 5):
        displayMore = sys.argv[5] not in ('n', 'N')

    main(numObjs, display, displayMore, HEIGHT, WIDTH)
