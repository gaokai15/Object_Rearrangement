from __future__ import division
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

import matplotlib.colors as colors
import matplotlib.cm as cmx

import IPython
import os

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
    patch = patches.PathPatch(path, linestyle='-', facecolor=color, lw=1, zorder=zorder)
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
        patch = patches.PathPatch(path, facecolor=color, lw=1, zorder=zorder)
    
    return patch

def drawProblem(HEIGHT, WIDTH, numObjs, RAD, wall, polygons, color_pool, points, example_index, saveimage, path=None, robotStart=None, robotGoal=None):
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

    for i in range(len(polygons)):
        obj_idx = i // 2
        isGoal = i % 2
        patch = createPolygonPatch_distinct(pu.pointList(polygons[i]), color_pool[obj_idx], isGoal)
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
        plt.savefig(path + str(numObjs) + "_" + str(int(RAD)) + "_" + str(HEIGHT) + "_" + str(WIDTH) + "_" + str(example_index) + "_object_deployment.png")
    plt.show()




def drawConGraph(HEIGHT, WIDTH, numObjs, RAD, paths, color_pool, points, example_index, saveimage, polygons=None):
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
        ### label these polygon
        for i in range(len(points)):
            obj_idx = i // 2
            ax.text(points[i][0], points[i][1], str(obj_idx), fontweight='bold', fontsize=10, zorder=3)

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
    #         patch = createPolygonPatch(pu.pointList(polygons[p]), color)
    #         ax.add_patch(patch)


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
    if saveimage:    
        path = os.getcwd() + "/figures/"
        plt.savefig(path + str(numObjs) + "_" + str(int(RAD)) + "_" + str(HEIGHT) + "_" + str(WIDTH) + "_" + str(example_index) + "_cgraph.png")
    plt.show()
    return


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

def setupPlot(HEIGHT, WIDTH):
    fig = plt.figure(num=None, figsize=(int(5*WIDTH/HEIGHT), 5), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()
    return fig, ax

def animatedMotions(HEIGHT, WIDTH, numObjs, RAD, rpaths, color_pool, points, object_ordering, example_index, objectShape=None):

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
    goal_pts = points[1::2]
    isGoalReached = [False for i in range(numObjs)]

    for obj in object_ordering:
        for wpt_idx in range(len(rpaths[(obj*2, obj*2+1)])-1):
            pt1 = rpaths[(obj*2, obj*2+1)][wpt_idx]
            pt2 = rpaths[(obj*2, obj*2+1)][wpt_idx+1]
            step1 = int(abs(pt1[0]-pt2[0]) / 50.0)
            step2 = int(abs(pt1[1]-pt2[1]) / 50.0)
            if step1 == 0 and step2 == 0:
                n_steps = 1
            else:
                n_steps = max(step1, step2)
            for step in range(n_steps+1):
                ### update current_pts
                current_pts[obj] = ( pt1[0]+(pt2[0]-pt1[0])/n_steps*step,  pt1[1]+(pt2[1]-pt1[1])/n_steps*step)
                ### check if the current object reaches the goal
                if current_pts[obj] == goal_pts[obj]:
                    isGoalReached[obj] = True            
                
                ax.cla()
                ax.plot(wallx, wally, 'blue')

                ### plot the objects
                for nn in range(len(current_pts)):
                    ### current
                    polygon = pn.Polygon(objectShape + current_pts[nn])
                    patch = createPolygonPatch_distinct(pu.pointList(polygon), color_pool[nn], isGoal=False, zorder=3)
                    ax.add_patch(patch)
                    ax.text(current_pts[nn][0], current_pts[nn][1], str(nn), fontweight='bold', fontsize=10, zorder=3)
                    ### goal
                    if not isGoalReached[nn]:
                        polygon = pn.Polygon(objectShape + goal_pts[nn])
                        patch = createPolygonPatch_distinct(pu.pointList(polygon), color_pool[nn], isGoal=True, zorder=1)
                        ax.add_patch(patch)
                        ax.text(goal_pts[nn][0], goal_pts[nn][1], str(nn), fontweight='bold', fontsize=10, zorder=1)
                
                plt.pause(0.0000005)

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
    cc = 0.0
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
                arrowprops=dict(
                    arrowstyle="simple", connectionstyle="arc3,rad=" + str(rad), color=color
                )
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
                arrowprops=dict(
                    arrowstyle="->", connectionstyle="arc3,rad=" + str(rad), color="black"
                )
            )

            pcolor = "black"
            circ = patches.Circle(path[i - 1], scale / 200.0, color=pcolor, zorder=3)
            ax.add_patch(circ)
            circ = patches.Circle(path[i], scale / 200.0, color=pcolor, zorder=3)
            ax.add_patch(circ)
    if saveimage:
        path = os.getcwd() + "/figures/"
        plt.savefig(path + str(numObjs) + "_" + str(int(RAD)) + "_" + str(HEIGHT) + "_" + str(WIDTH) + "_" + str(example_index) + "_paths.png")
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
    gist_ncar = cm = plt.get_cmap('gist_ncar')
    cNorm  = colors.Normalize(vmin=0, vmax=1)
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=gist_ncar)

    color_values = np.linspace(0, 0.9, numObjs)
    color_pool = [scalarMap.to_rgba(color_values[ii]) for ii in xrange(numObjs)]

    return color_pool


def genCGraph(numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, example_index):

    ### get some colors from colormap
    color_pool = getColorMap(numObjs)   

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
        print("starting object ", i)

        for j in range(2):
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
                print("cannot find the " + str(j) + " for object " + str(i))
                # print("Failed to generate!")
                # return False, False, False
                return False, False, False, False, False, False

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
        print("finishing object ", i)
    
    print("finish generating the points\n")
    if display:
        drawProblem(HEIGHT, WIDTH, numObjs, RAD, wall_pts, objects, color_pool, points, example_index, saveimage)
        print("The deployment of objects is shown!")
    
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
            print("Displaymore mode!")
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
        print("Time to draw the connectivity graph")
        drawConGraph(HEIGHT, WIDTH, numObjs, RAD, paths, color_pool, points, example_index, saveimage, objects)
    
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
    
    return graph, paths, objects, color_pool, points, polygon
    


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
