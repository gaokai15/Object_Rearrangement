from __future__ import division
import sys
import json
from time import time
from random import uniform, random, choice
from collections import deque
from itertools import combinations

import numpy as np
import matplotlib
# matplotlib.use('agg')
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

from util import *
import visilibity as vis
import Polygon as pn
import Polygon.Utils as pu
import Polygon.Shapes as ps

import matplotlib.colors as colors
import matplotlib.cm as cmx

import IPython
import os

EPSILON = 2**-8


def setupPlot(HEIGHT, WIDTH):
    fig = plt.figure(num=None, figsize=(int(5 * WIDTH / HEIGHT), 5), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()
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
    patch = patches.PathPatch(path, facecolor=color, lw=0.5, zorder=zorder)
    return patch


def createPolygonPatch_distinct(polygon, color, isGoal, buffers=None, zorder=1):
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
        patch = patches.PathPatch(path, linestyle='--', edgecolor=color, facecolor="white", lw=1, zorder=2)
    else:
        patch = patches.PathPatch(path, facecolor=color, lw=0.5, zorder=3)

    if buffers == "buffers":
        patch = patches.PathPatch(path, linestyle='dashdot', edgecolor="black", facecolor="white", lw=1, zorder=1)

    return patch


def drawConGraph(
    HEIGHT,
    WIDTH,
    numObjs,
    RAD,
    example_index,
    paths,
    color_pool,
    points,
    polygons,
    buffers=None,
    saveimage=False,
):
    _, ax = setupPlot(HEIGHT, WIDTH)
    scale = max(HEIGHT, WIDTH)

    wallx = [0, WIDTH, WIDTH, 0, 0]
    wally = [0, 0, HEIGHT, HEIGHT, 0]
    plt.plot(wallx, wally, 'blue')

    for i in range(len(polygons)):
        obj_idx = i // 2
        isGoal = i % 2
        poly = polygons[i]
        if type(poly) == tuple:
            poly, _ = poly
        for cont in poly:
            patch = createPolygonPatch_distinct(cont, color_pool[obj_idx], isGoal)
            ax.add_patch(patch)
            if not isGoal:
                ax.text(points[i][0], points[i][1], str(obj_idx), fontweight='bold', fontsize=10, zorder=3)

    if buffers is not None:
        ### visualize buffers as black dotted circle
        for i in range(len(buffers)):
            poly = buffers[i]
            if type(poly) == tuple:
                poly, _ = poly
            for cont in poly:
                patch = createPolygonPatch_distinct(cont, color_pool[obj_idx], False, "buffers")
                ax.add_patch(patch)

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

    if saveimage:
        path = os.getcwd() + "/figures/"
        plt.savefig(
            path + str(numObjs) + "_" + str(int(RAD)) + "_" + str(HEIGHT) + "_" + str(WIDTH) + "_" +
            str(example_index) + "_cgraph.png"
        )
    plt.show()
    return


def drawArrangement(
    HEIGHT,
    WIDTH,
    numObjs,
    RAD,
    example_index,
    wall,
    polygons,
    points,
    curr_index,
    final_index,
    color_pool,
    arrange_idx,
    saveimage=False
):
    _, ax = setupPlot(HEIGHT, WIDTH)

    for walls in wall:
        # walls = pu.pointList(cont)
        wallx = [p[0] for p in walls + [walls[0]]]
        wally = [p[1] for p in walls + [walls[0]]]
        plt.plot(wallx, wally, 'blue')

    for i in range(len(polygons)):
        if i in curr_index:
            ### This is a current slot for an object
            obj_idx = curr_index.index(i)
            poly = polygons[i]
            if type(poly) == tuple:
                poly, _ = poly
            for cont in poly:
                patch = createPolygonPatch_distinct(cont, color_pool[obj_idx], isGoal=False)
                ax.add_patch(patch)
                ax.text(points[i][0], points[i][1], str(obj_idx), fontweight='bold', fontsize=10, zorder=3)
        if i in final_index:
            ### This is a final location for an object
            obj_idx = final_index.index(i)
            poly = polygons[i]
            if type(poly) == tuple:
                poly, _ = poly
            for cont in poly:
                patch = createPolygonPatch_distinct(cont, color_pool[obj_idx], isGoal=True)
                ax.add_patch(patch)
        if (i not in curr_index) and (i not in final_index):
            ### This is currently a buffer slot
            poly = polygons[i]
            if type(poly) == tuple:
                poly, _ = poly
            for cont in poly:
                patch = createPolygonPatch_distinct(cont, "black", False, "buffers")
                ax.add_patch(patch)

    if saveimage:
        path = os.getcwd() + "/figures/"
        plt.savefig(
            path + str(numObjs) + "_" + str(int(RAD)) + "_" + str(HEIGHT) + "_" + str(WIDTH) + "_" +
            str(example_index) + "_arrangement_" + str(arrange_idx) + ".png"
        )

    plt.show()
    return


def drawProblem(
    HEIGHT,
    WIDTH,
    numObjs,
    RAD,
    wall,
    polygons,
    points,
    color_pool,
    example_index,
    buffers=None,
    saveimage=False,
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

    ### Dan's version ###
    # c = 0
    # for p in range(0, len(polygons)):
    #   if p % 2 == 0:
    #       c += 2.0 / len(polygons)
    #   color = patches.colors.hsv_to_rgb((c, 1, 1))
    #   poly = polygons[p]
    #   for cont in poly:
    #       patch = createPolygonPatch_distinct(cont, color, p % 2)
    #       ax.add_patch(patch)
    #       ax.annotate(str(int(p/2)), xy=(0,0), xytext=pn.Polygon(cont).center())

    for i in range(len(polygons)):
        obj_idx = i // 2
        isGoal = i % 2
        poly = polygons[i]
        if type(poly) == tuple:
            poly, _ = poly
        for cont in poly:
            patch = createPolygonPatch_distinct(cont, color_pool[obj_idx], isGoal)
            ax.add_patch(patch)
            if not isGoal:
                ax.text(points[i][0], points[i][1], str(obj_idx), fontweight='bold', fontsize=10, zorder=3)

    if buffers is not None:
        ### visualize buffers as black dotted circle
        for i in range(len(buffers)):
            poly = buffers[i]
            if type(poly) == tuple:
                poly, _ = poly
            for cont in poly:
                patch = createPolygonPatch_distinct(cont, "black", False, "buffers")
                ax.add_patch(patch)

    if path is not None:
        pts, color = path
        pathx = [p[0] for p in pts]
        pathy = [p[1] for p in pts]
        plt.plot(pathx, pathy, color)

    if saveimage and (buffers is None):
        path = os.getcwd() + "/figures/"
        plt.savefig(
            path + str(numObjs) + "_" + str(int(RAD)) + "_" + str(HEIGHT) + "_" + str(WIDTH) + "_" +
            str(example_index) + "_original_problem.png"
        )

    if saveimage and (buffers is not None):
        path = os.getcwd() + "/figures/"
        plt.savefig(
            path + str(numObjs) + "_" + str(int(RAD)) + "_" + str(HEIGHT) + "_" + str(WIDTH) + "_" +
            str(example_index) + "_original_problem_with_buffers.png"
        )

    plt.show()
    return


def drawRegionGraph(HEIGHT, WIDTH, numObjs, RAD, example_index, paths, polygons, saveimage=False, mode="0", label=True):
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

    if saveimage and (paths == {}) and (mode == "0"):
        path = os.getcwd() + "/figures/"
        plt.savefig(
            path + str(numObjs) + "_" + str(int(RAD)) + "_" + str(HEIGHT) + "_" + str(WIDTH) + "_" +
            str(example_index) + "_region_graph_with_buffer.png"
        )

    if saveimage and (paths == {}) and (mode == "1"):
        path = os.getcwd() + "/figures/"
        plt.savefig(
            path + str(numObjs) + "_" + str(int(RAD)) + "_" + str(HEIGHT) + "_" + str(WIDTH) + "_" +
            str(example_index) + "_region_graph_without_buffer.png"
        )

    if saveimage and (paths is not {}):
        path = os.getcwd() + "/figures/"
        plt.savefig(
            path + str(numObjs) + "_" + str(int(RAD)) + "_" + str(HEIGHT) + "_" + str(WIDTH) + "_" +
            str(example_index) + "_region_graph_with_connections.png"
        )

    plt.show()
    return


def drawLocalMotions(
    HEIGHT,
    WIDTH,
    numObjs,
    RAD,
    arr_pair,
    paths,
    color_pool,
    points,
    curr_arrangement,
    final_arrangement,
    example_index,
    polygons,
    saveimage=True
):
    _, ax = setupPlot(HEIGHT, WIDTH)
    scale = max(HEIGHT, WIDTH)

    wallx = [0, WIDTH, WIDTH, 0, 0]
    wally = [0, 0, HEIGHT, HEIGHT, 0]
    plt.plot(wallx, wally, 'blue')

    if polygons is not None:
        for i in range(len(polygons)):
            if i in curr_arrangement:
                obj_idx = curr_arrangement.index(i)
                patch = createPolygonPatch_distinct(pu.pointList(polygons[i]), color_pool[obj_idx], isGoal=False)
                ax.add_patch(patch)
                ax.text(points[i][0], points[i][1], str(obj_idx), fontweight='bold', fontsize=10, zorder=3)
            if i in final_arrangement:
                obj_idx = final_arrangement.index(i)
                patch = createPolygonPatch_distinct(pu.pointList(polygons[i]), color_pool[obj_idx], isGoal=True)
                ax.add_patch(patch)
            if (i not in curr_arrangement) and (i not in final_arrangement):
                ### This is a buffer
                patch = createPolygonPatch_distinct(pu.pointList(polygons[i]), "black", False, "buffers")
                ax.add_patch(patch)

    rads = {}
    # cc = 0.0
    for obj, path in paths.items():
        color = color_pool[obj]
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
            str(example_index) + "_(" + str(arr_pair[0]) + "--" + str(arr_pair[1]) + ").png"
        )
    # plt.show()
    plt.close()
    return


def drawMotions(
    HEIGHT,
    WIDTH,
    numObjs,
    RAD,
    paths,
    color_pool,
    points,
    curr_arrangement,
    final_arrangement,
    example_index,
    polygons,
    saveimage,
):
    _, ax = setupPlot(HEIGHT, WIDTH)
    scale = max(HEIGHT, WIDTH)

    wallx = [0, WIDTH, WIDTH, 0, 0]
    wally = [0, 0, HEIGHT, HEIGHT, 0]
    plt.plot(wallx, wally, 'blue')

    if polygons is not None:
        for i in range(len(polygons)):
            if i in curr_arrangement:
                obj_idx = curr_arrangement.index(i)
                patch = createPolygonPatch_distinct(pu.pointList(polygons[i]), color_pool[obj_idx], isGoal=False)
                ax.add_patch(patch)
                ax.text(points[i][0], points[i][1], str(obj_idx), fontweight='bold', fontsize=10, zorder=3)
            if i in final_arrangement:
                obj_idx = final_arrangement.index(i)
                patch = createPolygonPatch_distinct(pu.pointList(polygons[i]), color_pool[obj_idx], isGoal=True)
                ax.add_patch(patch)
            if (i not in curr_arrangement) and (i not in final_arrangement):
                ### This is a buffer
                patch = createPolygonPatch_distinct(pu.pointList(polygons[i]), "black", False, "buffers")
                ax.add_patch(patch)

    rads = {}
    # cc = 0.0
    for obj, path in paths.items():
        color = color_pool[obj]
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


def drawEntireAnimation(
    HEIGHT, WIDTH, numObjs, RAD, plan, final_arrangement, color_pool, points, example_index, objectShape=None
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

    final_pts = [points[i] for i in final_arrangement]  ### This is always fixed

    ### give a prompt to start animation
    raw_input("Press <ENTER> to start animation")

    # for arr_pair, rpaths in plan.whole_path.items():
    for path_segment in plan.whole_path:
        arr_pair = path_segment[0]
        rpaths = path_segment[1]
        ### first collect the current pose for each object
        current_pts = []
        for obj in range(numObjs):
            current_pts.append(rpaths[obj][0])
        ### work on current path of the current object
        for obj_idx, path in rpaths.items():
            for wpt_idx in range(len(path) - 1):
                pt1 = path[wpt_idx]
                pt2 = path[wpt_idx + 1]
                step1 = int(abs(pt1[0] - pt2[0]) / 50.0)
                step2 = int(abs(pt1[1] - pt2[1]) / 50.0)
                if step1 == 0 and step2 == 0:
                    n_steps = 1
                else:
                    n_steps = max(step1, step2)
                for step in range(n_steps + 1):
                    ### update current_pts
                    current_pts[obj_idx] = (
                        pt1[0] + (pt2[0] - pt1[0]) / n_steps * step, pt1[1] + (pt2[1] - pt1[1]) / n_steps * step
                    )

                    ax.cla()
                    ax.plot(wallx, wally, 'blue')

                    ### plot the objects
                    for nn in range(len(current_pts)):
                        polygon = pn.Polygon(objectShape + current_pts[nn])
                        patch = createPolygonPatch_distinct(
                            pu.pointList(polygon), color_pool[nn], isGoal=False, zorder=3
                        )
                        ax.add_patch(patch)
                        ax.text(
                            current_pts[nn][0], current_pts[nn][1], str(nn), fontweight='bold', fontsize=10, zorder=3
                        )
                    ### plot goal poses and buffers
                    for pose_idx in range(len(points)):
                        if pose_idx in final_arrangement:
                            ### It's a goal pose
                            nn = final_arrangement.index(pose_idx)
                            polygon = pn.Polygon(objectShape + final_pts[nn])
                            patch = createPolygonPatch_distinct(
                                pu.pointList(polygon), color_pool[nn], isGoal=True, zorder=2
                            )
                            ax.add_patch(patch)
                        else:
                            ### It's a buffer
                            polygon = pn.Polygon(objectShape + points[pose_idx])
                            patch = createPolygonPatch_distinct(pu.pointList(polygon), "black", False, "buffers")
                            ax.add_patch(patch)

                    plt.pause(0.0000005)

    plt.show()
    return


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
    goal_pts = points[1::2]
    isGoalReached = [False for i in range(numObjs)]

    for obj in object_ordering:
        for wpt_idx in range(len(rpaths[(obj * 2, obj * 2 + 1)]) - 1):
            pt1 = rpaths[(obj * 2, obj * 2 + 1)][wpt_idx]
            pt2 = rpaths[(obj * 2, obj * 2 + 1)][wpt_idx + 1]
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
                    ### current
                    polygon = pn.Polygon(objectShape + current_pts[nn])
                    patch = createPolygonPatch_distinct(pu.pointList(polygon), color_pool[nn], isGoal=False, zorder=3)
                    ax.add_patch(patch)
                    ax.text(current_pts[nn][0], current_pts[nn][1], str(nn), fontweight='bold', fontsize=10, zorder=3)
                    ### goal
                    if not isGoalReached[nn]:
                        polygon = pn.Polygon(objectShape + goal_pts[nn])
                        patch = createPolygonPatch_distinct(
                            pu.pointList(polygon), color_pool[nn], isGoal=True, zorder=1
                        )
                        ax.add_patch(patch)
                        ax.text(goal_pts[nn][0], goal_pts[nn][1], str(nn), fontweight='bold', fontsize=10, zorder=1)

                plt.pause(0.0000005)

    plt.show()
    return


def getColorMap(numObjs):
    ### set colormap
    gist_ncar = plt.get_cmap('gist_ncar')
    cNorm = colors.Normalize(vmin=0, vmax=1)
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=gist_ncar)
    color_values = np.linspace(0, 0.9, numObjs)
    color_pool = [scalarMap.to_rgba(color_values[ii]) for ii in xrange(numObjs)]

    return color_pool


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
