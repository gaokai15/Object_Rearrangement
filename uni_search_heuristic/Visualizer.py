from __future__ import division

import matplotlib
# matplotlib.use('agg')
import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches
import matplotlib.colors as colors
import matplotlib.cm as cmx

import visilibity as vis
import Polygon as pn
import Polygon.Utils as pu
import Polygon.Shapes as ps

import numpy as np
import IPython

class Visualizer(object):
    def __init__(self, HEIGHT, WIDTH, numObjs, wall_pts, display, displayMore, saveimage, data_path, solution_subfolder, tree_subfolder):
        self.HEIGHT = HEIGHT
        self.WIDTH = WIDTH
        self.numObjs = numObjs
        self.display = display
        self.displayMore = displayMore
        self.saveimage = saveimage
        self.color_pool = self.getColorMap()
        self.wall_pts = wall_pts
        self.data_path = data_path
        self.solution_subfolder = solution_subfolder
        self.tree_subfolder = tree_subfolder

    def getColorMap(self):
        ### This function assign a distinct color for each object
        ### set colormap
        gist_ncar = plt.get_cmap('gist_ncar')
        cNorm = colors.Normalize(vmin=0, vmax=1)
        scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=gist_ncar)
        color_values = np.linspace(0, 0.9, self.numObjs)
        color_pool = [scalarMap.to_rgba(color_values[ii]) for ii in xrange(self.numObjs)]
        
        return color_pool
 

    def drawProblem(self, objects, points, buffers=[]):
        ### This function is only used to draw the initial arrangment problem 

        ### set the canvas
        _, ax = self.setupPlot()
        ### plot the wall first
        for wall_pt in self.wall_pts:
            wallx = [p[0] for p in wall_pt + [wall_pt[0]]]
            wally = [p[1] for p in wall_pt + [wall_pt[0]]]
            plt.plot(wallx, wally, 'blue')

        ### plot objects' current and goal locations
        for i in range(len(objects)):
            obj_idx = i // 2
            isGoal = i % 2
            poly = objects[i]
            if type(poly) == tuple:
                poly, _ = poly
            for cont in poly:
                patch = self.createPolygonPatch_distinct(cont, self.color_pool[obj_idx], isGoal)
                ax.add_patch(patch)
                if not isGoal:
                    ax.text(points[i][0], points[i][1], str(obj_idx), fontweight='bold', fontsize=10, zorder=3)

        ### draw buffers if there is any
        if len(buffers) != 0:
            ### visualize buffers as black dotted circles
            for i in range(len(buffers)):
                poly = buffers[i]
                if type(poly) == tuple:
                    poly, _ = poly
                for cont in poly:
                    patch = self.createPolygonPatch_distinct(cont, "black", False, "buffers")
                    ax.add_patch(patch)

        ### decide whether you want to visualize and save the images
        if self.saveimage and (buffers == []):
            plt.savefig(self.data_path + "/original_problem.png")
        if self.saveimage and (buffers != []):
            plt.savefig(self.data_path + "/original_problem_with_buffers.png")
        if self.display:
            plt.show()
        plt.close()
        return


    def drawRegionGraph(self, paths, regions, label=True):
        ### This function draws the region graph
        _, ax = self.setupPlot()
        scale = max(self.HEIGHT, self.WIDTH)

        wallx = [0, self.WIDTH, self.WIDTH, 0, 0]
        wally = [0, 0, self.HEIGHT, self.HEIGHT, 0]
        plt.plot(wallx, wally, 'blue')

        if regions is not None:
            c = 0
            for p in range(0, len(regions)):
                if label:
                    if p % 2 == 0:
                        c += 2.0 / len(regions)
                else:
                    c += 1.0 / len(regions)
                color = patches.colors.hsv_to_rgb((c, 1, 1))
                poly = regions[p]
                if type(poly) == tuple:
                    poly, _ = poly
                for cont in poly:
                    patch = self.createPolygonPatch_distinct(cont, color, label and p % 2)
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

        if self.saveimage and paths == {}:
            plt.savefig(self.data_path + "/regionGraph.png")        
        if self.saveimage and paths != {}:
            plt.savefig(self.data_path + "/regionGraph_connection.png")
        if self.display:
            plt.show()
        plt.close()
        return


    def drawConGraph(self, paths, points, objects, buffers=[]):
        ### This function draws the connectivity graph
        _, ax = self.setupPlot()
        scale = max(self.HEIGHT, self.WIDTH)

        wallx = [0, self.WIDTH, self.WIDTH, 0, 0]
        wally = [0, 0, self.HEIGHT, self.HEIGHT, 0]
        plt.plot(wallx, wally, 'blue')

        ### plot the objects' current and final locations
        for i in range(len(objects)):
            obj_idx = i // 2
            isGoal = i % 2
            poly = objects[i]
            if type(poly) == tuple:
                poly, _ = poly
            for cont in poly:
                patch = self.createPolygonPatch_distinct(cont, self.color_pool[obj_idx], isGoal)
                ax.add_patch(patch)
                if not isGoal:
                    ax.text(points[i][0], points[i][1], str(obj_idx), fontweight='bold', fontsize=10, zorder=3)
        ### plot the buffers
        if buffers != []:
            ### visualize buffers as black dotted circle
            for i in range(len(buffers)):
                poly = buffers[i]
                if type(poly) == tuple:
                    poly, _ = poly
                for cont in poly:
                    patch = self.createPolygonPatch_distinct(cont, "black", False, "buffers")
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

        if self.saveimage:
            plt.savefig(self.data_path + "/cgraph.png")
        if self.display:
            plt.show()
        plt.close()
        return


    def drawLocalMotions(self, arr_pair, paths, points, poses, curr_arrangement, final_arrangement, savewhere, debug=False):
        ### This function draws the local motions between two neighboring arrangement state
        ### Input: 1. arr_pair (curr_arrangement_id, next_arrangement_id)
        ###        2. paths: point paths incurred by the transition from curr_arrangement to next_arrangements
        ###        3. points, poses: for visualization
        ###        4: curr_arrangement: indicate where these poses currently are
        ###        5. final_arrangement: always show their destination
        ###        6. saveFolder: specify where to store these figures if saveimage mode is on
        ###        7. debug: set to be true if you want to see each image during the process

        ### Output: images saved with name "id-id" indicating the transition from one arrangement state to the other
        
        _, ax = self.setupPlot()
        scale = max(self.HEIGHT, self.WIDTH)

        wallx = [0, self.WIDTH, self.WIDTH, 0, 0]
        wally = [0, 0, self.HEIGHT, self.HEIGHT, 0]
        plt.plot(wallx, wally, 'blue')

        ### first draw all the poses given curr_arrangement and goal_arrangement
        if poses is not None:
            for pose_idx in range(len(poses)):
                if pose_idx in curr_arrangement:
                    ### draw curr pose with obj color with concrete circle
                    obj_idx = curr_arrangement.index(pose_idx)
                    patch = self.createPolygonPatch_distinct(
                                        pu.pointList(poses[pose_idx]), self.color_pool[obj_idx], isGoal=False)
                    ax.add_patch(patch)
                    ax.text(points[pose_idx][0], points[pose_idx][1], str(obj_idx), fontweight='bold', fontsize=10, zorder=3)

                if pose_idx in final_arrangement:
                    ### draw final pose with obj color with dashed circle
                    obj_idx = final_arrangement.index(pose_idx)
                    patch = self.createPolygonPatch_distinct(
                                        pu.pointList(poses[pose_idx]), self.color_pool[obj_idx], isGoal=True)
                    ax.add_patch(patch)

                if (pose_idx not in curr_arrangement) and (pose_idx not in final_arrangement):
                    ### This is a buffer
                    ### draw the buffer pose with black dashed-dotted circle
                    patch = self.createPolygonPatch_distinct(pu.pointList(poses[pose_idx]), "black", False, "buffers")
                    ax.add_patch(patch)


        ### Now let's draw the point paths given paths
        rads = {}
        # cc = 0.0
        for obj, path in paths.items():
            color = self.color_pool[obj]
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

        if self.saveimage and debug==False and savewhere=="save-to-solution":
            plt.savefig(self.solution_subfolder + "/" + arr_pair[0] + "--" + arr_pair[1] + ".png")
        if self.saveimage and debug==False and savewhere=="save-to-tree":
            plt.savefig(self.tree_subfolder + "/" + arr_pair[0] + "--" + arr_pair[1] + ".png")

        if self.display or debug==True:
            plt.show()
        plt.close()
        return


    def drawSolutionPaths(self, plan, instance, final_arrangement):
        ### let's save all local paths to see if it makes sense
        for local_path in plan.whole_path:
            arr_pair = local_path[0]
            curr_arrangement_id = arr_pair[0]
            next_arrangement_id = arr_pair[1]
            ppaths = local_path[1]
            if curr_arrangement_id[0] == "L":
                self.drawLocalMotions(
                    (curr_arrangement_id, next_arrangement_id), ppaths, \
                        instance.points+instance.buffer_points, instance.objects+instance.buffers, \
                            plan.treeL[curr_arrangement_id].arrangement, final_arrangement, "save-to-solution")
            if curr_arrangement_id[0] == "R":
                self.drawLocalMotions(
                    (curr_arrangement_id, next_arrangement_id), ppaths, \
                        instance.points+instance.buffer_points, instance.objects+instance.buffers, \
                            plan.treeR[curr_arrangement_id].arrangement, final_arrangement, "save-to-solution")


    def drawCurrArrangement(self, arr_id, points, poses, curr_arrangement, final_arrangement, savewhere, debug=False):
        ### This function draws the current rearrangement without showing the path (the way to reach this arrangement state)
        ### Input: 1. points, poses: for visualization
        ###        2. curr_arrangement: indicate where these poses currently are
        ###        3. final_arrangement: always show their destination
        ###        4. saveFolder: specify where to store these figures if saveimage mode is on
        ###        5. debug: set to be true if you want to see each image during the process

        ### Output: images saved with the id in the tree

        _, ax = self.setupPlot()
        scale = max(self.HEIGHT, self.WIDTH)

        wallx = [0, self.WIDTH, self.WIDTH, 0, 0]
        wally = [0, 0, self.HEIGHT, self.HEIGHT, 0]
        plt.plot(wallx, wally, 'blue')

        ### first draw all the poses given curr_arrangement and goal_arrangement
        if poses is not None:
            for pose_idx in range(len(poses)):
                if pose_idx in curr_arrangement:
                    ### draw curr pose with obj color with concrete circle
                    obj_idx = curr_arrangement.index(pose_idx)
                    patch = self.createPolygonPatch_distinct(
                                        pu.pointList(poses[pose_idx]), self.color_pool[obj_idx], isGoal=False)
                    ax.add_patch(patch)
                    ax.text(points[pose_idx][0], points[pose_idx][1], str(obj_idx), fontweight='bold', fontsize=10, zorder=3)

                if pose_idx in final_arrangement:
                    ### draw final pose with obj color with dashed circle
                    obj_idx = final_arrangement.index(pose_idx)
                    patch = self.createPolygonPatch_distinct(
                                        pu.pointList(poses[pose_idx]), self.color_pool[obj_idx], isGoal=True)
                    ax.add_patch(patch)

                if (pose_idx not in curr_arrangement) and (pose_idx not in final_arrangement):
                    ### This is a buffer
                    ### draw the buffer pose with black dashed-dotted circle
                    patch = self.createPolygonPatch_distinct(pu.pointList(poses[pose_idx]), "black", False, "buffers")
                    ax.add_patch(patch)

        if self.saveimage and debug==False and savewhere=="save-to-solution":
            plt.savefig(self.solution_subfolder + "/" + arr_id + ".png")
        if self.saveimage and debug==False and savewhere=="save-to-tree":
            plt.savefig(self.tree_subfolder + "/" + arr_id + ".png")

        if self.display or debug==True:
            plt.show()
        plt.close()
        return

        



    def drawEntireAnimation(self, plan, instance, final_arrangement):
        ### This function is used for display the animation
        ### Input: 1. plan: the resulting plan (we need the whole path from there and some key nodes on the tree)
        ###        2. instance: that is used for visualization all poses of the given shape
        ###        3. final_arrangment: show the destination, always needed

        ### Output: the animation showing the whole movement (solution)


        # n_steps = 5

        ### set the canvas
        fig, ax = self.setupPlot()
        wallx = [0, self.WIDTH, self.WIDTH, 0, 0]
        wally = [0, 0, self.HEIGHT, self.HEIGHT, 0]
        ax.plot(wallx, wally, 'blue')
        plt.show(block=False)

        final_pts = [instance.points[i] for i in final_arrangement]  ### This is always fixed

        ### give a prompt to start animation
        raw_input("Press <ENTER> to start animation")

        # for arr_pair, ppaths in plan.whole_path.items():
        for path_segment in plan.whole_path:
            arr_pair = path_segment[0]
            ppaths = path_segment[1]

            ### first collect the current pose for each object
            current_pts = []
            for obj in range(self.numObjs):
                current_pts.append(ppaths[obj][0])

            ### work on current path of the current object
            for obj_idx, path in ppaths.items():
                for wpt_idx in range(len(path)-1):
                    pt1 = path[wpt_idx]
                    pt2 = path[wpt_idx+1]
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
                            polygon = pn.Polygon(instance.polygon + current_pts[nn])
                            patch = self.createPolygonPatch_distinct(
                                pu.pointList(polygon), self.color_pool[nn], isGoal=False, zorder=3)
                            ax.add_patch(patch)
                            ax.text(current_pts[nn][0], current_pts[nn][1], str(nn), 
                                                                fontweight='bold', fontsize=10, zorder=3)
                        ### plot goal poses and buffers
                        for pose_idx in range(len(instance.points)):
                            if pose_idx in final_arrangement:
                                ### It's a goal pose
                                nn = final_arrangement.index(pose_idx)
                                polygon = pn.Polygon(instance.polygon + final_pts[nn])
                                patch = self.createPolygonPatch_distinct(
                                    pu.pointList(polygon), self.color_pool[nn], isGoal=True, zorder=2)
                                ax.add_patch(patch)
                            else:
                                ### It's a buffer
                                polygon = pn.Polygon(instance.polygon + instance.points[pose_idx])
                                patch = self.createPolygonPatch_distinct(
                                    pu.pointList(polygon), "black", False, "buffers")
                                ax.add_patch(patch)

                        plt.pause(0.0000005)

        plt.show()
        return


    def drawEntireAnimation1(self, plan, instance, final_arrangement):
        ### This function is used for display the animation
        ### Input: 1. plan: the resulting plan (we need the whole path from there and some key nodes on the tree)
        ###        2. instance: that is used for visualization all poses of the given shape
        ###        3. final_arrangment: show the destination, always needed

        ### Output: the animation showing the whole movement (solution) 


        # n_steps = 5

        ### set the canvas
        fig, ax = self.setupPlot()
        wallx = [0, self.WIDTH, self.WIDTH, 0, 0]
        wally = [0, 0, self.HEIGHT, self.HEIGHT, 0]
        ax.plot(wallx, wally, 'blue')
        plt.show(block=False)

        final_pts = [instance.points[i] for i in final_arrangement]  ### This is always fixed

        ### give a prompt to start animation
        raw_input("Press <ENTER> to start animation")

        # for arr_pair, ppaths in plan.whole_path.items():
        for path_segment in plan.whole_path:
            arr_pair = path_segment[0]
            ppaths = path_segment[1]

            if arr_pair[0][0] == "L":
                temp_curr_arrangement = plan.treeL[arr_pair[0]].arrangement
            else:
                temp_curr_arrangement = plan.treeR[arr_pair[0]].arrangement

            ### first collect the current pose for each object
            current_pts = []
            for obj in range(self.numObjs):
                current_pts.append(plan.points[temp_curr_arrangement[obj]])

            ### work on current path of the current object
            for obj_idx, path in ppaths.items():
                for wpt_idx in range(len(path)-1):
                    pt1 = path[wpt_idx]
                    pt2 = path[wpt_idx+1]
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
                            polygon = pn.Polygon(instance.polygon + current_pts[nn])
                            patch = self.createPolygonPatch_distinct(
                                pu.pointList(polygon), self.color_pool[nn], isGoal=False, zorder=3)
                            ax.add_patch(patch)
                            ax.text(current_pts[nn][0], current_pts[nn][1], str(nn), 
                                                                fontweight='bold', fontsize=10, zorder=3)
                        ### plot goal poses and buffers
                        for pose_idx in range(len(instance.points)):
                            if pose_idx in final_arrangement:
                                ### It's a goal pose
                                nn = final_arrangement.index(pose_idx)
                                polygon = pn.Polygon(instance.polygon + final_pts[nn])
                                patch = self.createPolygonPatch_distinct(
                                    pu.pointList(polygon), self.color_pool[nn], isGoal=True, zorder=2)
                                ax.add_patch(patch)
                            else:
                                ### It's a buffer
                                polygon = pn.Polygon(instance.polygon + instance.points[pose_idx])
                                patch = self.createPolygonPatch_distinct(
                                    pu.pointList(polygon), "black", False, "buffers")
                                ax.add_patch(patch)

                        plt.pause(0.0000005)

        plt.show()
        return


    def createPolygonPatch_distinct(self, polygon, color, isGoal, buffers=None, zorder=1):
        ### this function create polygon patch based on the object role
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

    def setupPlot(self):
        ### this function setup the workspace
        fig = plt.figure(num=None, figsize=(int(5*self.WIDTH/self.HEIGHT), 5), dpi=120, facecolor='w', edgecolor='k')
        ax = fig.subplots()
        return fig, ax


if __name__ == "__main__":
    print("welcome to Visualizer! Please call it from Experiment.py\n")