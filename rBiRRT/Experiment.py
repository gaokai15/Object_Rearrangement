from __future__ import division

import os
import sys
import numpy as np
from time import clock
import copy
from random import sample
from rgraph import *

from InstanceGenerator import InstanceGenerator
from Visualizer import Visualizer
from RegionGraphGenerator import RegionGraphGenerator
from DensePathGenerator import DensePathGenerator
from BiRRTPlanner import BiRRTPlanner
from BiRRTStarPlanner import BiRRTStarPlanner
from BiRRT_tester import BiRRT_tester
from BiRRTstar_tester import BiRRTstar_tester
from BiDirDPPlanner import BiDirDPPlanner


class Experiment(object):
    def __init__(
        self, numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, savestat, exp_id, data_path
    ):
        ### essential member
        self.numObjs = numObjs
        self.RAD = RAD
        self.HEIGHT = HEIGHT
        self.WIDTH = WIDTH
        self.display = display
        self.displayMore = displayMore
        self.savefile = savefile
        self.saveimage = saveimage
        self.savestat = savestat
        self.exp_id = exp_id
        self.data_path = self.createDirForExp(data_path)

        self.polygon = np.array(poly_disk([0, 0], self.RAD, 30))  ### object shape
        self.wall_pts = [[0, 0], [self.WIDTH, 0], [self.WIDTH, self.HEIGHT], [0, self.HEIGHT]]  ### workspace
        ### wall_mink: inner boundary to limit the center of the object
        self.wall_mink = region(
            [
                [self.RAD, self.RAD],
                [self.WIDTH - self.RAD, self.RAD],
                [self.WIDTH - self.RAD, self.HEIGHT - self.RAD],
                [self.RAD, self.HEIGHT - self.RAD],
            ], False
        )

        self.genInstanceFailure = False
        ### we need an instance generator (at most generate for 20 trials)
        self.instance = InstanceGenerator(
            self.numObjs, self.HEIGHT, self.WIDTH, self.polygon
        )  ### an InstanceGenerator object
        if (self.instance.points == False):
            self.genInstanceFailure = True
            print("failed to generating a valid instance at exp " + str(exp_id))
            return

        ### only continue when the instance is successfully generated
        ### an Visualizer object
        self.visualTool = Visualizer(self.HEIGHT, self.WIDTH, self.numObjs, self.wall_pts, \
                                            self.display, self.displayMore, self.saveimage, self.data_path)
        self.instance.genBuffers(self.HEIGHT, self.WIDTH, self.numObjs)
        # self.instance.genBuffers2(self.wall_mink, self.numObjs)
        self.visualTool.drawProblem(self.instance.objects, self.instance.points, self.instance.buffers)
        print("finishing generating the instance and the buffers")
        ### index the arrangement
        self.initial_arrangement = [i for i in range(0, 2 * numObjs, 2)]
        self.final_arrangement = [i for i in range(1, 2 * numObjs, 2)]
        print "initial_arrangement: " + str(self.initial_arrangement)
        print "final_arrangement: " + str(self.final_arrangement)

        ### Now let's generate the region graph and build its connection
        self.regionGraph = RegionGraphGenerator(self.instance, self.visualTool, self.wall_mink)
        ### get the region dict and LL from the graph
        region_dict, linked_list = self.linked_list_conversion(self.regionGraph.graph)

        self.genSolutionFailure_DP_local = False
        start_time = clock()
        self.plan_DP_local = BiDirDPPlanner(
            self.initial_arrangement, self.final_arrangement, self.instance, self.regionGraph.obj2reg, region_dict,
            linked_list, self.visualTool, self.wall_mink
        )
        self.comTime_DP_local = clock() - start_time
        print "Time to perform BiDirectional search with DP local solver: " + str(self.comTime_DP_local)

        if self.plan_DP_local.isConnected == False:
            ### the solution is not found
            self.genSolutionFailure_DP_local = True
        else:
            self.totalActions_DP_local = self.plan_DP_local.totalActions

        self.genSolutionFailure_biRRT = True
        self.genSolutionFailure_biRRTstar = True

        # if self.saveimage or self.display:
        #     self.visualTool.displayLocalPaths(self.plan_DP_local, self.instance, self.final_arrangement)
        # if self.display:
        #     self.visualTool.drawEntireAnimation(self.plan_DP_local, self.instance, self.final_arrangement)
        return

        ### Now let's generate the region graph and build its connection
        self.regionGraph = RegionGraphGenerator(self.instance, self.visualTool, self.wall_mink)
        ### Now let's generate path options
        start_time = clock()
        self.gpd = DensePathGenerator(self.regionGraph.graph, self.regionGraph.obj2reg)
        print "total time for path generation: " + str(clock() - start_time)
        print "total path connections: " + str(len(self.gpd.dependency_dict))

        self.new_paths = {}
        for r1, r2 in self.regionGraph.paths.keys():
            self.new_paths[(self.gpd.region_dict[r1], self.gpd.region_dict[r2])] = \
                                            copy.deepcopy(self.regionGraph.paths[(r1, r2)])

        ### generate 300 arrangement as samples for both solvers
        nPoses = len(self.instance.objects) + len(self.instance.buffers)
        allPoses = range(nPoses)
        self.allObjects = self.instance.objects + self.instance.buffers
        print("nPoses: " + str(nPoses))
        print("allPoses: " + str(allPoses))
        self.sample_arrangements = []
        for i in range(600):
            self.sample_arrangements.append(self.generateNewArrangement(allPoses))

        self.plan = None

        self.genSolutionFailure_biRRT = False
        start_time = clock()
        self.plan_biRRT = BiRRT_tester(
            self.initial_arrangement, self.final_arrangement, self.sample_arrangements, self.gpd, self.instance,
            self.new_paths, self.visualTool
        )
        self.comTime_biRRT = clock() - start_time
        print "Time to perform arrangement biRRT is: " + str(self.comTime_biRRT)
        if self.plan_biRRT.isConnected == False:
            self.genSolutionFailure_biRRT = True
        else:
            self.plan_biRRT.constructWholePath()
            self.plan_biRRT.getSolutionStats()
            self.totalActions_biRRT = self.plan_biRRT.totalActions
            self.plan = self.plan_biRRT

        self.genSolutionFailure_biRRTstar = False
        start_time = clock()
        self.plan_biRRTstar = BiRRTstar_tester(
            self.initial_arrangement, self.final_arrangement, self.sample_arrangements, self.gpd, self.instance,
            self.new_paths, self.visualTool
        )
        self.comTime_biRRTstar = clock() - start_time
        print "Time to perform arrangement biRRT* is: " + str(self.comTime_biRRTstar)
        if self.plan_biRRTstar.isConnected == False:
            self.genSolutionFailure_biRRTstar = True
        else:
            self.plan_biRRTstar.constructWholePath()
            self.plan_biRRTstar.getSolutionStats()
            self.totalActions_biRRTstar = self.plan_biRRTstar.totalActions
            self.plan = self.plan_biRRT

        if self.plan:
            if self.saveimage or self.display:
                self.visualTool.displayLocalPaths(self.plan, self.instance, self.final_arrangement)
            if self.display:
                self.visualTool.drawEntireAnimation(self.plan, self.instance, self.final_arrangement)

        return

    def linked_list_conversion(self, graph):
        # print "graph"
        # print graph
        region_dict = {}  # (1,2,'a'): 0
        LL = {}  # 0:[1,2,3]
        for key in graph:
            index = len(region_dict.keys())
            region_dict[key] = index
            LL[index] = []
        for key in graph:
            for v in graph[key]:
                LL[region_dict[key]].append(region_dict[v])
        # print "LL"
        # print self.LL
        # print "region dict"
        # print self.region_dict
        return region_dict, LL

    def generateNewArrangement(self, allPoses):
        isfree = False
        while (isfree != True):
            ### sample an arrangement (we currently allow duplicate node)
            new_arrangement = sample(allPoses, self.numObjs)
            ### check if it is a collision-free arrangement
            isfree = collisionCheck([self.allObjects[t] for t in new_arrangement])

        return new_arrangement

    def createDirForExp(self, data_path):
        ### This function creates the directory/folder to store the data for current experiment
        data_path = os.path.join(data_path, str(self.exp_id))
        try:
            os.mkdir(data_path)
        except OSError:
            print("Creation of the directory %s failed" % data_path)
        else:
            pass

        return data_path


if __name__ == "__main__":
    print("welcome to Experiment! Please call it from main.py\n")
