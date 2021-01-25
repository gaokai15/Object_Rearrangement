from __future__ import division

import sys
import numpy as np
import time
import copy
import shutil
from random import sample
from util import *
import IPython
import os
import pickle
from collections import OrderedDict

from Visualizer import Visualizer
from InstanceGenerator import InstanceGenerator
from RegionGraphGenerator import RegionGraphGenerator
from DensePathGenerator import DensePathGenerator

from BiDirDPPlanner import BiDirDPPlanner
from UltimateHeuristicPlanner import UltimateHeuristicPlanner 
# from BiDir_fmRS_Planner import BiDir_fmRS_Planner


class Experiment_genAndTest(object):

    def __init__(self, numObjs, RAD, HEIGHT, WIDTH, \
                    display, displayMore, displayAnimation, savefile, saveimage, savestat, \
                    instance_dir):

        self.numObjs = numObjs
        self.RAD = RAD
        self.HEIGHT = HEIGHT
        self.WIDTH = WIDTH
        self.display = display
        self.displayMore = displayMore
        self.displayAnimation = displayAnimation
        self.savefile = savefile
        self.saveimage = saveimage
        self.savestat = savestat

        self.instance_dir = instance_dir

        ###################################### workspace setup ###################################################
        self.polygon = np.array(poly_disk([0, 0], self.RAD, 30)) ### object shape
        self.wall_pts = pn.Polygon([(0, 0), (self.WIDTH, 0), (self.WIDTH, self.HEIGHT), (0, self.HEIGHT)]) ### workspace
        ### wall_mink: inner boundary to limit the center of the object
        self.wall_mink = pn.Polygon([(self.RAD, self.RAD), (self.WIDTH - self.RAD, self.RAD), \
            (self.WIDTH - self.RAD, self.HEIGHT - self.RAD), (self.RAD, self.HEIGHT - self.RAD)])
        ###########################################################################################################


        ###################################### instance generation #################################################
        self.genInstanceFailure = False
        ### we need an instance generator (at most generate for 20 trials)
        ### the output is an instance which includes (points, objects)
        self.instance = InstanceGenerator(
            self.numObjs, self.HEIGHT, self.WIDTH, self.polygon) ### an InstanceGenerator object
        if (self.instance.points == False):
            self.genInstanceFailure = True
            print("failed to generating a valid instance at current experiment")
            return

        ### only continue to generate buffers when the instance is successfully generated
        ### the output now includes (points, objects, buffer_points, buffers)
        self.instance.genBuffers(self.HEIGHT, self.WIDTH, self.numObjs)
        print("finishing generating the instance and the buffers")

        ### index the arrangement
        self.initial_arrangement = [i for i in range(0, 2*numObjs, 2)]
        self.final_arrangement = [i for i in range(1, 2*numObjs, 2)]
        print "initial_arrangement: " + str(self.initial_arrangement)
        print "final_arrangement: " + str(self.final_arrangement)
        ##############################################################################################################


        ########## Now let's generate the region graph and build its connection #############
        self.regionGraph = RegionGraphGenerator(self.instance, self.wall_mink)
        # print(self.regionGraph.graph, self.regionGraph.obj2reg)
        ### The things we want to store are in gpd (LL, region_dict)
        self.gpd = DensePathGenerator(self.regionGraph.graph, self.regionGraph.obj2reg)
        self.new_paths = {}
        for r1, r2 in self.regionGraph.paths.keys():
            self.new_paths[(self.gpd.region_dict[r1], self.gpd.region_dict[r2])] = \
                                            copy.deepcopy(self.regionGraph.paths[(r1, r2)])
        ######################################################################################


        ### Let's visualize the problem
        self.visualTool = Visualizer(self.HEIGHT, self.WIDTH, self.numObjs, self.wall_pts, \
                                self.display, self.displayMore, self.saveimage, self.instance_dir)
        self.visualTool.drawProblem(self.instance.objects, self.instance.points, self.instance.buffers)

        ### region graph without connections
        self.visualTool.drawRegionGraph({}, self.regionGraph.regions.values(), label=False)
        ### connectivity graph
        self.visualTool.drawConGraph(
            self.regionGraph.paths, self.instance.points, self.instance.objects, self.instance.buffers)

        ### get the constraint sets for poses 
        ### key: pose_id
        ### value: a list of ids for poses it intersects
        self.constraint_set = self.getConstrSet(self.regionGraph.obj2reg)


        # ### method 1: DP
        # self.genSolutionFailure_DP = False
        # start_time = time.clock()
        # self.plan_DP = BiDirDPPlanner(
        #     self.initial_arrangement, self.final_arrangement, self.instance, self.gpd, \
        #     self.new_paths, self.polygon, self.RAD, self.visualTool)
        # self.comTime_DP = time.clock() - start_time
        # print "Time to perform BiDirectional search with DP solver: " + str(self.comTime_DP)
        # if self.plan_DP.isConnected == False:
        #     ### the solution is not found
        #     self.genSolutionFailure_DP = True
        # else:
        #     self.plan_DP.getSolutionStats()
        #     if self.savestat:
        #         self.writeStat(self.plan_DP.simplePath, self.plan_DP.totalActions, self.comTime_DP, "DP_solution.txt")
        #     self.plan_DP.constructWholePath()
        #     self.totalActions_DP = self.plan_DP.totalActions
        #     self.visualTool.drawSolutionPaths(self.plan_DP, self.instance, self.final_arrangement)

        # print("\n")

        # ### method 2: BiDirfmRS
        # self.genSolutionFailure_BiDirfmRS = False
        # start_time = time.clock()
        # self.plan_BiDirfmRS = BiDir_fmRS_Planner(
        #     self.initial_arrangement, self.final_arrangement, self.instance, self.gpd, \
        #     self.new_paths, self.polygon, self.RAD, self.visualTool)
        # self.comTime_BiDirfmRS = time.clock() - start_time
        # print "Time to perform search with Bidirectional fmRS solver: " + str(self.comTime_BiDirfmRS)
        # print("find solution? " + str(self.plan_BiDirfmRS.isConnected))
        # if self.plan_BiDirfmRS.isConnected == False:
        #     ### the solution is not found
        #     self.genSolutionFailure_BiDirfmRS = True
        # else:
        #     self.plan_BiDirfmRS.getSolutionStats()
        #     print("self.savestat: " + str(self.savestat))
        #     if self.savestat:
        #         self.writeStat(self.plan_BiDirfmRS.simplePath, self.plan_BiDirfmRS.totalActions, self.comTime_BiDirfmRS, "BiDirfmRS_solution.txt")
        #     # self.plan_BiDirfmRS.constructWholePath()
        #     self.totalActions_BiDirfmRS = self.plan_BiDirfmRS.totalActions
        #     # self.visualTool.drawSolutionPaths(self.plan_BiDirfmRS, self.instance, self.final_arrangement)

        ### method 4: Ultimate promising heuristic
        self.genSolutionFailure_UltimateHeuristic = False
        start_time = time.clock()
        self.plan_UltimateHeuristic = UltimateHeuristicPlanner(
            self.initial_arrangement, self.final_arrangement, self.instance, self.gpd, \
            self.new_paths, self.polygon, self.RAD, self.regionGraph, self.constraint_set, self.visualTool)
        self.comTime_UltimateHeuristic = time.clock() - start_time
        print "Time to perform search with Ultimateheuristic solver: " + str(self.comTime_UltimateHeuristic)
        print("find solution? " + str(self.plan_UltimateHeuristic.isConnected))
        if self.plan_UltimateHeuristic.isConnected == False:
            ### the solution is not found
            self.genSolutionFailure_UltimateHeuristic = True
        else:
            self.plan_UltimateHeuristic.getSolutionStats()
            if self.savestat:
                self.writeStat(self.plan_UltimateHeuristic.simplePath, self.plan_UltimateHeuristic.totalActions, self.comTime_UltimateHeuristic, "UltimateHeuristic_solution.txt")
            # self.copyBranchSolution(self.plan_UltimateHeuristic.simplePath)
            self.plan_UltimateHeuristic.constructWholePath()
            self.totalActions_UltimateHeuristic = self.plan_UltimateHeuristic.totalActions
            self.visualTool.drawSolutionPaths(self.plan_UltimateHeuristic, self.instance, self.final_arrangement)


        # if self.displayAnimation:
        #     self.visualTool.drawEntireAnimation1(self.plan_DP, self.instance, self.final_arrangement)

        if self.displayAnimation:
            self.visualTool.drawEntireAnimation1(self.plan_UltimateHeuristic, self.instance, self.final_arrangement)



    def getConstrSet(self, pose2reg):
        ### Input: pose2reg (key: pose_id, value: (pose_id, pose_id, str))
        ### Output: constraint set (key: pose_id, value: [pose_id, pose_id, ...])

        # print(pose2reg)
        # print("\n")
        
        constraint_set = OrderedDict()

        ### loop through all the poses
        for pose_id, constrs in pose2reg.items():
            constraint_set[pose_id] = []
            for constr in constrs:
                if type(constr) == int and constr >= 0 and constr != pose_id:
                    constraint_set[pose_id].append(constr)

        # for pose_id, constrs in constraint_set.items():
        #     print(str(pose_id) + ": " + str(constrs))

        return constraint_set


    def writeStat(self, simplePath, totalActions, comTime, stat_txt_file):
        stat_file = os.path.join(self.instance_dir, stat_txt_file)
        # print("writing stat into %s" % stat_file)
        f = open(stat_file, "w")
        f.write(str(totalActions) + "\n")
        f.write(str(comTime) + "\n")
        for waypoint in simplePath:
            f.write(waypoint + " ")
        f.close()
