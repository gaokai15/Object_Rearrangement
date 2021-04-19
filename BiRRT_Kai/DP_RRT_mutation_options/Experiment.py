from __future__ import division

import os
import sys
import numpy as np
import time
import copy
from random import sample
from util import *
import cPickle as pickle
import resource

from InstanceGenerator import InstanceGenerator
from Visualizer import Visualizer
from RegionGraphGenerator import RegionGraphGenerator
from DensePathGenerator import DensePathGenerator
from BiDirDPPlanner1 import BiDirDPPlanner_Leaf_Root
from DPBruteForce import Non_Monotone_Solver_General, Generalized_Brute_force
from FVS import feekback_vertex_ILP
from FastHeuristicDPPlanner import FastHeuristicDPPlanner
from BiDir_fmRS_Planner import BiDir_fmRS_Planner
from BiDir_mRS_Planner import BiDir_mRS_Planner
from partial_fmRS import partial_fmRS

# timeout version function
from Timeout_Functions import timeout_Fast_heuristic, \
    timeout_BiDirDPPlanner_Leaf_Root, \
    timeout_DensePathGenerator, \
    timeout_Non_Monotone_Solver_General, \
    timeout_Generalized_Brute_force, \
    timeout_BiDir_fmRS_Planner, \
    timeout_BiDir_mRS_Planner

my_path = os.path.abspath(os.path.dirname(__file__))

def set_max_memory(MAX):
    soft, hard = resource.getrlimit(resource.RLIMIT_AS)
    resource.setrlimit(resource.RLIMIT_AS, (MAX, hard))

# Disable
def blockPrint():
    sys.stdout = open(os.devnull, 'w')

# Restore
def enablePrint():
    sys.stdout = sys.__stdout__

class Experiment(object):
    def __init__(self, numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, savestat, exp_id, data_path):
        blockPrint()
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

        self.polygon = np.array(poly_disk([0, 0], self.RAD, 30)) ### object shape
        self.wall_pts = pn.Polygon([(0, 0), (self.WIDTH, 0), (self.WIDTH, self.HEIGHT), (0, self.HEIGHT)]) ### workspace
        ### wall_mink: inner boundary to limit the center of the object
        self.wall_mink = pn.Polygon([(self.RAD, self.RAD), (self.WIDTH - self.RAD, self.RAD), \
            (self.WIDTH - self.RAD, self.HEIGHT - self.RAD), (self.RAD, self.HEIGHT - self.RAD)])

        self.genInstanceFailure = False
        ### we need an instance generator (at most generate for 20 trials)
        self.instance = InstanceGenerator(
            self.numObjs, self.HEIGHT, self.WIDTH, self.polygon) ### an InstanceGenerator object 
        if (self.instance.points == False):
            self.genInstanceFailure = True
            print("failed to generating a valid instance at exp " + str(exp_id))
            return

        ### only continue when the instance is successfully generated
        ### an Visualizer object
        self.visualTool = Visualizer(self.HEIGHT, self.WIDTH, self.numObjs, self.wall_pts, \
                                            self.display, self.displayMore, self.saveimage, self.data_path)
        self.instance.genBuffers(self.HEIGHT, self.WIDTH, self.numObjs)
        # self.visualTool.drawProblem(self.instance.objects, self.instance.points, self.instance.buffers)
        print("finishing generating the instance and the buffers")
        ### index the arrangement
        self.initial_arrangement = [i for i in range(0, 2*numObjs, 2)]
        self.final_arrangement = [i for i in range(1, 2*numObjs, 2)]
        print "initial_arrangement: " + str(self.initial_arrangement)
        print "final_arrangement: " + str(self.final_arrangement)

        ### Now let's generate the region graph and build its connection
        self.regionGraph = RegionGraphGenerator(self.instance, self.visualTool, self.wall_mink)
        with open(os.path.join(my_path, "instance00.pkl"),
                    'wb') as output:
            pickle.dump((self.regionGraph, self.instance), output, pickle.HIGHEST_PROTOCOL)

        # with open(os.path.join(my_path, "instance00.pkl"), 'rb') as input:
        #     (self.regionGraph, self.instance) = pickle.load(input)

        # method: fmRS
        self.genSolutionFailure_fmRS = False
        start_time = time.clock()
        try:
            self.gpd = DensePathGenerator(self.regionGraph.graph, self.regionGraph.obj2reg)
            self.new_paths = {}
            for r1, r2 in self.regionGraph.paths.keys():
                self.new_paths[(self.gpd.region_dict[r1], self.gpd.region_dict[r2])] = \
                                                copy.deepcopy(self.regionGraph.paths[(r1, r2)])
            self.plan_fmRS = timeout_BiDir_fmRS_Planner(
                self.initial_arrangement, self.final_arrangement, self.instance, self.gpd, self.new_paths, \
                self.polygon, self.RAD, self.visualTool)
            if self.plan_fmRS.isConnected == False:
                ### the solution is not found
                self.genSolutionFailure_fmRS = True
            else:
                self.totalActions_fmRS = self.plan_fmRS.totalActions
        except Warning:
            self.genSolutionFailure_fmRS = True
        self.comTime_fmRS = time.clock() - start_time
        print "Time to perform BiDirectional search with fmRS local solver: " + str(self.comTime_fmRS)

        # method: mRS
        self.genSolutionFailure_mRS = False
        start_time = time.clock()
        try:
            self.gpd = DensePathGenerator(self.regionGraph.graph, self.regionGraph.obj2reg)
            self.new_paths = {}
            for r1, r2 in self.regionGraph.paths.keys():
                self.new_paths[(self.gpd.region_dict[r1], self.gpd.region_dict[r2])] = \
                                                copy.deepcopy(self.regionGraph.paths[(r1, r2)])
            self.plan_mRS = timeout_BiDir_mRS_Planner(
                self.initial_arrangement, self.final_arrangement, self.instance, self.gpd, self.new_paths, \
                self.polygon, self.RAD, self.visualTool)
            if self.plan_mRS.isConnected == False:
                ### the solution is not found
                self.genSolutionFailure_mRS = True
            else:
                self.totalActions_mRS = self.plan_mRS.totalActions
        except Warning:
            self.genSolutionFailure_mRS = True
        self.comTime_mRS = time.clock() - start_time
        print "Time to perform BiDirectional search with mRS local solver: " + str(self.comTime_mRS)

        # method 3: DP with fast heuristic
        self.genSolutionFailure_Fast_heuristic = False
        start_time = time.clock()
        try:
            self.gpd = DensePathGenerator(self.regionGraph.graph, self.regionGraph.obj2reg)
            self.new_paths = {}
            for r1, r2 in self.regionGraph.paths.keys():
                self.new_paths[(self.gpd.region_dict[r1], self.gpd.region_dict[r2])] = \
                                                copy.deepcopy(self.regionGraph.paths[(r1, r2)])
            self.plan_Fast_heuristic = timeout_Fast_heuristic(
                self.initial_arrangement, self.final_arrangement, self.instance, self.gpd, self.new_paths, \
                self.polygon, self.visualTool)
            if self.plan_Fast_heuristic.isConnected == False:
                ### the solution is not found
                self.genSolutionFailure_Fast_heuristic = True
            else:
                self.plan_Fast_heuristic.getSolutionStats()
                # self.writeStat(self.plan_DP.simplePath, self.plan_DP.totalActions, self.comTime_DP)
                # self.copyBranchSolution(self.plan_DP.simplePath)
                self.plan_Fast_heuristic.constructWholePath()
                self.totalActions_Fast_heuristic = self.plan_Fast_heuristic.totalActions
        except Exception:
            self.genSolutionFailure_Fast_heuristic = True
        self.comTime_Fast_heuristic = time.clock() - start_time
        print "Time to perform search with DP_Fast_heuristic solver: " + str(self.comTime_Fast_heuristic)
        
        #########################################################
        self.genSolutionFailure_DP_local_leaf_root = False
        start_time = time.clock()
        try:
            self.plan_DP_local_leaf_root = timeout_BiDirDPPlanner_Leaf_Root(
                self.initial_arrangement, self.final_arrangement, self.instance,  
                self.regionGraph.obj2reg, region_dict, linked_list, 
                self.visualTool)
            if self.plan_DP_local_leaf_root.isConnected == False:
                ### the solution is not found
                self.genSolutionFailure_DP_local_leaf_root = True
            else:
                self.totalActions_DP_local_leaf_root = self.plan_DP_local_leaf_root.totalActions
            enablePrint()
            # print "# nodes: ", len(self.plan_DP_local_leaf_root.treeL)+len(self.plan_DP_local_leaf_root.treeR)
            blockPrint()
        except Exception:
            self.genSolutionFailure_DP_local_leaf_root = True
        self.comTime_DP_local_leaf_root = time.clock() - start_time
        print "Time to perform BiDirectional search with DP local_leaf_root solver: " + str(self.comTime_DP_local_leaf_root)


        ##################################################
        self.genSolutionFailure_DP_BruteForce = False
        start_time = time.clock()
        try:
            start_poses = {}
            goal_poses = {}
            for i in range(len(self.initial_arrangement)):
                start_poses[i] = self.initial_arrangement[i]
                goal_poses[i] = self.final_arrangement[i]
            self.plan_DP_BruteForce = timeout_Non_Monotone_Solver_General(
                self.regionGraph.graph, self.regionGraph.obj2reg, start_poses, goal_poses)
            if self.plan_DP_BruteForce.totalActions == 0:
                ### the solution is not found
                self.genSolutionFailure_DP_BruteForce = True
            else:
                self.totalActions_DP_BruteForce = self.plan_DP_BruteForce.totalActions
        except Exception:
            ### the solution is not found
            self.genSolutionFailure_DP_BruteForce = True
        self.comTime_DP_BruteForce = time.clock() - start_time
        print "Time to perform BiDirectional search with DP local solver: " + str(self.comTime_DP_BruteForce)



        ##################################################
        self.genSolutionFailure_Generalized_BruteForce = False
        start_time = time.clock()
        try:
            print Intentional_Error
            start_poses = {}
            goal_poses = {}
            for i in range(len(self.initial_arrangement)):
                start_poses[i] = self.initial_arrangement[i]
                goal_poses[i] = self.final_arrangement[i]
            self.plan_Generalized_BruteForce = timeout_Generalized_Brute_force(
                self.regionGraph.graph, self.regionGraph.obj2reg, start_poses, goal_poses)
            if self.plan_Generalized_BruteForce.totalActions == 0:
                ### the solution is not found
                self.genSolutionFailure_Generalized_BruteForce = True
            else:
                self.totalActions_Generalized_BruteForce = self.plan_Generalized_BruteForce.totalActions
        except Exception:
            ### the solution is not found
            self.genSolutionFailure_Generalized_BruteForce = True
        self.comTime_Generalized_BruteForce = time.clock() - start_time
        print "Time to perform Generalized Brute Force solver: " + str(self.comTime_Generalized_BruteForce)




        # ### Now let's generate the region graph and build its connection
        # self.regionGraph = RegionGraphGenerator(self.instance, self.visualTool, self.wall_mink)
        ### Now let's generate path options
        self.genSolutionFailure_GPD = False
        start_time = time.clock()
        try:
            print Intentional_Error
            self.gpd = timeout_DensePathGenerator(self.regionGraph.graph, self.regionGraph.obj2reg)
        except Exception:
            self.genSolutionFailure_GPD = True
            self.gpd = None
            self.totalActions_FVS = -1
            self.genSolutionFailure_biRRT = True
            self.genSolutionFailure_biRRTstar = True
            self.comTime_PathOptions = time.clock()-start_time
            return
        self.comTime_PathOptions = time.clock()-start_time
        print "total time for path generation: " + str(self.comTime_PathOptions)
        # print "total path connections: " + str(len(self.gpd.dependency_dict))

        # IP = feekback_vertex_ILP(self.gpd.dependency_dict, numObjs)
        # self.totalActions_FVS = numObjs + IP.optimum

        self.new_paths = {}
        for r1, r2 in self.regionGraph.paths.keys():
            self.new_paths[(self.gpd.region_dict[r1], self.gpd.region_dict[r2])] = \
                                            copy.deepcopy(self.regionGraph.paths[(r1, r2)])

        ### generate 300 arrangement as samples for both solvers
        start_time = time.clock()
        nPoses = len(self.instance.objects) + len(self.instance.buffers)
        allPoses = range(nPoses)
        self.allObjects = self.instance.objects + self.instance.buffers
        print("nPoses: " + str(nPoses))
        print("allPoses: " + str(allPoses))
        # self.sample_arrangements = []
        # for i in range(600):
        #     self.sample_arrangements.append(self.generateNewArrangement(allPoses))
        self.comTime_SampleArrangements = time.clock()-start_time


        


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










