from __future__ import division

import os
import sys
import numpy as np
import time
import copy
import shutil
from random import sample
from util import *
import IPython

from InstanceGenerator import InstanceGenerator
from Visualizer import Visualizer
from RegionGraphGenerator import RegionGraphGenerator
from DensePathGenerator import DensePathGenerator
from BiRRTPlanner import BiRRTPlanner
from BiRRTStarPlanner import BiRRTStarPlanner
from BiDirDPPlanner import BiDirDPPlanner
from HeuristicDPPlanner import HeuristicDPPlanner
from FastHeuristicDPPlanner import FastHeuristicDPPlanner
from BruteForcePlanner import Non_Monotone_Solver_General


class Experiment(object):

    def __init__(self, numObjs, RAD, HEIGHT, WIDTH, display, displayMore, displayAnimation, savefile, saveimage, savestat, exp_id, data_path):
        ### essential member
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
        self.exp_id = exp_id
        self.data_path = self.createDirForExp(data_path)
        self.solution_subfolder, self.tree_subfolder = self.createSubFolder()

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
        self.instance = InstanceGenerator(
            self.numObjs, self.HEIGHT, self.WIDTH, self.polygon) ### an InstanceGenerator object 
        if (self.instance.points == False):
            self.genInstanceFailure = True
            print("failed to generating a valid instance at exp " + str(exp_id))
            return

        ### only continue when the instance is successfully generated
        ### an Visualizer object
        self.visualTool = Visualizer(self.HEIGHT, self.WIDTH, self.numObjs, self.wall_pts, \
                                            self.display, self.displayMore, self.saveimage, \
                                            self.data_path, self.solution_subfolder, self.tree_subfolder)
        self.instance.genBuffers(self.HEIGHT, self.WIDTH, self.numObjs)
        self.visualTool.drawProblem(self.instance.objects, self.instance.points, self.instance.buffers)
        print("finishing generating the instance and the buffers")
        ### index the arrangement
        self.initial_arrangement = [i for i in range(0, 2*numObjs, 2)]
        self.final_arrangement = [i for i in range(1, 2*numObjs, 2)]
        print "initial_arrangement: " + str(self.initial_arrangement)
        print "final_arrangement: " + str(self.final_arrangement)
        ############################################################################################################

        
        ########## Now let's generate the region graph and build its connection #############
        self.regionGraph = RegionGraphGenerator(self.instance, self.visualTool, self.wall_mink)
        self.gpd = DensePathGenerator(self.regionGraph.graph, self.regionGraph.obj2reg)
        self.new_paths = {}
        for r1, r2 in self.regionGraph.paths.keys():
            self.new_paths[(self.gpd.region_dict[r1], self.gpd.region_dict[r2])] = \
                                            copy.deepcopy(self.regionGraph.paths[(r1, r2)])
        ######################################################################################

        # ## method 0: bruce force
        # self.genSolutionFailure_BF = False
        # start_poses = {}
        # goal_poses = {}
        # for i in range(numObjs):
        #     start_poses[i] = 2*i
        #     goal_poses[i] = 2*i + 1        
        # start_time = time.clock()
        # self.plan_BF = Non_Monotone_Solver_General(
        #     self.regionGraph.graph, self.regionGraph.obj2reg, start_poses, goal_poses)
        # self.comTime_BF = time.clock() - start_time
        # print("BF time: " + str(self.comTime_BF))
        # if self.plan_BF.isConnected == False:
        #     ### the solution is not found
        #     self.genSolutionFailure_BF = True
        # else:
        #     self.totalActions_BF = len(self.plan_BF.object_ordering)
        #     print("the ordering: " + str(self.plan_BF.object_ordering))
        #     print("total action: " + str(self.totalActions_BF))


        ## method 1: DP 
        self.genSolutionFailure_DP = False
        start_time = time.clock()
        self.plan_DP = BiDirDPPlanner(
            self.initial_arrangement, self.final_arrangement, self.instance, self.gpd, self.new_paths, self.polygon, self.visualTool)
        self.comTime_DP = time.clock() - start_time
        print "Time to perform BiDirectional search with DP solver: " + str(self.comTime_DP)
        if self.plan_DP.isConnected == False:
            ### the solution is not found
            self.genSolutionFailure_DP = True
        else:
            self.plan_DP.getSolutionStats()
            # self.writeStat(self.plan_DP.simplePath, self.plan_DP.totalActions, self.comTime_DP)
            # self.copyBranchSolution(self.plan_DP.simplePath)
            self.plan_DP.constructWholePath()
            self.totalActions_DP = self.plan_DP.totalActions


        # # method 2: DP with heuristic
        # self.genSolutionFailure_DP_heuristic = False
        # start_time = time.clock()
        # self.plan_DP_heuristic = HeuristicDPPlanner(
        #     self.initial_arrangement, self.final_arrangement, self.instance, self.gpd, self.new_paths, \
        #     self.polygon, self.visualTool)
        # self.comTime_DP_heuristic = time.clock() - start_time
        # print "Time to perform search with DP_heuristic solver: " + str(self.comTime_DP_heuristic)
        # if self.plan_DP_heuristic.isConnected == False:
        #     ### the solution is not found
        #     self.genSolutionFailure_DP_heuristic = True
        # else:
        #     self.plan_DP_heuristic.getSolutionStats()
        #     # self.writeStat(self.plan_DP.simplePath, self.plan_DP.totalActions, self.comTime_DP)
        #     # self.copyBranchSolution(self.plan_DP.simplePath)
        #     self.plan_DP_heuristic.constructWholePath()
        #     self.totalActions_DP_heuristic = self.plan_DP_heuristic.totalActions


        # method 3: DP with fast heuristic
        self.genSolutionFailure_Fast_heuristic = False
        start_time = time.clock()
        self.plan_Fast_heuristic = FastHeuristicDPPlanner(
            self.initial_arrangement, self.final_arrangement, self.instance, self.gpd, self.new_paths, \
            self.polygon, self.visualTool)
        self.comTime_Fast_heuristic = time.clock() - start_time
        print "Time to perform search with DP_Fast_heuristic solver: " + str(self.comTime_Fast_heuristic)
        if self.plan_Fast_heuristic.isConnected == False:
            ### the solution is not found
            self.genSolutionFailure_Fast_heuristic = True
        else:
            self.plan_Fast_heuristic.getSolutionStats()
            # self.writeStat(self.plan_DP.simplePath, self.plan_DP.totalActions, self.comTime_DP)
            # self.copyBranchSolution(self.plan_DP.simplePath)
            self.plan_Fast_heuristic.constructWholePath()
            self.totalActions_Fast_heuristic = self.plan_Fast_heuristic.totalActions


        ### method 3: bidirectional RRT
        # self.genSolutionFailure_biRRT = False
        # start_time = time.clock()
        # self.plan_biRRT = BiRRTPlanner(
        #     self.initial_arrangement, self.final_arrangement, self.instance, self.gpd, self.new_paths, self.visualTool)
        # self.comTime_biRRT = time.clock()-start_time
        # print "Time to perform arrangement biRRT is: " + str(self.comTime_biRRT)
        # if self.plan_biRRT.isConnected == False: 
        #     self.genSolutionFailure_biRRT = True
        # else:
        #     self.plan_biRRT.constructWholePath()
        #     self.plan_biRRT.getSolutionStats()
        #     self.totalActions_biRRT = self.plan_biRRT.totalActions


        ### method 4: bidirectional RRT*
        # self.genSolutionFailure_biRRTstar = False
        # start_time = time.clock()
        # self.plan_biRRTstar = BiRRTStarPlanner(
        #     self.initial_arrangement, self.final_arrangement, self.instance, self.gpd, self.new_paths, self.visualTool)
        # self.comTime_biRRTstar = time.clock()-start_time
        # print "Time to perform arrangement biRRT* is: " + str(self.comTime_biRRTstar)
        # if self.plan_biRRTstar.isConnected == False: 
        #     self.genSolutionFailure_biRRTstar = True
        # else:
        #     self.plan_biRRTstar.constructWholePath()
        #     self.plan_biRRTstar.getSolutionStats()
        #     self.totalActions_biRRTstar = self.plan_biRRTstar.totalActions
        

        # if self.saveimage or self.display:
        #     self.visualTool.drawSolutionPaths(self.plan_DP, self.instance, self.final_arrangement)
        # if self.displayAnimation:
        #     self.visualTool.drawEntireAnimation1(self.plan_DP, self.instance, self.final_arrangement)


        # if self.saveimage or self.display:
        #     self.visualTool.drawSolutionPaths(self.plan_DP_heuristic, self.instance, self.final_arrangement)
        # if self.displayAnimation:
        #     self.visualTool.drawEntireAnimation1(self.plan_DP_heuristic, self.instance, self.final_arrangement)


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
            print("successfully create %s" % data_path)
        except OSError:
            print("Creation of the directory %s failed" % data_path)
        else:
            pass

        return data_path


    def createSubFolder(self):
        
        solution_subfolder = os.path.join(self.data_path, "solution")
        tree_subfolder = os.path.join(self.data_path, "tree")

        # time.sleep(5)

        # try:
        #     os.mkdir(solution_subfolder)
        #     print("successfully create %s" % solution_subfolder)
        # except OSError:
        #     print("Creation of the directory %s failed" % solution_subfolder)
        # else:
        #     pass

        os.mkdir(solution_subfolder)

        # try:
        #     os.mkdir(tree_subfolder)
        #     print("successfully create %s" % tree_subfolder)
        # except OSError:
        #     print("Creation of the directory %s failed" % tree_subfolder)
        # else:
        #     pass

        os.mkdir(tree_subfolder)
        
        return solution_subfolder, tree_subfolder


    def writeStat(self, simplePath, totalActions, comTime):
        stat_file = os.path.join(self.data_path, "stat.txt")
        # print("writing stat into %s" % stat_file)
        f = open(stat_file, "w")
        for waypoint in simplePath:
            f.write(waypoint + " ")
        f.write("\n")
        f.write(str(totalActions) + "\n")
        f.write(str(comTime) + "\n")
        f.close()


    def copyBranchSolution(self, simplePath):
        for i in range(len(simplePath)-1):
            left_id = simplePath[i]
            right_id = simplePath[i+1]
            target_file = os.path.join(self.tree_subfolder, left_id+"--"+right_id+".png")
            shutil.copy(target_file, self.solution_subfolder)


if __name__ == "__main__":
    print("welcome to Experiment! Please call it from main.py\n")










