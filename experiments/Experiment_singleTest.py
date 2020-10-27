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

from BiDirDPPlanner import BiDirDPPlanner
from UniDir_mRS_Planner import UniDir_mRS_Planner
from UniDir_fmRS_Planner import UniDir_fmRS_Planner
from UniDir_DPPlanner import UniDir_DPPlanner
from UniDir_leafNode_DPPlanner import UniDir_leafNode_DPPlanner
from UltimateHeuristicPlanner import UltimateHeuristicPlanner


class Experiment_singleTest(object):

    def __init__(self, numObjs, RAD, HEIGHT, WIDTH, \
                    display, displayMore, displayAnimation, savefile, saveimage, savestat, \
                    instance_dir):
        ### essential members
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

        ### load the problem
        self.deserialize() 
        self.initial_arrangement = self.arr_problem.initial_arrangement
        self.final_arrangement = self.arr_problem.final_arrangement
        self.instance = self.arr_problem.instance
        self.gpd = self.arr_problem.gpd
        self.new_paths = self.arr_problem.new_paths
        self.polygon = self.arr_problem.polygon
        self.regionGraph = self.arr_problem.regionGraph
        self.visualTool = self.arr_problem.visualTool

        ### for visualization purposes
        self.visualTool.updateDisplaySetting(self.display, self.displayMore, self.saveimage)
        # ### let's test for a second before solving the problem
        # self.visualTool.drawProblem(self.instance.objects, self.instance.points, self.instance.buffers)
        # self.visualTool.drawConGraph(
        #     self.regionGraph.paths, self.instance.points, self.instance.objects, self.instance.buffers)

        ### get the constraint sets for poses 
        ### key: pose_id
        ### value: a list of ids for poses it intersects
        self.constraint_set = self.getConstrSet(self.regionGraph.obj2reg)


        # ### method 0: UniDir mRS (only run on up to ten objects)
        # self.genSolutionFailure_UniDirmRS = False
        # self.solution_subfolder_UniDirmRS, self.tree_subfolder_UniDirmRS = self.createSolutionTreeFolder("UniDirmRS")
        # self.visualTool.addSolutionFolder(self.solution_subfolder_UniDirmRS, self.tree_subfolder_UniDirmRS)
        # start_time = time.clock()
        # self.plan_UniDirmRS = UniDir_mRS_Planner(
        #     self.initial_arrangement, self.final_arrangement, self.instance, self.gpd, \
        #     self.new_paths, self.polygon, self.RAD, self.visualTool)
        # self.comTime_UniDirmRS = time.clock() - start_time
        # print "Time to perform search with Uni-directional mRS solver: " + str(self.comTime_UniDirmRS)
        # if self.plan_UniDirmRS.isConnected == False:
        #     ### the solution is not found
        #     self.genSolutionFailure_UniDirmRS = True
        # else:
        #     self.plan_UniDirmRS.getSolutionStats()
        #     if self.savestat:
        #         self.writeStat(self.plan_UniDirmRS.simplePath, self.plan_UniDirmRS.totalActions, \
        #                                                 self.comTime_UniDirmRS, "UniDirmRS_solution.txt")
        #     # self.plan_UniDirmRS.constructWholePath()
        #     self.additionalActions_UniDirmRS = self.plan_UniDirmRS.totalActions - self.numObjs
        #     # self.visualTool.drawSolutionPaths(self.plan_UniDirmRS, self.instance, self.final_arrangement)

        # print("\n")

        # # method 1: UniDir fmRS
        # self.genSolutionFailure_UniDirfmRS = False
        # self.solution_subfolder_UniDirfmRS, self.tree_subfolder_UniDirfmRS = self.createSolutionTreeFolder("UniDirfmRS")
        # self.visualTool.addSolutionFolder(self.solution_subfolder_UniDirfmRS, self.tree_subfolder_UniDirfmRS)
        # start_time = time.clock()
        # self.plan_UniDirfmRS = UniDir_fmRS_Planner(
        #     self.initial_arrangement, self.final_arrangement, self.instance, self.gpd, \
        #     self.new_paths, self.polygon, self.RAD, self.visualTool)
        # self.comTime_UniDirfmRS = time.clock() - start_time
        # print "Time to perform search with Uni-directional fmRS solver: " + str(self.comTime_UniDirfmRS)
        # if self.plan_UniDirfmRS.isConnected == False:
        #     ### the solution is not found
        #     self.genSolutionFailure_UniDirfmRS = True
        # else:
        #     self.plan_UniDirfmRS.getSolutionStats()
        #     if self.savestat:
        #         self.writeStat(self.plan_UniDirfmRS.simplePath, self.plan_UniDirfmRS.totalActions, \
        #                                                 self.comTime_UniDirfmRS, "UniDirfmRS_solution.txt")
        #     # self.plan_UniDirfmRS.constructWholePath()
        #     self.additionalActions_UniDirfmRS = self.plan_UniDirfmRS.totalActions - self.numObjs
        #     # self.visualTool.drawSolutionPaths(self.plan_UniDirfmRS, self.instance, self.final_arrangement)

        # print("\n")

        # ### method 2: unidirectional DP
        # self.genSolutionFailure_uniDirDP = False
        # self.solution_subfolder_uniDirDP, self.tree_subfolder_uniDirDP = self.createSolutionTreeFolder("uniDirDP")
        # self.visualTool.addSolutionFolder(self.solution_subfolder_uniDirDP, self.tree_subfolder_uniDirDP)
        # start_time = time.clock()
        # self.plan_uniDirDP = UniDir_DPPlanner(
        #     self.initial_arrangement, self.final_arrangement, self.instance, self.gpd, \
        #     self.new_paths, self.polygon, self.RAD, self.visualTool)
        # self.comTime_uniDirDP = time.clock() - start_time
        # print "Time to perform unidirectional search with DP solver: " + str(self.comTime_uniDirDP)
        # if self.plan_uniDirDP.isConnected == False:
        #     ### the solution is not found
        #     self.genSolutionFailure_uniDirDP = True
        # else:
        #     self.plan_uniDirDP.getSolutionStats()
        #     if self.savestat:
        #         self.writeStat(self.plan_uniDirDP.simplePath, self.plan_uniDirDP.totalActions, \
        #                                                 self.comTime_uniDirDP, "uniDirDP_solution.txt")
        #     self.plan_uniDirDP.constructWholePath()
        #     self.additionalActions_uniDirDP = self.plan_uniDirDP.totalActions - self.numObjs
        #     self.visualTool.drawSolutionPaths(self.plan_uniDirDP, self.instance, self.final_arrangement)

        # print("\n")

        ### method 3: unidirectional leafNode DP
        self.genSolutionFailure_uniDirLN_DP = False
        self.solution_subfolder_uniDirLN_DP, self.tree_subfolder_uniDirLN_DP = self.createSolutionTreeFolder("uniDirLN_DP")
        self.visualTool.addSolutionFolder(self.solution_subfolder_uniDirLN_DP, self.tree_subfolder_uniDirLN_DP)
        start_time = time.clock()
        self.plan_uniDirLN_DP = UniDir_leafNode_DPPlanner(
            self.initial_arrangement, self.final_arrangement, self.instance, self.gpd, \
            self.new_paths, self.polygon, self.RAD, self.visualTool)
        self.comTime_uniDirLN_DP = time.clock() - start_time
        print "Time to perform unidirectional leafNode search with DP solver: " + str(self.comTime_uniDirLN_DP)
        if self.plan_uniDirLN_DP.isConnected == False:
            ### the solution is not found
            self.genSolutionFailure_uniDirLN_DP = True
        else:
            self.plan_uniDirLN_DP.getSolutionStats()
            if self.savestat:
                self.writeStat(self.plan_uniDirLN_DP.simplePath, self.plan_uniDirLN_DP.totalActions, \
                                                        self.comTime_uniDirLN_DP, "uniDirLN_DP_solution.txt")
            self.plan_uniDirLN_DP.constructWholePath()
            self.additionalActions_uniDirLN_DP = self.plan_uniDirLN_DP.totalActions - self.numObjs
            self.visualTool.drawSolutionPaths(self.plan_uniDirLN_DP, self.instance, self.final_arrangement)

        print("\n")

        # ### method 4: Ultimate heuristic
        # self.genSolutionFailure_UltimateHeuristic = False
        # self.solution_subfolder_UltimateHeuristic, self.tree_subfolder_UltimateHeuristic = self.createSolutionTreeFolder("UltimateHeuristic")
        # self.visualTool.addSolutionFolder(self.solution_subfolder_UltimateHeuristic, self.tree_subfolder_UltimateHeuristic)
        # start_time = time.clock()
        # self.plan_UltimateHeuristic = UltimateHeuristicPlanner(
        #     self.initial_arrangement, self.final_arrangement, self.instance, self.gpd, \
        #     self.new_paths, self.polygon, self.RAD, self.constraint_set, self.visualTool)
        # self.comTime_UltimateHeuristic = time.clock() - start_time
        # print "Time to perform search with Ultimateheuristic solver: " + str(self.comTime_UltimateHeuristic)
        # if self.plan_UltimateHeuristic.isConnected == False:
        #     ### the solution is not found
        #     self.genSolutionFailure_UltimateHeuristic = True
        # else:
        #     self.plan_UltimateHeuristic.getSolutionStats()
        #     if self.savestat:
        #         self.writeStat(self.plan_UltimateHeuristic.simplePath, self.plan_UltimateHeuristic.totalActions, \
        #                                             self.comTime_UltimateHeuristic, "UltimateHeuristic_solution.txt")
        #     self.plan_UltimateHeuristic.constructWholePath()
        #     self.additionalActions_UltimateHeuristic = self.plan_UltimateHeuristic.totalActions - self.numObjs
        #     self.visualTool.drawSolutionPaths(self.plan_UltimateHeuristic, self.instance, self.final_arrangement)


        # if self.saveimage or self.display:
        #     self.visualTool.drawSolutionPaths(self.plan_DP, self.instance, self.final_arrangement)
        #     self.visualTool.drawSolutionPaths(self.plan_anytimeDP, self.instance, self.final_arrangement)
        # if self.displayAnimation:
        #     self.visualTool.drawEntireAnimation1(self.plan_DP, self.instance, self.final_arrangement)


        # if self.saveimage or self.display:
        #     self.visualTool.drawSolutionPaths(self.plan_Fast_heuristic, self.instance, self.final_arrangement)
        # if self.displayAnimation:
        #     self.visualTool.drawEntireAnimation1(self.plan_Fast_heuristic, self.instance, self.final_arrangement)

        return



    def deserialize(self):
        file_arrProblem = open(self.instance_dir+"/arrProblem.obj", 'r')
        self.arr_problem = pickle.load(file_arrProblem)


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

    def createSolutionTreeFolder(self, folderName):
        solution_subfolder = os.path.join(self.instance_dir, "solution_"+folderName)
        tree_subfolder = os.path.join(self.instance_dir, "tree_"+folderName)
        if self.saveimage:
            if os.path.exists(solution_subfolder):
                shutil.rmtree(solution_subfolder)
            os.mkdir(solution_subfolder)
        if self.saveimage:
            if os.path.exists(tree_subfolder):
                shutil.rmtree(tree_subfolder)
            os.mkdir(tree_subfolder)

        return solution_subfolder, tree_subfolder

    def writeStat(self, simplePath, totalActions, comTime, stat_txt_file):
        stat_file = os.path.join(self.instance_dir, stat_txt_file)
        # print("writing stat into %s" % stat_file)
        f = open(stat_file, "w")
        f.write(str(totalActions - self.numObjs) + "\n")
        f.write(str(comTime) + "\n")
        for waypoint in simplePath:
            f.write(waypoint + " ")
        f.close()

    ### the function right below is not needed
    # def copyBranchSolution(self, simplePath):
    #     for i in range(len(simplePath)-1):
    #         left_id = simplePath[i]
    #         right_id = simplePath[i+1]
    #         target_file = os.path.join(self.tree_subfolder, left_id+"--"+right_id+".png")
    #         shutil.copy(target_file, self.solution_subfolder)


class ArrProblem(object):

    def __init__(initial_arrangement, final_arrangement, \
            instance, gpd, new_paths, polygon):
        self.initial_arrangement = initial_arrangement
        self.final_arrangement = final_arrangement
        self.instance = instance
        self.gpd = gpd
        self.new_paths = new_paths
        self.polygon = polygon