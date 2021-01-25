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

from InstanceGenerator import InstanceGenerator
from Visualizer import Visualizer
from RegionGraphGenerator import RegionGraphGenerator
from DensePathGenerator import DensePathGenerator

from BruteForcePlanner import Non_Monotone_Solver_General


class Experiment_genInstances(object):

    def __init__(self, numObjs, RAD, HEIGHT, WIDTH, \
                    display, displayMore, displayAnimation, savefile, saveimage, savestat, \
                    instances_zeroBuffer_id, instances_oneBuffer_id, instances_twoBuffer_id, instances_unsolvable_id, \
                    instances_zeroBuffer_additionalActions, instances_oneBuffer_additionalActions, instances_twoBuffer_additionalActions, instances_unsolvable_additionalActions, \
                    instances_zeroBuffer_totalTime, instances_oneBuffer_totalTime, instances_twoBuffer_totalTime, instances_unsolvable_totalTime, \
                    exp_path):
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

        self.instances_zeroBuffer_id = instances_zeroBuffer_id
        self.instances_oneBuffer_id = instances_oneBuffer_id
        self.instances_twoBuffer_id = instances_twoBuffer_id
        self.instances_unsolvable_id = instances_unsolvable_id
        self.instances_zeroBuffer_additionalActions = instances_zeroBuffer_additionalActions
        self.instances_oneBuffer_additionalActions = instances_oneBuffer_additionalActions
        self.instances_twoBuffer_additionalActions = instances_twoBuffer_additionalActions
        self.instances_unsolvable_additionalActions = instances_unsolvable_additionalActions
        self.instances_zeroBuffer_totalTime = instances_zeroBuffer_totalTime
        self.instances_oneBuffer_totalTime = instances_oneBuffer_totalTime
        self.instances_twoBuffer_totalTime = instances_twoBuffer_totalTime
        self.instances_unsolvable_totalTime = instances_unsolvable_totalTime
        self.exp_path = exp_path
        

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
        ### The things we want to store are in gpd (LL, region_dict)
        self.gpd = DensePathGenerator(self.regionGraph.graph, self.regionGraph.obj2reg)
        self.new_paths = {}
        for r1, r2 in self.regionGraph.paths.keys():
            self.new_paths[(self.gpd.region_dict[r1], self.gpd.region_dict[r2])] = \
                                            copy.deepcopy(self.regionGraph.paths[(r1, r2)])
        ######################################################################################

        
        ### Now let's use brute force method to test the instance
        ### we only save instances which can be solved at most two buffers
        self.genSolutionFailure_BF = False
        start_poses = {}
        goal_poses = {}
        for i in range(numObjs):
            start_poses[i] = 2*i
            goal_poses[i] = 2*i + 1
        start_time = time.clock()
        self.plan_BF = Non_Monotone_Solver_General(
            self.regionGraph.graph, self.regionGraph.obj2reg, start_poses, goal_poses)
        self.comTime_BF = time.clock() - start_time


        if self.plan_BF.isConnected == False:
            self.genSolutionFailure_BF = True
            self.temp_nBuffers = 1000 ### 1000 indicates the problem is not solved
            self.instance_dir = self.createFolderAndUpdateStat()
            self.writeSolution_BF(self.plan_BF)
            # return
        else:
            ### create the subfolder with the right instance id in the right superfolder
            self.temp_nBuffers = self.plan_BF.totalActions - self.numObjs
            self.instance_dir = self.createFolderAndUpdateStat()
            self.writeSolution_BF(self.plan_BF)

        ### Now it's time to save the instances
        ### first save images (original problems, region graph and connectivity graph)
        ### initialize the visualTool
        self.visualTool = Visualizer(self.HEIGHT, self.WIDTH, self.numObjs, self.wall_pts, \
                                self.display, self.displayMore, self.saveimage, self.instance_dir)
        self.visualTool.drawProblem(self.instance.objects, self.instance.points, self.instance.buffers)
        ### region graph without connections
        self.visualTool.drawRegionGraph({}, self.regionGraph.regions.values(), label=False)
        ### connectivity graph
        self.visualTool.drawConGraph(
            self.regionGraph.paths, self.instance.points, self.instance.objects, self.instance.buffers)
        ### save parameters, instance and region graph
        arr_problem = ArrProblem(
            self.initial_arrangement, self.final_arrangement, self.instance, self.gpd, self.new_paths, self.polygon, self.regionGraph, self.visualTool)
        self.serializeInstance(arr_problem)



    def serializeInstance(self, arr_problem):
        file_arrProblem = open(self.instance_dir+"/arrProblem.obj", 'w')
        pickle.dump(arr_problem, file_arrProblem)


    def createFolderAndUpdateStat(self):
        if self.temp_nBuffers == 0:
            self.instances_zeroBuffer_id += 1
            self.instances_zeroBuffer_additionalActions += self.temp_nBuffers
            self.instances_zeroBuffer_totalTime += self.comTime_BF
            instance_dir = self.exp_path + "/zero_buffer/" + str(self.instances_zeroBuffer_id)
        elif self.temp_nBuffers == 1:
            self.instances_oneBuffer_id += 1
            self.instances_oneBuffer_additionalActions += self.temp_nBuffers
            self.instances_oneBuffer_totalTime += self.comTime_BF
            instance_dir = self.exp_path + "/one_buffer/" + str(self.instances_oneBuffer_id)
        elif self.temp_nBuffers == 2:
            self.instances_twoBuffer_id += 1
            self.instances_twoBuffer_additionalActions += self.temp_nBuffers
            self.instances_twoBuffer_totalTime += self.comTime_BF
            instance_dir = self.exp_path + "/two_buffer/" + str(self.instances_twoBuffer_id)
        else:
            ### either self.temp_nBuffers = 1000 or self.temp_nBuffers > 3
            self.instances_unsolvable_id += 1
            self.instances_unsolvable_additionalActions += self.temp_nBuffers
            self.instances_unsolvable_totalTime += self.comTime_BF
            instance_dir = self.exp_path + "/unsolvable/" + str(self.instances_unsolvable_id)

        os.mkdir(instance_dir)

        return instance_dir


    def writeSolution_BF(self, plan_BF):
        f_sol = open(self.instance_dir+"/optimal_solution.txt", "w")
        f_sol.write(str(self.temp_nBuffers) + "\n")
        f_sol.write(str(self.comTime_BF) + "\n")
        if plan_BF.isConnected == True:
            ### only save the following solution (object_ordering and [obj, buff])
            for obj_idx in plan_BF.object_ordering:
                f_sol.write(str(obj_idx) + " ")
            f_sol.write("\n")
            for obj, buff in plan_BF.obj_buffer_collections.items():
                f_sol.write(str(obj) + " " + str(buff) + " " + "\n")

        f_sol.close()



class ArrProblem(object):

    def __init__(self, initial_arrangement, final_arrangement, \
            instance, gpd, new_paths, polygon, regionGraph, visualTool):
        self.initial_arrangement = initial_arrangement
        self.final_arrangement = final_arrangement
        self.instance = instance
        self.gpd = gpd
        self.new_paths = new_paths
        self.polygon = polygon
        self.regionGraph = regionGraph
        self.visualTool = visualTool



