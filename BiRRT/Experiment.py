from __future__ import division

import os
import sys
import numpy as np
import time
import copy
from random import sample
from util import *

from InstanceGenerator import InstanceGenerator
from Visualizer import Visualizer
from RegionGraphGenerator import RegionGraphGenerator
from DensePathGenerator import DensePathGenerator
from BiRRTPlanner import BiRRTPlanner
from BiRRTStarPlanner import BiRRTStarPlanner
from BiRRT_tester import BiRRT_tester
from BiRRTstar_tester import BiRRTstar_tester


class Experiment(object):

    def __init__(self, numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, savestat, exp_id, data_path):
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
        self.visualTool.drawProblem(self.instance.objects, self.instance.points, self.instance.buffers)
        print("finishing generating the instance and the buffers")

        ### Now let's generate the region graph and build its connection
        self.regionGraph = RegionGraphGenerator(self.instance, self.visualTool, self.wall_mink)
        ### Now let's generate path options
        start_time = time.clock()
        self.gpd = DensePathGenerator(self.regionGraph.graph, self.regionGraph.obj2reg)
        print "total time for path generation: " + str(time.clock()-start_time)
        print "total path connections: " + str(len(self.gpd.dependency_dict))

        self.new_paths = {}
        for r1, r2 in self.regionGraph.paths.keys():
            self.new_paths[(self.gpd.region_dict[r1], self.gpd.region_dict[r2])] = \
                                            copy.deepcopy(self.regionGraph.paths[(r1, r2)])

        self.initial_arrangement = [i for i in range(0, 2*numObjs, 2)]
        self.final_arrangement = [i for i in range(1, 2*numObjs, 2)]
        print "initial_arrangement: " + str(self.initial_arrangement)
        print "final_arrangement: " + str(self.final_arrangement)

        ### generate 300 arrangement as samples for both solvers
        nPoses = len(self.instance.objects) + len(self.instance.buffers)
        allPoses = range(nPoses)
        self.allObjects = self.instance.objects + self.instance.buffers
        print("nPoses: " + str(nPoses))
        print("allPoses: " + str(allPoses))
        self.sample_arrangements = []
        for i in range(600):
            self.sample_arrangements.append(self.generateNewArrangement(allPoses))
        # print "300 arrangements: "
        # for sample in self.sample_arrangements:
        #     print sample
        
        # self.heuristic_sample_arrangements = []
        # self.heuristic_sample_arrangements.append(self.generateHeuristicNewArrangement(allPoses))


        
        self.genSolutionFailure = False
        start_time = time.clock()
        self.plan = BiRRT_tester(self.initial_arrangement, self.final_arrangement, self.sample_arrangements, \
                                    self.gpd, self.instance, self.new_paths, self.visualTool)
        self.comTime = time.clock()-start_time
        print "Time to perform arrangement biRRT is: " + str(self.comTime)
        if self.plan.isConnected == False: 
            self.genSolutionFailure = True
        else:
            self.plan.constructWholePath()
            self.plan.getSolutionStats()
            self.totalActions = self.plan.totalActions

        self.genSolutionFailure_star = False
        start_time = time.clock()
        self.plan_star = BiRRTstar_tester(self.initial_arrangement, self.final_arrangement, self.sample_arrangements, \
                                    self.gpd, self.instance, self.new_paths, self.visualTool)
        self.comTime_star = time.clock()-start_time
        print "Time to perform arrangement biRRT* is: " + str(self.comTime_star)
        if self.plan_star.isConnected == False: 
            self.genSolutionFailure_star = True
        else:
            self.plan_star.constructWholePath()
            self.plan_star.getSolutionStats()
            self.totalActions_star = self.plan_star.totalActions
        



        # if self.saveimage or self.display:
        #     self.visualTool.displayLocalPaths(self.plan, self.instance, self.final_arrangement)
        # if self.display:
        #     self.visualTool.drawEntireAnimation(self.plan, self.instance, self.final_arrangement)

        return


    def generateNewArrangement(self, allPoses):
        isfree = False
        while (isfree != True):
            ### sample an arrangement (we currently allow duplicate node)
            new_arrangement = sample(allPoses, self.numObjs)
            ### check if it is a collision-free arrangement
            isfree = collisionCheck([self.allObjects[t] for t in new_arrangement])

        return new_arrangement

    # def generateHeuristicNewArrangement(self, allPoses):
    #     for i range()


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










