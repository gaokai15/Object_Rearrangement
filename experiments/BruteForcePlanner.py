from __future__ import division

from DPLocalSolver import DFS_Rec_for_Non_Monotone_General
from util import *
import os
import copy
import IPython
import time
import random
import numpy as np
from random import sample
from collections import OrderedDict
import Polygon as pn
from random import uniform, random
from itertools import combinations, product

# non monotone solver where the start/goal poses are not necessarily 2*i/2*i+1
class Non_Monotone_Solver_General(object):
    def __init__(self, graph, obj_locations, start_poses, goal_poses):
        self.obj_locations = obj_locations
        self.path_dict = {}
        self.dependency_dict = {}
        self.object_ordering = []
        self.start_poses = copy.deepcopy(start_poses)
        self.goal_poses = copy.deepcopy(goal_poses)
        self.n = len(self.start_poses)
        self.linked_list_conversion(graph)
        self.enumerate_cases()
        if self.isConnected:
            self.dependency_dict_conversion()
        
    def enumerate_cases(self):
        # enumerate possible cases
        time_allowed = 500
        start_time = time.clock()
        FOUND = False
        self.isConnected = False
        for obj_num in range(self.n+1): # num of objects that need buffers
            ### if it needs more than 2 buffers, let's give up this instances
            if obj_num >= 3:
                return
            print "number of objects that use buffers", obj_num
            for obj_set in combinations(self.start_poses.keys(), obj_num): # which objs need buffers
                if time.clock() - start_time > time_allowed:
                    ### time exceed, solution not found
                    return
                for buffer_set in product(sorted(self.obj_locations.keys(), reverse=True), repeat=obj_num): # which poses are buffers
                    if time.clock() - start_time > time_allowed:
                        ### time exceed, solution not found
                        return
                    obj_buffer_dict = {}
                    Degrade = False # when an object uses its own start or goal pose as a buffer, Degrade = True.
                    for index in xrange(len(obj_set)):
                        obj = obj_set[index]
                        buffer = buffer_set[index]
                        if (buffer == self.start_poses[obj]) or (buffer == self.goal_poses[obj]):
                            Degrade = True
                            break
                        obj_buffer_dict[obj] = (self.n+index, buffer)
                    if Degrade:
                        continue
                    # monotone solver input path_dict, dependency_dict, obj_locations, LL, region_dict, obj_buffer_dict
                    # DFS = DFS_for_Non_Monotone_General(self.start_poses, self.goal_poses, self.dependency_dict, self.path_dict, self.obj_locations, self.LL, self.region_dict, obj_buffer_dict)
                    DFS = DFS_Rec_for_Non_Monotone_General(self.start_poses, self.goal_poses, self.dependency_dict, self.path_dict, self.obj_locations, self.LL, self.region_dict, obj_buffer_dict)
                    self.dependency_dict = copy.deepcopy(DFS.dependency_dict)
                    self.path_dict = copy.deepcopy(DFS.path_dict)
                    if len(DFS.object_ordering)>0:
                        print "Find a solution!"
                        FOUND = True
                        self.isConnected = True
                        print "obj_buffer_dict", obj_buffer_dict
                        self.obj_buffer_collections = {}
                        for obj, buffer_event in obj_buffer_dict.items():
                            self.obj_buffer_collections[obj] = buffer_event[1] 
                        # print "DFS.object_ordering", DFS.object_ordering
                        self.object_ordering = DFS.object_ordering
                        print("object_ordering: " + str(self.object_ordering))
                        self.totalActions = len(self.object_ordering)
                        print("total actions: " + str(self.totalActions))
                        break
                if FOUND:
                    break
            if FOUND:
                break
            
        
    def linked_list_conversion(self, graph):
        # print "graph"
        # print graph
        self.region_dict = {}  # (1,2,'a'): 0
        self.LL = {}  # 0:[1,2,3]
        for key in graph:
            index = len(self.region_dict.keys())
            self.region_dict[key] = index
            self.LL[index] = []
        for key in graph:
            for v in graph[key]:
                self.LL[self.region_dict[key]].append(self.region_dict[v])
        # print "LL"
        # print self.LL
        # print "region dict"
        # print self.region_dict

    def dependency_dict_conversion(self):
        for key in self.dependency_dict.keys():
            number_set_list = self.dependency_dict[key]
            pose_set_list = []
            for number_set in number_set_list:
                pose_set = set()
                for number in number_set:
                    pose_set = pose_set.union({(number // 2, number % 2)})
                pose_set_list.append(pose_set)
            self.dependency_dict[key] = pose_set_list