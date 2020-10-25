import os
import sys
import copy
from collections import OrderedDict
# from cgraph import genDenseCGraph, drawMotions, animatedMotions, drawProblem
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import gurobipy as gp
from gurobipy import GRB
import math
import matplotlib
# matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import cPickle as pickle

import IPython

import time
from itertools import combinations, product
import resource

my_path = os.path.abspath(os.path.dirname(__file__))

def set_max_memory(MAX):
    soft, hard = resource.getrlimit(resource.RLIMIT_AS)
    resource.setrlimit(resource.RLIMIT_AS, (MAX, hard))


class MRS_for_Non_Monotone(object):
    def __init__(self, start_poses, goal_poses, object_locations, LL, region_dict):
        ###### output   ############
        self.isMonotone = False
        self.parent = {}
        self.path_option = {}
        self.mutation_node = () # the node goes furthest
        ######
        self.n = len(start_poses)
        self.start_poses = start_poses
        self.goal_poses = goal_poses
        self.dependency_dict = {}
        self.path_dict = {}
        self.obj_locations = copy.deepcopy(object_locations)
        self.LL = copy.deepcopy(LL)
        self.region_dict = copy.deepcopy(region_dict)
        self.isMonotone = self.DFS()
        self.interface()

    def interface(self):
        new_parent = {}
        new_path_options = {}
        for key in self.parent.keys():
            new_key = 0
            for i in key:
                new_key += (1<<i)
            new_value = 0
            for i in self.parent[key]:
                new_value += (1<<i)
            new_parent[new_key] = new_value
            new_path_options[new_key] = self.path_option[key]
        self.parent = copy.deepcopy(new_parent)
        self.path_option = copy.deepcopy(new_path_options)
        new_mutation_node = 0
        for key in self.mutation_node:
            new_mutation_node += (1<<key)
        self.mutation_node = new_mutation_node

        
    def linked_list_conversion(self, graph):
        # print "graph"
        # print graph
        self.region_dict = {}  # (1,2,'a'): 0
        self.linked_list = {}  # 0:[1,2,3]
        for key in graph:
            index = len(self.region_dict.keys())
            self.region_dict[key] = index
            self.linked_list[index] = []
        for key in graph:
            for v in graph[key]:
                self.linked_list[self.region_dict[key]].append(self.region_dict[v])
        # print "LL"
        # print self.LL
        # print "region dict"
        # print self.region_dict

    def DFS(self):
        self.object_ordering = []
        self.explored = {}
        self.queue = [()]
        self.explored[()] = 0
        # it is a stack when pop(-1)
        old_node = self.queue.pop(-1)
        # Recursion
        Flag = self.DFS_rec(old_node)

        if Flag:
            task_index = tuple()
            for t in self.explored:
                if len(t)==self.n:
                    task_index = t
                    break
            if len(task_index)==0:
                return False
            current_task = task_index
            path_selection_dict = {}
            object_ordering = []
            while current_task in self.parent:
                parent_task = self.parent[current_task]
                last_object = current_task[-1]
                path_selection_dict[last_object] = self.path_option[current_task]
                object_ordering.append( last_object)
                
                
                current_task = parent_task
            path_selection_list = []
            for i in range(self.n):
                path_selection_list.append(path_selection_dict[i])
            self.path_selection = tuple(path_selection_list)
            self.object_ordering = list(reversed(object_ordering))
            return True
        else:
            furthest = -1
            for e in self.explored:
                if len(e)>=furthest:
                    self.mutation_node = e
                    furthest = len(e)
            # print "Non-monotone"
            # exit(0)
            return False
            # print MISTAKE

    def DFS_rec(self, old_node):
        FLAG = False # Flag = True iff we find a solution to the rearrangement problem
        for next_object in self.next_object(old_node):
            new_node = tuple(list(old_node) + [next_object])
            if new_node in self.explored:
                continue
            
            # Detect which poses are occupied
            occupied_poses = []
            for i in range(self.n):
                if i == next_object:
                    continue
                elif (i in old_node):
                    occupied_poses.append(self.goal_poses[i])
                else:
                    occupied_poses.append(self.start_poses[i])

            path_index = self.transformation(occupied_poses, next_object)
            if path_index >= 0:
                self.path_option[new_node] = path_index
                self.parent[new_node] = old_node
                self.queue.append(new_node)
                self.explored[new_node] = 0
                if len(new_node) == self.n:
                    return True
                FLAG = self.DFS_rec(new_node)
                if FLAG:
                    break
        return FLAG


    def next_object(self, index):
        for i in range(self.n):
            if (i in index): # it has moved
                pass
            else: # it is at the start pose
                yield i

    def generate_task_index(self, obj_set):
        task_index = 0
        for obj in obj_set:
            task_index += 2**obj
        return task_index

    def transformation(self,occupied_poses, obj):
        start = self.start_poses[obj]
        goal = self.goal_poses[obj]
        dependency_dict_key = (min(start, goal), max(start, goal))
        if dependency_dict_key not in self.dependency_dict:
            self.dependency_dict[dependency_dict_key] = []
            self.path_dict[dependency_dict_key] = []
        Available_Regions = []
        for region in self.region_dict.keys():
            OCCUPIED = False
            for pose in region:
                if pose in occupied_poses:
                    OCCUPIED = True
                    break
            if not OCCUPIED:
                Available_Regions.append(self.region_dict[region])
        if (self.region_dict[self.obj_locations[goal]] not in Available_Regions):
            # print "Not accessable"
            return -1
        if (self.region_dict[self.obj_locations[start]] not in Available_Regions):
            # print "Not accessable"
            return -1
        if (self.region_dict[self.obj_locations[start]] == self.region_dict[self.obj_locations[goal]]):
            path = [self.region_dict[self.obj_locations[start]]]
            dep_set = set(self.get_dependency_set_from_index(self.region_dict[self.obj_locations[start]]))
            self.path_dict[dependency_dict_key].append(list(reversed(path)))
            self.dependency_dict[dependency_dict_key].append(dep_set)
            return len(self.dependency_dict[dependency_dict_key]) - 1
            
        Found = False
        parents = {}
        explored = {}
        for key in self.region_dict.values():
            explored[key] = 0
        queue = [self.region_dict[self.obj_locations[start]]]
        explored[self.region_dict[self.obj_locations[start]]] = 1
        while (len(queue) >0) and (not Found):
            # stack(-1) for DFS and queue(0) for BFS
            old_node = queue.pop(-1)
            if old_node in self.LL:
                for region in self.LL[old_node]:
                    if explored[region]:
                        continue
                    if region not in Available_Regions:
                        continue
                    parents[region] = old_node
                    if region == self.region_dict[self.obj_locations[goal]]:
                        Found = True
                        break
                    queue.append(region)
                    explored[region] = 1
            else:
                print("Linked list error")
        
        for path_index in range(len(self.dependency_dict[dependency_dict_key])):
            path = self.dependency_dict[dependency_dict_key][path_index]
            OCCUPIED = False
            for pose in path:
                if pose in occupied_poses:
                    OCCUPIED = True
                    break
            if not OCCUPIED:
                return path_index

        if Found:
            path = []
            dep_set = set()
            current_node = self.region_dict[self.obj_locations[goal]]
            while current_node in parents:
                path.append(current_node)
                dep_set = dep_set.union(self.get_dependency_set_from_index(current_node))
                current_node = parents[current_node]
            path.append(current_node)
            dep_set = dep_set.union(self.get_dependency_set_from_index(current_node))
            if dependency_dict_key[0]==start:
                self.path_dict[dependency_dict_key].append(list(reversed(path)))
            else:
                self.path_dict[dependency_dict_key].append(list(path))
            self.dependency_dict[dependency_dict_key].append(dep_set)
            return len(self.dependency_dict[dependency_dict_key]) - 1
        else:
            return -1
                    
    def get_dependency_set_from_index(self, index):
        for key, value in self.region_dict.items():
            if value == index:
                region_tuple = key
                break
        dependency_set = set()
        for i in region_tuple:
            value = -1
            try:
                value = int(i)
            except ValueError:
                pass  # it was a string, not an int.
            if value >= -0.5:
                dependency_set = dependency_set.union({value})
        return dependency_set
