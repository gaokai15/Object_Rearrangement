import os
import sys
import copy
from collections import OrderedDict
from cgraph import genDenseCGraph, drawMotions, animatedMotions, drawProblem
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

# Disable
def blockPrint():
    sys.stdout = open(os.devnull, 'w')

# Restore
def enablePrint():
    sys.stdout = sys.__stdout__


def set_max_memory(MAX):
    soft, hard = resource.getrlimit(resource.RLIMIT_AS)
    resource.setrlimit(resource.RLIMIT_AS, (MAX, hard))


class partial_fmRS(object):
    def __init__(self, start_poses, goal_poses, object_locations, LL, region_dict):
        self.isMonotone = False
        self.DG = {}
        self.obstacle_set = set()
        self.potential_occupied_poses = set()
        self.start_poses = {}
        self.goal_poses = {}
        for i in start_poses.keys():
            if start_poses[i] == goal_poses[i]:
                self.obstacle_set.add(start_poses[i])
                self.potential_occupied_poses.add(start_poses[i])
            else:
                self.start_poses[i] = start_poses[i]
                self.goal_poses[i] = goal_poses[i]
                self.potential_occupied_poses.add(start_poses[i])
                self.potential_occupied_poses.add(goal_poses[i])

        self.dependency_dict = {}
        self.path_dict = {}

        self.obj_locations = copy.deepcopy(object_locations)
        self.LL = copy.deepcopy(LL)
        self.region_dict = copy.deepcopy(region_dict)

        self.parent = {}
        self.path_option = {}
        self.final_path_selections = {}
        self.ordering = []
        self.h = {}
        self.construct_path_dict(self.start_poses.keys())
        # self.dependency_dict_conversion()

        path_selections, removed_obj_list = self.move_objects()

        self.ordering = self.ordering + removed_obj_list

        for i in removed_obj_list:
            self.final_path_selections[i] = copy.deepcopy(path_selections[i])
            self.start_poses[i] = self.goal_poses[i]

        SCC_class = Strongly_Connected_Component(self.DG, set(self.DG.keys()))
        SCC = SCC_class.partition
        G_SCC = {}
        for i_scc in range(len(SCC)):
            G_SCC[i_scc] = []
            Collision = False
            for v in SCC[i_scc]:
                for j_scc in range(len(SCC)):
                    if i_scc == j_scc:
                        continue
                    if len(set(self.DG[v]).intersection(SCC[j_scc]))>0:
                        Collision = True
                        break
                if Collision:
                    break
            if Collision:
                G_SCC[i_scc].append(j_scc)
        topo_ordering = self.topological_sort(G_SCC)

        for i in topo_ordering:
            if len(SCC[i])==1:
                next_object = list(SCC[i])[0]

                # Detect which poses are occupied
                occupied_poses = copy.deepcopy(list(self.obstacle_set))
                for i in self.start_poses.keys():
                    if i == next_object:
                        continue
                    occupied_poses.append(self.start_poses[i])

                path_index = self.transformation(occupied_poses, next_object)
                if path_index >=0:
                    self.ordering.append(next_object)
                    self.final_path_selections[next_object] = copy.deepcopy(self.path_dict[next_object][path_index])
                    self.start_poses[next_object] = self.goal_poses[next_object]
            else:
                next_goal_poses = {}
                for obj in self.goal_poses.keys():
                    if obj in SCC[i]:
                        next_goal_poses[obj] = self.goal_poses[obj]
                    else:
                        next_goal_poses[obj] = self.start_poses[obj]
                fmRS = MCR_based_Monotone(self.start_poses, next_goal_poses, self.obstacle_set, self.obj_locations, self.LL, self.region_dict)
                self.ordering = self.ordering + fmRS.ordering
                for i in fmRS.ordering:
                    self.final_path_selections[i] = copy.deepcopy(fmRS.final_path_selections[i])
                    self.start_poses[i] = self.goal_poses[i]

        self.interface()

    def interface(self):
        if len(self.ordering) == len(self.start_poses):
            self.isMonotone = True
        self.parent = {}
        current_node = 0
        for obj in self.ordering:
            new_node = current_node + (1<<obj)
            self.parent[new_node] = current_node
            self.path_option[new_node] = 0
            current_node = new_node
        
        
    def dependency_dict_conversion(self):
        for key in self.dependency_dict.keys():
            number_set_list = self.dependency_dict[key]
            pose_set_list = []
            for number_set in number_set_list:
                pose_set = set()
                for number in number_set:
                    FOUND = False
                    for i in self.start_poses.keys():
                        if (self.start_poses[i] == number):
                            pose_set.add((i, 0))
                            FOUND = True
                            break
                    if FOUND:
                        continue
                    for i in self.goal_poses.keys():
                        if (self.goal_poses[i] == number):
                            pose_set.add((i, 1))
                            break
                pose_set_list.append(pose_set)
            self.dependency_dict[key] = pose_set_list

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

    def construct_path_dict(self, obj_list):
        for key in obj_list:
            self.dependency_set_pruning_search(key)
            # if len(self.dependency_dict[key])==0:
            #     print "isolation occurs, key = ", key
            #     self.add_trivial_path(key)

    def h_heuristic(self, key):
        h = {}
        goal_region = self.region_dict[self.obj_locations[self.goal_poses[key]]]
        parents = {}
        Q = [goal_region]
        h[goal_region] = 0
        while len(Q)!=0:
            old_node = Q.pop(0)
            if old_node in self.LL:
                for new_node in self.LL[old_node]:
                    if new_node in h:
                        continue
                    h[new_node] = h[old_node]+1
                    parents[new_node] = old_node
                    Q.append(new_node)
        
        dep_set = set()
        start_region = self.region_dict[self.obj_locations[self.start_poses[key]]]
        current_node = start_region
        path = []
        dep_set.union(self.get_dependency_set_from_index(current_node))
        while current_node in parents:  # while it is not the root(start pose).
            path.append(current_node)
            current_node = parents[current_node]
            dep_set.union(self.get_dependency_set_from_index(current_node))
        path.append(current_node)
        return h, len(path)-1

    def dependency_set_pruning_search(self, key):
        F = 1.5
        start = self.start_poses[key]
        goal = self.goal_poses[key]
        h, X = self.h_heuristic(key)
        dependency_dict_key = key
        if dependency_dict_key not in self.dependency_dict:
            self.dependency_dict[dependency_dict_key] = []
            self.path_dict[dependency_dict_key] = []  # key:obj, value: a list of dependency set
        vertex2node_dict = {}
        for region in self.region_dict.values():
            vertex2node_dict[region] = []
        node_dependency_set_dict = {}  # the dictionary for the dependency set of each node in the search tree
        nodes = {}
        parents = {}
        f = {}
        nodes[1] = self.region_dict[self.obj_locations[start]]
        node_dependency_set_dict[1] = self.get_dependency_set_from_index(nodes[1])
        vertex2node_dict[self.region_dict[self.obj_locations[start]]].append(1)
        f[1] = 0
        queue = [1]
        while len(queue) > 0:
            old_node = queue.pop()
            if nodes[old_node] in self.LL:  # if it has neighbor
                for pose in self.LL[nodes[old_node]]:
                    current_f = f[old_node] + 1
                    if (current_f + h[pose] > F*X):
                        continue
                    current_dependency_set = node_dependency_set_dict[old_node].union(
                        self.get_dependency_set_from_index(pose)
                    )
                    if len(current_dependency_set.intersection(self.obstacle_set))>0:
                        continue
                    Abandoned = False
                    for n in vertex2node_dict[pose]:
                        if current_dependency_set.issuperset(node_dependency_set_dict[n]):
                            Abandoned = True
                            break
                        if node_dependency_set_dict[n].issuperset(current_dependency_set):
                            vertex2node_dict[pose].remove(n)
                    if not Abandoned:
                        node = len(nodes) + 1
                        nodes[node] = pose
                        parents[node] = old_node
                        vertex2node_dict[pose].append(node)
                        queue.append(node)
                        f[node] = current_f
                        node_dependency_set_dict[node] = current_dependency_set

        goal_nodes = vertex2node_dict[self.region_dict[self.obj_locations[goal]]]

        for node in goal_nodes:
            current_node = node
            path = []
            while current_node in parents:  # while it is not the root(start pose).
                path.append(nodes[current_node])
                current_node = parents[current_node]
            path.append(nodes[current_node])
            # node_dependency_set_dict[node] = node_dependency_set_dict[node].difference({self.start_poses[key], self.goal_poses[key]})
            self.path_dict[dependency_dict_key].append(list(reversed(path)))
            self.dependency_dict[dependency_dict_key].append(node_dependency_set_dict[node])


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
        return dependency_set.intersection(self.potential_occupied_poses)

    def move_objects(self):
        paths = {}
        dead_end = []
        for obj in self.start_poses.keys():
            if (len(self.dependency_dict[obj])==0):
                dead_end.append(obj)
                continue
            least_dep = float('inf')
            MCR_path = -1
            for path_index in range(len(self.dependency_dict[obj])):
                if len(self.dependency_dict[obj][path_index])<least_dep:
                    least_dep = len(self.dependency_dict[obj][path_index])
                    MCR_path = path_index
            if MCR_path == -1:
                dead_end.append(obj)
            else:
                paths[obj] = MCR_path
        self.DG = {}
        for obj in self.start_poses.keys():
            self.DG[obj] = set()
        for obj in paths.keys():
            for dep in self.dependency_dict[obj][paths[obj]]:
                for key in self.start_poses.keys():
                    if key == obj:
                        continue
                    if dep == self.start_poses[key]:
                        self.DG[obj].add(key)
                    if dep == self.goal_poses[key]:
                        self.DG[key].add(obj)
        for obj in dead_end:
            for o in self.start_poses.keys():
                self.DG[obj].add(o)
        removed_obj_list = []
        GO_ON = True
        while(GO_ON):
            new_obj_list = []
            for i in self.DG.keys():
                if( len(self.DG[i])==0):
                    del self.DG[i]
                    new_obj_list.append(i)
            if len(new_obj_list)==0:
                GO_ON = False
            else:
                removed_obj_list = removed_obj_list + new_obj_list
                new_obj_set = set(new_obj_list)
                for key in self.DG.keys():
                    self.DG[key] = self.DG[key].difference(new_obj_set)
        path_selection_dict = {}
        for i in removed_obj_list:
            path_selection_dict[i] = self.path_dict[i][paths[i]]
        return path_selection_dict, removed_obj_list

    def transformation(self,occupied_poses, obj):
        start = self.start_poses[obj]
        goal = self.goal_poses[obj]
        dependency_dict_key = obj
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
            explored[key] = False
        queue = [self.region_dict[self.obj_locations[start]]]
        explored[self.region_dict[self.obj_locations[start]]] = True
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
                    explored[region] = True
            else:
                print("Linked list error")
        
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
            self.path_dict[dependency_dict_key].append(list(path))
            self.dependency_dict[dependency_dict_key].append(dep_set)
            return len(self.dependency_dict[dependency_dict_key]) - 1
        else:
            return -1

    def topological_sort(self, G):
        ordering = []
        while len(G)!=0:
            new_vertices = []
            for i in G.keys():
                if len(G[i]) == 0:
                    new_vertices.append(i)
            if len(new_vertices) == 0:
                return ordering
            for i in new_vertices:
                del G[i]
            new_vertices_set = set(new_vertices)
            for i in G.keys():
                intersect = new_vertices_set.intersection(set(G[i]))
                for j in intersect:
                    G[i].remove(j)
            ordering = ordering + new_vertices
        return ordering

class MCR_based_Monotone(object):
    def __init__(self, start_poses, goal_poses, obstacle_set, object_locations, LL, region_dict):
        self.final_path_selections = {}
        self.obstacle_set = copy.deepcopy(obstacle_set)
        self.potential_occupied_poses = copy.deepcopy(obstacle_set)
        self.start_poses = {}
        self.goal_poses = {}
        for i in start_poses.keys():
            if start_poses[i] == goal_poses[i]:
                self.obstacle_set.add(start_poses[i])
                self.potential_occupied_poses.add(start_poses[i])
            else:
                self.start_poses[i] = start_poses[i]
                self.goal_poses[i] = goal_poses[i]
                self.potential_occupied_poses.add(start_poses[i])
                self.potential_occupied_poses.add(goal_poses[i])
        self.n = len(self.start_poses.keys())
        self.dependency_dict = {}
        self.path_dict = {}
        self.obj_locations = copy.deepcopy(object_locations)
        self.ordering = []
        self.LL = copy.deepcopy(LL)
        self.region_dict = copy.deepcopy(region_dict)
        # self.linked_list_conversion(graph)
        self.h = {}
        self.construct_path_dict(self.start_poses.keys())
        self.dependency_dict_conversion()
        self.goal_objs = set()
        self.pending_objs = set(self.start_poses.keys()).difference(self.goal_objs)
        Fail = False
        while((not Fail) and (len(self.pending_objs)>0)):
            path_selections,new_obj_list = self.move_objects()
            self.ordering = self.ordering + new_obj_list
            for i in new_obj_list:
                self.final_path_selections[i] = copy.deepcopy(path_selections[i])
            if(len(new_obj_list)==0):
                Fail = True
                continue
            self.goal_objs = self.goal_objs.union(set(new_obj_list))
            self.pending_objs = set(self.start_poses.keys()).difference(self.goal_objs)
            if(len(self.pending_objs)>0):
                Fail = True # fmRS
                break
        if Fail:
            print "non monotone"
        else:
            print "monotone"

    def prune_dict(self, new_objs):
        empty_poses = set([(obj, 0) for obj in new_objs])
        occupied_poses = set([(obj, 1) for obj in new_objs])
        for obj in list(self.pending_objs):
            if obj not in self.dependency_dict:
                continue
            else:
                dep_set_index = 0
                while(dep_set_index<len(self.dependency_dict[obj])):
                    if (len(self.dependency_dict[obj][dep_set_index].intersection(occupied_poses))>0):
                        del self.dependency_dict[obj][dep_set_index]
                        del self.path_dict[obj][dep_set_index]
                        continue
                    self.dependency_dict[obj][dep_set_index] = self.dependency_dict[obj][dep_set_index].difference(empty_poses)
                    dep_set_index += 1
        
    def h_heuristic(self, key):
        h = {}
        a = self.goal_poses[key]
        b = self.obj_locations[a]
        c = self.region_dict[b]
        goal_region = self.region_dict[self.obj_locations[self.goal_poses[key]]]
        parents = {}
        Q = [goal_region]
        h[goal_region] = 0
        while len(Q)!=0:
            old_node = Q.pop(0)
            if old_node in self.LL:
                for new_node in self.LL[old_node]:
                    if new_node in h:
                        continue
                    h[new_node] = h[old_node]+1
                    parents[new_node] = old_node
                    Q.append(new_node)
        
        dep_set = set()
        start_region = self.region_dict[self.obj_locations[self.start_poses[key]]]
        current_node = start_region
        path = []
        dep_set.union(self.get_dependency_set_from_index(current_node))
        while current_node in parents:  # while it is not the root(start pose).
            path.append(current_node)
            current_node = parents[current_node]
            dep_set.union(self.get_dependency_set_from_index(current_node))
        path.append(current_node)
        return h, len(path)-1




    def move_objects(self):
        path_selection_dict = {}
        paths = {}
        dead_end = []
        for obj in list(self.pending_objs):
            if (len(self.dependency_dict[obj])==0):
                dead_end.append(obj)
                continue
            least_dep = float('inf')
            MCR_path = -1
            for path_index in range(len(self.dependency_dict[obj])):
                if len(self.dependency_dict[obj][path_index])<least_dep:
                    least_dep = len(self.dependency_dict[obj][path_index])
                    MCR_path = path_index
            if MCR_path == -1:
                dead_end.append(obj)
            else:
                paths[obj] = MCR_path
        DG = {}
        for obj in list(self.pending_objs):
            DG[obj] = set()
        for obj in paths.keys():
            for dep in self.dependency_dict[obj][paths[obj]]:
                if (dep[0]==obj):
                    continue
                if (dep[1] ==0) :
                    DG[obj].add(dep[0])
                else:
                    DG[dep[0]].add(obj)
        for obj in dead_end:
            for o in self.start_poses.keys():
                DG[obj].add(o)
        removed_obj_list = []
        GO_ON = True
        while(GO_ON):
            new_obj_list = []
            for i in DG.keys():
                if( len(DG[i])==0):
                    del DG[i]
                    new_obj_list.append(i)
            if len(new_obj_list)==0:
                GO_ON = False
            else:
                removed_obj_list = removed_obj_list + new_obj_list
                new_obj_set = set(new_obj_list)
                for key in DG.keys():
                    DG[key] = DG[key].difference(new_obj_set)
        for i in removed_obj_list:
            path_selection_dict[i] = copy.deepcopy(self.path_dict[i][paths[i]])
        return path_selection_dict, removed_obj_list
        


    
    def dependency_dict_conversion(self):
        for key in self.dependency_dict.keys():
            number_set_list = self.dependency_dict[key]
            pose_set_list = []
            for number_set in number_set_list:
                pose_set = set()
                for number in number_set:
                    FOUND = False
                    for i in self.start_poses.keys():
                        if (self.start_poses[i] == number):
                            pose_set.add((i, 0))
                            # FOUND = True
                            # break
                    # if FOUND:
                    #     continue
                    for i in self.goal_poses.keys():
                        if (self.goal_poses[i] == number):
                            pose_set.add((i, 1))
                            break
                pose_set_list.append(pose_set)
            self.dependency_dict[key] = pose_set_list

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

    def construct_path_dict(self, obj_list):
        for key in obj_list:
            self.dependency_set_pruning_search(key)
            # if len(self.dependency_dict[key])==0:
            #     print "isolation occurs, key = ", key
            #     self.add_trivial_path(key)

    def dependency_set_pruning_search(self, key):
        F = 1.5
        h, X = self.h_heuristic(key)
        self.path_dict[key] = []
        self.dependency_dict[key] = []  # key:obj, value: a list of dependency set
        vertex2node_dict = {}
        for region in self.region_dict.values():
            vertex2node_dict[region] = []
        node_dependency_set_dict = {}  # the dictionary for the dependency set of each node in the search tree
        nodes = {}
        parents = {}
        f = {}
        nodes[1] = self.region_dict[self.obj_locations[self.start_poses[key]]]
        node_dependency_set_dict[1] = self.get_dependency_set_from_index(nodes[1])
        vertex2node_dict[self.region_dict[self.obj_locations[self.start_poses[key]]]].append(1)
        f[1] = 0
        queue = [1]
        while len(queue) > 0:
            old_node = queue.pop()
            if nodes[old_node] in self.LL:  # if it has neighbor
                for pose in self.LL[nodes[old_node]]:
                    current_f = f[old_node] + 1
                    if (current_f + h[pose] > F*X):
                        continue
                    current_dependency_set = node_dependency_set_dict[old_node].union(
                        self.get_dependency_set_from_index(pose)
                    )
                    if len(current_dependency_set.intersection(self.obstacle_set))>0:
                        continue
                    Abandoned = False
                    for n in vertex2node_dict[pose]:
                        if current_dependency_set.issuperset(node_dependency_set_dict[n]):
                            Abandoned = True
                            break
                        if node_dependency_set_dict[n].issuperset(current_dependency_set):
                            vertex2node_dict[pose].remove(n)
                    if not Abandoned:
                        node = len(nodes) + 1
                        nodes[node] = pose
                        parents[node] = old_node
                        vertex2node_dict[pose].append(node)
                        queue.append(node)
                        f[node] = current_f
                        node_dependency_set_dict[node] = current_dependency_set

        goal_nodes = vertex2node_dict[self.region_dict[self.obj_locations[self.goal_poses[key]]]]

        for node in goal_nodes:
            current_node = node
            path = []
            while current_node in parents:  # while it is not the root(start pose).
                path.append(nodes[current_node])
                current_node = parents[current_node]
            path.append(nodes[current_node])
            # node_dependency_set_dict[node] = node_dependency_set_dict[node].difference({self.start_poses[key], self.goal_poses[key]})
            self.path_dict[key].append(list(reversed(path)))
            self.dependency_dict[key].append(node_dependency_set_dict[node])

        # print "parents", parents
        # print "goal", goal_nodes
        # print "nodes", nodes
        # print "node_dependency_set_dict"
        # print node_dependency_set_dict
        # print "vertex2node_dict"
        # print vertex2node_dict

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
        return dependency_set.intersection(self.potential_occupied_poses)


class Strongly_Connected_Component(object):
    def __init__(self, Graph, vertex_set):
        self.Graph = Graph
        self.remaining_vertices = vertex_set
        self.Finish_queue = []
        self.DFS()
        GT = self.Transpose_Graph(Graph)
        self.partition = self.Second_DFS(GT, self.Finish_queue)

    def DFS(self):
        while len(self.remaining_vertices)>0:
            stack = [self.remaining_vertices.pop()]
            while len(stack)>0:
                old_vertex = stack.pop(-1)
                self.DFS_rec(old_vertex)
                self.Finish_queue.append(old_vertex)
        # print "1", Finish_queue
        # print "2", list(reversed(Finish_queue))
        self.Finish_queue = list(reversed(self.Finish_queue)) # decreasing finishing

    def DFS_rec(self, old_vertex):
        for new_vertex in self.Graph[old_vertex]:
            if new_vertex in self.remaining_vertices:
                self.remaining_vertices.remove(new_vertex)
                self.DFS_rec(new_vertex)
                self.Finish_queue.append(new_vertex)
    
    def Transpose_Graph(self,G):
        GT = {}
        for i in G.keys():
            GT[i] = []
        for out_deg in G:
            for in_deg in G[out_deg]:
                GT[in_deg].append(out_deg)
        return GT

    def Second_DFS(self, Graph, finish_queue):
        Partition = []
        remaining_vertices = copy.deepcopy(finish_queue)
        while len(remaining_vertices)>0:
            new_tree = set()
            root = remaining_vertices.pop(0)
            stack = [root]
            new_tree.add(root)
            while len(stack)>0:
                old_vertex = stack.pop(-1)
                for new_vertex in Graph[old_vertex]:
                    if new_vertex not in remaining_vertices:
                        continue
                    stack.append(new_vertex)
                    new_tree.add(new_vertex)
                    remaining_vertices.remove(new_vertex)
            Partition.append(new_tree)
        return Partition
