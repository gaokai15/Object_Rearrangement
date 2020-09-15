from __future__ import print_function
import os
import sys
import copy
import resource
from time import time
import cPickle as pickle
from collections import OrderedDict
from itertools import combinations, product

import numpy as np
# import gurobipy as gp
# from gurobipy import GRB

from dspace import *

visualize = False
num_buffers = 2


def set_max_memory(MAX):
    soft, hard = resource.getrlimit(resource.RLIMIT_AS)
    resource.setrlimit(resource.RLIMIT_AS, (MAX, hard))


class Experiments(object):
    def single_instance(self, space):
        graph = space.RGAdj
        object_locations = space.pose2reg

        start_poses = {}
        goal_poses = {}
        for pid in space.poseMap:
            dd = str(pid).strip('ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz')
            sd = str(pid).strip('0123456789')
            if 'S' in sd:
                start_poses[int(dd)] = pid
            elif 'G' in sd:
                goal_poses[int(dd)] = pid

        region_dict, LL = linked_list_conversion(graph)
        DFS_Rec_for_Monotone_General(start_poses, goal_poses, {}, {}, object_locations, LL, region_dict)

        start = time()
        DFS_non_gen = Non_Monotone_Solver_General(graph, object_locations, start_poses, goal_poses)
        stop = time()
        DFS_non_time = stop - start
        print("DFS_time", DFS_non_time)

        print("DFS_non_monotone_object_ordering", DFS_non_gen.object_ordering)


# class DG_Space(object):
#     def __init__(self, path_dict):
#         self.path_dict = path_dict
#         self.DG_space_construction()
#         # print("Finish DG space construction.\n")

#     def DG_space_construction(self):
#         self.DGs = {}
#         objs = self.path_dict.keys()
#         path_choices = [len(self.path_dict[obj]) - 1 for obj in objs]
#         for path_set in self.choice_generator(path_choices):
#             self.DGs[tuple(path_set)] = self.DG_construction(path_set)
#         # print("objs: ", objs)
#         # print("path choices: ", path_choices)
#         # print("All dependency graphs: ")
#         # self.printDGs()

#     def printDGs(self):
#         for path_comb in self.DGs:
#             print(str(path_comb) + ": " + str(self.DGs[path_comb]))

#     def choice_generator(self, path_choices):
#         n = len(path_choices)
#         choice = [0 for i in range(n)]
#         while 1:
#             pointer = n - 1
#             while (pointer > 0) and (choice[pointer] == path_choices[pointer]):
#                 pointer -= 1
#             if (pointer == 0) and (choice[pointer] == path_choices[pointer]):
#                 yield path_choices
#                 break
#             else:
#                 yield choice
#                 choice[pointer] += 1
#                 for i in range(pointer + 1, n):
#                     choice[i] = 0

#     def DG_construction(self, path_set):
#         n = len(path_set)
#         M = np.zeros([n, n])
#         total_num_constr = 0
#         objs = self.path_dict.keys()
#         for i in range(n):
#             key = objs[i]
#             total_num_constr += len(self.path_dict[key][path_set[i]])
#             for constr in self.path_dict[key][path_set[i]]:
#                 if constr[1] == 0:
#                     M[i, objs.index(constr[0])] = 1
#                 else:
#                     M[objs.index(constr[0]), i] = 1
#         num_constr = np.count_nonzero(M)
#         # num_feedback = self.count_feedback_arc(M)
#         num_feedback, vertices_to_be_removed, final_order = self.count_feedback_vertex(M)
#         # return [total_num_constr, num_constr, num_feedback]
#         return [total_num_constr, num_constr, num_feedback, vertices_to_be_removed, final_order]

#     def count_feedback_vertex(self, M):
#         n = M.shape[0]
#         m = gp.Model()
#         ### The first step is to add the inner edges
#         for i in range(n):
#             M[i, i] = 1

#         m.setParam('OutputFlag', 0)
#         m.modelSense = GRB.MINIMIZE

#         y = m.addVars(2 * n, 2 * n, vtype=GRB.BINARY)
#         m.setObjective(
#             sum(
#                 sum(M[k, i] * y[k, i + n] for k in range(i)) + M[i, i] * (1 - y[i, i + n]) +
#                 sum(M[l, i] * y[l, i + n] for l in range(i + 1, n)) for i in range(n)
#             )
#         )
#         for i in range(2 * n):
#             for j in range(i + 1, 2 * n):
#                 for k in range(j + 1, 2 * n):
#                     m.addConstr(y[i, j] + y[j, k] + y[k, i] >= 1)
#                     m.addConstr(y[i, j] + y[j, k] + y[k, i] <= 2)
#         for i in range(2 * n):
#             for j in range(i + 1, 2 * n):
#                 m.addConstr(y[i, j] + y[j, i] == 1)

#         m.optimize()
#         obj = m.getObjective()

#         vertices_to_be_removed = []
#         for i in range(n):
#             for j in range(n):
#                 if (M[i, j] == 1.0):
#                     if (i != j) and (y[i, j + n].x == 1.0):
#                         vertices_to_be_removed.append((i, j))
#                     if (i == j) and (y[i, j + n].x == 0.0):
#                         vertices_to_be_removed.append((i, j))

#         Y = np.zeros([n, 2 * n])
#         for i in range(n):
#             for j in range(2 * n):
#                 Y[i, j] = y[i, j].x
#         obj_indexes = [i for i in range(n)]
#         order_count = np.sum(Y, axis=1)
#         final_order = [obj_index for _, obj_index in sorted(zip(order_count, obj_indexes), reverse=True)]

#         return obj.getValue(), vertices_to_be_removed, final_order

#     def count_feedback_arc(self, M):
#         n = M.shape[0]
#         m = gp.Model()

#         m.setParam('OutputFlag', 0)
#         m.modelSense = GRB.MINIMIZE

#         y = m.addVars(n, n, vtype=GRB.BINARY)
#         m.setObjective(
#             sum(
#                 sum(M[k, j] * y[k, j]
#                     for k in range(j)) + sum(M[l, j] * (1 - y[j, l])
#                                              for l in range(j + 1, n))
#                 for j in range(n)
#             )
#         )
#         for i in range(n):
#             for j in range(i + 1, n):
#                 for k in range(j + 1, n):
#                     m.addConstr(y[i, j] + y[j, k] - y[i, k] <= 1)
#                     m.addConstr(-y[i, j] - y[j, k] + y[i, k] <= 0)

#         m.optimize()
#         obj = m.getObjective()
#         return obj.getValue()

# class feekback_vertex_ILP(object):
#     def __init__(self, path_dict):
#         self.path_dict = path_dict
#         self.optimum = self.run_vertex_ILP()
#         # self.optimum = self.run_arc_ILP()

#     def run_vertex_ILP(self):
#         MOST_PATH = 0
#         for paths in self.path_dict.values():
#             MOST_PATH = max(MOST_PATH, len(paths))
#         print("most paths", MOST_PATH)
#         n = len(self.path_dict)
#         objs = self.path_dict.keys()
#         dependency_dict = {}
#         for i in range(n):
#             key = objs[i]
#             for path_index in range(len(self.path_dict[key])):
#                 for constr in self.path_dict[key][path_index]:
#                     if constr[1] == 0:
#                         if (i, objs.index(constr[0])) not in dependency_dict:
#                             dependency_dict[(i, objs.index(constr[0]))] = []
#                         dependency_dict[(i, objs.index(constr[0]))].append([key, path_index])
#                     else:
#                         if (objs.index(constr[0]), i) not in dependency_dict:
#                             dependency_dict[(objs.index(constr[0]), i)] = []
#                         dependency_dict[(objs.index(constr[0]), i)].append([key, path_index])

#         m = gp.Model()

#         m.setParam('OutputFlag', 0)
#         m.modelSense = GRB.MINIMIZE

#         ###### Minimum feedback vertex ILP formulation ######
#         ### variables
#         y = m.addVars(2 * n, 2 * n, vtype=GRB.BINARY)
#         M = m.addVars(n, n, vtype=GRB.BINARY)
#         x = m.addVars(n, MOST_PATH, vtype=GRB.BINARY)
#         ### Objective function ###
#         m.setObjective(
#             sum(
#                 sum(M[k, i] * y[k, i + n] for k in range(i)) + M[i, i] * (1 - y[i, i + n]) +
#                 sum(M[l, i] * y[l, i + n] for l in range(i + 1, n)) for i in range(n)
#             )
#         )
#         ### constraints ###
#         for i in range(n):
#             m.addConstr(M[i, i] == 1)

#         for i in range(2 * n):
#             for j in range(i + 1, 2 * n):
#                 for k in range(j + 1, 2 * n):
#                     m.addConstr(y[i, j] + y[j, k] + y[k, i] >= 1)
#                     m.addConstr(y[i, j] + y[j, k] + y[k, i] <= 2)
#         for i in range(2 * n):
#             for j in range(i + 1, 2 * n):
#                 m.addConstr(y[i, j] + y[j, i] == 1)

#         for i in range(n):
#             for j in range(len(self.path_dict[objs[i]]), MOST_PATH):
#                 m.addConstr(x[i, j] == 0)
#         for i in range(n):
#             m.addConstr(sum(x[i, u] for u in range(MOST_PATH)) == 1)
#         m.addConstrs(
#             M[j, k] <= sum(x[objs.index(i), u]
#                            for (i, u) in dependency_dict[(j, k)])
#             for (j, k) in dependency_dict.keys()
#         )
#         for (j, k) in dependency_dict.keys():
#             for (i, u) in dependency_dict[(j, k)]:
#                 m.addConstr(M[j, k] >= x[objs.index(i), u])

#         m.optimize()
#         obj = m.getObjective()

#         # print("The useful information below: ")

#         ### figure out what path options are chosen for each object
#         path_selection = []
#         for i in range(n):
#             for j in range(MOST_PATH):
#                 if x[i, j].x > 0.5:
#                     path_selection.append(j)
#         # print("path_selection: ", path_selection)

#         ### figure out the vertices and constraints to be removed
#         vertices_to_be_removed = []
#         for i in range(n):
#             for j in range(n):
#                 if (M[i, j].x == 1.0):
#                     if (i != j) and (y[i, j + n].x == 1.0):
#                         vertices_to_be_removed.append((i, j))
#                     if (i == j) and (y[i, j + n].x == 0.0):
#                         vertices_to_be_removed.append((i, j))

#         ### display the dependency graph as a matrix C
#         C = np.zeros([n, n])
#         for i in range(n):
#             for j in range(n):
#                 if M[i, j].x == 1.0:
#                     if i != j:
#                         C[i, j] = 1
#         # print("The dependecy graph is: ")
#         # print(C)

#         Y = np.zeros([n, 2 * n])
#         for i in range(n):
#             for j in range(2 * n):
#                 Y[i, j] = y[i, j].x
#         obj_indexes = [i for i in range(n)]
#         order_count = np.sum(Y, axis=1)
#         final_order = [obj_index for _, obj_index in sorted(zip(order_count, obj_indexes), reverse=True)]
#         print("Final objects order: ")
#         print(final_order)

#         # print("order: ", y)
#         # print("\n")
#         # print("DG: " , M)
#         # print("\n")
#         # print("Number of vertex to be removed: ", obj.getValue())
#         # print("Vertices to be removed: ", vertices_to_be_removed)

#         return obj.getValue(), vertices_to_be_removed, tuple(path_selection), final_order

#     def run_arc_ILP(self):
#         MOST_PATH = 0
#         for paths in self.path_dict.values():
#             MOST_PATH = max(MOST_PATH, len(paths))
#         print("most paths", MOST_PATH)
#         n = len(self.path_dict)
#         objs = self.path_dict.keys()
#         dependency_dict = {}
#         for i in range(n):
#             key = objs[i]
#             for path_index in range(len(self.path_dict[key])):
#                 for constr in self.path_dict[key][path_index]:
#                     if constr[1] == 0:
#                         if (i, objs.index(constr[0])) not in dependency_dict:
#                             dependency_dict[(i, objs.index(constr[0]))] = []
#                         dependency_dict[(i, objs.index(constr[0]))].append([key, path_index])
#                     else:
#                         if (objs.index(constr[0]), i) not in dependency_dict:
#                             dependency_dict[(objs.index(constr[0]), i)] = []
#                         dependency_dict[(objs.index(constr[0]), i)].append([key, path_index])

#         m = gp.Model()

#         m.setParam('OutputFlag', 0)
#         m.modelSense = GRB.MINIMIZE

#         y = m.addVars(n, n, vtype=GRB.BINARY)
#         M = m.addVars(n, n, vtype=GRB.BINARY)
#         x = m.addVars(n, MOST_PATH, vtype=GRB.BINARY)
#         m.setObjective(
#             sum(
#                 sum(M[k, j] * y[k, j]
#                     for k in range(j)) + sum(M[l, j] * (1 - y[j, l])
#                                              for l in range(j + 1, n))
#                 for j in range(n)
#             )
#         )

#         for i in range(n):
#             for j in range(i + 1, n):
#                 for k in range(j + 1, n):
#                     m.addConstr(y[i, j] + y[j, k] - y[i, k] <= 1)
#                     m.addConstr(-y[i, j] - y[j, k] + y[i, k] <= 0)
#         for i in range(n):
#             for j in range(len(self.path_dict[objs[i]]), MOST_PATH):
#                 m.addConstr(x[i, j] == 0)
#         for i in range(n):
#             m.addConstr(sum(x[i, j] for j in range(MOST_PATH)) == 1)
#         m.addConstrs(
#             M[j, k] >= sum(x[objs.index(u), i]
#                            for (u, i) in dependency_dict[(j, k)]) / (2 * MOST_PATH)
#             for (j, k) in dependency_dict.keys()
#         )

#         m.optimize()
#         obj = m.getObjective()
#         DG_index = []
#         for i in range(n):
#             for j in range(MOST_PATH):
#                 if x[i, j].x > 0.5:
#                     DG_index.append(j)
#         print("DG_index", DG_index)
#         # print("path selection", x)
#         # print("order", y)
#         # print("DG", M)
#         return obj.getValue()

# class Inner_Buffer_IQP(object):
#     def __init__(self, path_dict, n, num_buffers):
#         self.n = n
#         self.num_buffers = num_buffers
#         self.path_dict = path_dict
#         self.data = [True]
#         start = time.time()
#         try:
#             self.optimum = self.run_single_buffer_IQP()
#         except Exception:
#             self.data[0] = False
#             self.optimum = []
#             print("Fail to solve the problem!")
#         stop = time.time()
#         self.data.append(stop - start)

#     def run_single_buffer_IQP(self):
#         MOST_PATH = 0
#         Total_Path = 0
#         for paths in self.path_dict.values():
#             MOST_PATH = max(MOST_PATH, len(paths))
#             Total_Path += len(paths)

#         self.data.append(MOST_PATH)
#         self.data.append(Total_Path)

#         print("MOST PATH", MOST_PATH)
#         print("total paths", Total_Path)
#         m = gp.Model()

#         m.Params.NodefileStart = 0.5
#         m.Params.TimeLimit = 200.0
#         m.Params.Threads = 8
#         # m.Params.MIPFocus = 1
#         # m.Params.MinRelNodes = 1
#         m.Params.Method = 1
#         # m.setParam(m.Params.Method, 1)

#         # m.setParam('OutputFlag',0)
#         m.modelSense = GRB.MAXIMIZE

#         ######### IQP #############
#         ### Variables
#         z1 = m.addVar(vtype=GRB.INTEGER, ub=self.n, lb=0)
#         z2 = m.addVar(vtype=GRB.INTEGER, lb=0)
#         y = m.addVars(2 * self.n, 2 * self.n, vtype=GRB.BINARY)
#         c = m.addVars(2 * self.n, 2 * self.n, vtype=GRB.BINARY)
#         x = m.addVars(2 * self.n, 2 * self.n + self.num_buffers, MOST_PATH, vtype=GRB.BINARY)
#         ### Objective function ###
#         m.setObjective(z1 - (self.n + 1) * z2)
#         ### constraints ###

#         m.addConstr(sum(sum(x[2 * i, 2 * i + 1, k] for k in range(MOST_PATH)) for i in range(self.n)) >= z1)

#         for i in range(self.n):
#             # each object choose one buffer
#             m.addConstr(
#                 sum(sum(x[2 * i + 1, p, k]
#                         for k in range(MOST_PATH))
#                     for p in range(2 * self.n + self.num_buffers)) == 1
#             )
#             # obj i goes before obj n+i
#             m.addConstr(c[self.n + i, i] == 1)

#         # |FAS| = 0
#         m.addConstr(
#             sum(
#                 (sum(c[k, i] * y[k, i]
#                      for k in range(i)) + sum(c[l, i] * (1 - y[i, l])
#                                               for l in range(i, 2 * self.n)))
#                 for i in range(2 * self.n)
#             ) <= z2
#         )

#         for k in range(2 * self.n):
#             for i in range(k + 1, 2 * self.n):
#                 for l in range(i + 1, 2 * self.n):
#                     m.addConstr(y[k, i] + y[i, l] - y[k, l] >= 0)
#                     m.addConstr(y[k, i] + y[i, l] - y[k, l] <= 1)

#         for i in range(self.n):
#             for p in range(2 * self.n + self.num_buffers):
#                 # where the obj n+i starts is where the obj i ends
#                 m.addConstr(
#                     sum(x[2 * i, p, k] for k in range(MOST_PATH)) - sum(x[2 * i + 1, p, k]
#                                                                         for k in range(MOST_PATH)) == 0
#                 )

#             # obj i cannot use Si to be the buffer
#             m.addConstr(sum(x[2 * i, 2 * i, k] for k in range(MOST_PATH)) <= 0)

#         for key in self.path_dict.keys():
#             path_key_list = []
#             if (key[0] == key[1]) and (key[0] % 2):
#                 path_key_list = [(key[0], key[0])]
#             elif (key[1] >= 2 * self.n):
#                 path_key_list = [(key[0], key[1])]
#             elif (key[0] != key[1]):
#                 path_key_list = [(key[0], key[1]), (key[1], key[0])]
#             for path_key in path_key_list:
#                 obj_index = path_key[0] // 2 + (path_key[0] % 2) * self.n
#                 for k in range(len(self.path_dict[key])):
#                     dependency_set = self.path_dict[key][k]
#                     for pose in dependency_set:
#                         #collides with start poses
#                         if (pose[1] == 0):
#                             m.addConstr(x[path_key[0], path_key[1], k] - c[obj_index, pose[0]] <= 0)
#                             for l in range(self.n):
#                                 m.addConstr(
#                                     sum(x[2 * l, 2 * pose[0], k]
#                                         for k in range(MOST_PATH)) * x[path_key[0], path_key[1], k] -
#                                     (c[l, obj_index] + c[obj_index, self.n + l]) <= 0
#                                 )
#                         # collides with goal poses
#                         if (pose[1] == 1):
#                             m.addConstr(x[path_key[0], path_key[1], k] - c[self.n + pose[0], obj_index] <= 0)
#                             for l in range(self.n):
#                                 m.addConstr(
#                                     sum(x[2 * l, 2 * pose[0] + 1, k]
#                                         for k in range(MOST_PATH)) * x[path_key[0], path_key[1], k] -
#                                     (c[l, obj_index] + c[obj_index, self.n + l]) <= 0
#                                 )
#                         if (pose[1] == 2):
#                             for l in range(self.n):
#                                 m.addConstr(
#                                     sum(x[2 * l, 2 * self.n + pose[0], k]
#                                         for k in range(MOST_PATH)) * x[path_key[0], path_key[1], k] -
#                                     (c[l, obj_index] + c[obj_index, self.n + l]) <= 0
#                                 )
#                 # if there is no such path, you cannot use it
#                 for k in range(len(self.path_dict[key]), MOST_PATH):
#                     m.addConstr(x[path_key[0], path_key[1], k] <= 0)

#         m.optimize()
#         obj = m.getObjective()
#         if obj.getValue() >= 0:
#             self.data.append(True)
#         else:
#             self.data.append(False)
#         try:
#             self.data.append(self.n - obj.getValue())
#         except Exception:
#             self.data.append(-1)

#         ### figure out what path options are chosen
#         path_selection = {}
#         buffer_selection = {}
#         for i in range(self.n):
#             FLAG = False
#             for p in range(2 * self.n + self.num_buffers):
#                 for k in range(MOST_PATH):
#                     if x[2 * i, p, k].x == 1:
#                         FLAG = True
#                         buffer_selection[i] = p
#                         path_selection[i] = k
#                         break
#                 if FLAG:
#                     for j in range(MOST_PATH):
#                         if x[2 * i + 1, p, j].x == 1:
#                             print(2 * i + 1, p, j)
#                             path_selection[i + self.n] = j
#                             break
#                     break

#         # print(x and c)
#         # for i in range(2*self.n):
#         #     for j in range(2*self.n):
#         #         lst = [x[i,j,k].x for k in range(MOST_PATH)]
#         #         print("x", [i,j], lst)

#         # for i in range(2*self.n):
#         #     for j in range(2*self.n):
#         #         if c[i,j].x>=0.5:
#         #             print(i,j)

#         ### figure out the order
#         order = []
#         for i in range(2 * self.n):
#             ranking = 0
#             for j in range(i):
#                 ranking += y[j, i].x
#             for j in range(i + 1, 2 * self.n):
#                 ranking += (1 - y[i, j].x)
#             order.append((ranking, i))
#         sorted_order = sorted(order)
#         final_order = [object_ranking[1] for object_ranking in sorted_order]

#         print("path_selection", path_selection)
#         print("buffer_selection", buffer_selection)
#         print("final order", final_order)
#         return 2 * self.n - obj.getValue(), buffer_selection, tuple(
#             [path_selection[i] for i in range(2 * self.n)]
#         ), final_order

# class Generate_Path_Dictionary(object):
#     # Input: the linked list of the Connectivity Graph.
#     # Output: the dependency dictionary
#     def __init__(self, graph):
#         self.path_dict = {}
#         self.dependency_dict = {}
#         self.n = len(graph) / 2
#         # print("the number of objects:", self.n)
#         self.start_pose = range(1, self.n + 1)
#         self.linked_list_conversion(graph)
#         self.construct_path_dict()

#     def linked_list_conversion(self, graph):
#         self.LL = {}
#         for key in graph:
#             self.LL[(key // 2 + 1, key % 2)] = []
#             for pose in graph[key]:
#                 self.LL[(key // 2 + 1, key % 2)].append((pose // 2 + 1, pose % 2))

#     def construct_path_dict(self):
#         for key in self.start_pose:
#             self.pruning_search(key)
#             # if len(self.dependency_dict[key])==0:
#             #     print("isolation occurs, key = ", key)
#             #     self.add_trivial_path(key)

#     def pruning_search(self, key):
#         self.path_dict[key] = []
#         self.dependency_dict[key] = []
#         nodes = {}
#         parents = {}
#         pruning = {}
#         goal_nodes = []
#         nodes[1] = (key, 0)
#         queue = [1]
#         while len(queue) > 0:
#             node = queue.pop()
#             if nodes[node] == (key, 1):
#                 goal_nodes.append(node)
#                 continue
#             if node in parents:
#                 pruning_set = pruning[parents[node]]
#             else:
#                 pruning_set = set(nodes[node])
#             if nodes[node] in self.LL:
#                 for pose in self.LL[nodes[node]]:
#                     if pose not in pruning_set:
#                         index = len(nodes) + 1
#                         nodes[index] = pose
#                         queue.append(index)
#                         parents[index] = node
#                 pruning[node] = pruning_set.union(self.LL[nodes[node]])
#         # print("parents", parents)
#         # print("goal", goal_nodes)
#         # print("nodes", nodes)
#         for node in goal_nodes:
#             current_node = node
#             path = []
#             dependency_set = set()
#             while current_node in parents:
#                 path.append(nodes[current_node])
#                 dependency_set = dependency_set.union({nodes[current_node]})
#                 current_node = parents[current_node]
#             path.append(current_node)
#             dependency_set = dependency_set.union({nodes[current_node]})
#             dependency_set = dependency_set.difference({(key, 0), (key, 1)})
#             self.path_dict[key].append(list(reversed(path)))
#             self.dependency_dict[key].append(dependency_set)

# class Dense_Path_Generation(object):
#     # Input: Danniel's region connectivity graph
#     # Output: the dependency dictionary
#     def __init__(self, num_Objs, graph, obj_locations):
#         self.obj_locations = obj_locations
#         self.path_dict = {}
#         self.dependency_dict = {}
#         self.n = num_Objs
#         # print("the number of objects:", self.n)
#         self.poses = range(len(obj_locations))
#         # self.start_pose = range(0, self.n)
#         self.linked_list_conversion(graph)
#         self.construct_path_dict()
#         self.dependency_dict_conversion()
#         # dependency_dict: key:(5,8) value: [set((2,1), (1,0), ..., (4,0)), set(...)]

#     def dependency_dict_conversion(self):
#         for key in self.dependency_dict.keys():
#             number_set_list = self.dependency_dict[key]
#             pose_set_list = []
#             for number_set in number_set_list:
#                 pose_set = set()
#                 for number in number_set:
#                     if number < 2 * self.n:
#                         pose_set = pose_set.union({(number // 2, number % 2)})
#                     else:
#                         pose_set = pose_set.union({(number - 2 * self.n, 2)})
#                 pose_set_list.append(pose_set)
#             self.dependency_dict[key] = pose_set_list

#     def linked_list_conversion(self, graph):
#         # print("graph")
#         # print(graph)
#         self.region_dict = {}  # (1,2,'a'): 0
#         self.LL = {}  # 0:[1,2,3]
#         for key in graph:
#             index = len(self.region_dict.keys())
#             self.region_dict[key] = index
#             self.LL[index] = []
#         for key in graph:
#             for v in graph[key]:
#                 self.LL[self.region_dict[key]].append(self.region_dict[v])
#         # print("LL")
#         # print(self.LL)
#         # print("region dict")
#         # print(self.region_dict)

#     def construct_path_dict(self):
#         for i in range(2 * self.n):
#             key1 = self.poses[i]
#             for j in range(i, len(self.poses)):
#                 if i == j:
#                     if j % 2:
#                         self.dependency_dict[(key1, key1)] = [
#                             self.get_dependency_set_from_index(self.region_dict[self.obj_locations[key1]])
#                         ]
#                         self.path_dict[(key1, key1)] = [
#                             [self.region_dict[self.obj_locations[key1]], self.region_dict[self.obj_locations[key1]]]
#                         ]
#                     continue
#                 key2 = self.poses[j]
#                 key = (key1, key2)
#                 self.dependency_set_pruning_search(key)

#     def dependency_set_pruning_search(self, key):
#         self.path_dict[key] = []
#         self.dependency_dict[key] = []  # key:obj,obj, value: a list of dependency set
#         vertex2node_dict = {}
#         for region in self.region_dict.values():
#             vertex2node_dict[region] = []
#         node_dependency_set_dict = {}  # the dictionary for the dependency set of each node in the search tree
#         nodes = {}
#         parents = {}
#         nodes[1] = self.region_dict[self.obj_locations[key[0]]]
#         node_dependency_set_dict[1] = self.get_dependency_set_from_index(nodes[1])
#         vertex2node_dict[self.region_dict[self.obj_locations[key[0]]]].append(1)
#         queue = [1]
#         while len(queue) > 0:
#             old_node = queue.pop()
#             if nodes[old_node] in self.LL:  # if it has neighbors
#                 for pose in self.LL[nodes[old_node]]:
#                     current_dependency_set = node_dependency_set_dict[old_node].union(
#                         self.get_dependency_set_from_index(pose)
#                     )
#                     Abandoned = False
#                     remove_list = []
#                     for n in vertex2node_dict[pose]:
#                         if current_dependency_set.issuperset(node_dependency_set_dict[n]):
#                             Abandoned = True
#                             break
#                         if node_dependency_set_dict[n].issuperset(current_dependency_set):
#                             remove_list.append(n)
#                     for n in remove_list:
#                         vertex2node_dict[pose].remove(n)
#                     if not Abandoned:
#                         node = len(nodes) + 1
#                         nodes[node] = pose
#                         parents[node] = old_node
#                         vertex2node_dict[pose].append(node)
#                         queue.append(node)
#                         node_dependency_set_dict[node] = current_dependency_set

#         goal_nodes = vertex2node_dict[self.region_dict[self.obj_locations[key[1]]]]

#         for node in goal_nodes:
#             current_node = node
#             path = []
#             while current_node in parents:  # while it is not the root(start pose).
#                 path.append(nodes[current_node])
#                 current_node = parents[current_node]
#             path.append(nodes[current_node])
#             # In this model we should keep the start and goal poses in the dependency graph, because they could be occupied
#             # node_dependency_set_dict[node] = node_dependency_set_dict[node].difference({key[0], key[1]})
#             self.path_dict[key].append(list(reversed(path)))
#             self.dependency_dict[key].append(node_dependency_set_dict[node])

#         # print("parents", parents)
#         # print("goal", goal_nodes)
#         # print("nodes", nodes)
#         # print("node_dependency_set_dict")
#         # print(node_dependency_set_dict)
#         # print("vertex2node_dict")
#         # print(vertex2node_dict)

#     def get_path_dependency_set(self, node, parents, nodes, key):
#         current_node = node
#         path = []
#         dependency_set = set()
#         while current_node in parents:  # while it is not the root(start pose).
#             path.append(nodes[current_node])
#             dependency_set = dependency_set.union(self.get_dependency_set_from_index(nodes[current_node]))
#             current_node = parents[current_node]
#         path.append(current_node)
#         dependency_set = dependency_set.union(self.get_dependency_set_from_index(nodes[current_node]))
#         dependency_set = dependency_set.difference({2 * key, 2 * key + 1})
#         self.path_dict[key].append(list(reversed(path)))
#         self.dependency_dict[key].append(dependency_set)

#     def get_dependency_set_from_index(self, index):
#         for key, value in self.region_dict.items():
#             if value == index:
#                 region_tuple = key
#                 break
#         dependency_set = set()
#         for i in region_tuple:
#             value = -1
#             try:
#                 value = int(i)
#             except ValueError:
#                 pass  # it was a string, not an int.
#             if value >= -0.5:
#                 dependency_set = dependency_set.union({value})
#         return dependency_set


# DFS_rec not considering buffers where the start/goal poses are not necessarily 2*i/2*i+1
class DFS_Rec_for_Monotone_General(object):
    def __init__(self, start_poses, goal_poses, dependency_dict, path_dict, object_locations, linked_list, region_dict):
        self.object_ordering = []
        self.path_selection_dict = {}
        self.obstacle_lst = []
        self.start_poses = {}
        self.goal_poses = {}
        self.isMonotone = False
        for i in start_poses.keys():
            if start_poses[i] == goal_poses[i]:
                self.obstacle_lst.append(start_poses[i])
            else:
                self.start_poses[i] = start_poses[i]
                self.goal_poses[i] = goal_poses[i]
        self.n = len(self.start_poses)

        self.dependency_dict = copy.deepcopy(dependency_dict)
        self.path_dict = copy.deepcopy(path_dict)
        self.obj_locations = copy.deepcopy(object_locations)
        self.linked_list = copy.deepcopy(linked_list)
        self.region_dict = copy.deepcopy(region_dict)
        self.dynamic_programming()

    def dynamic_programming(self):
        self.parent = {}
        self.path_option = {}
        self.explored = {}
        self.queue = [0]
        self.explored[0] = True
        # it is a stack when pop(-1)
        old_node = self.queue.pop(-1)
        # Recursion
        self.DFS_rec(old_node)

        task_index = 0
        for i in self.start_poses.keys():
            task_index += (1 << i)
        if task_index in self.path_option:
            current_task = task_index
            object_ordering = []
            while current_task in self.parent:
                parent_task = self.parent[current_task]
                last_object = int(log(current_task - parent_task, 2))
                self.path_selection_dict[last_object] = self.path_option[current_task]
                object_ordering.append(last_object)

                current_task = parent_task
            self.object_ordering = list(reversed(object_ordering))
            print("DFS_REC_MONOTONE", self.object_ordering)
            self.isMonotone = True
            return True
        else:
            print("Non-monotone")
            # exit(0)
            self.isMonotone = False
            return False
            # print(MISTAKE)

    def DFS_rec(self, old_node):
        FLAG = False  # Flag = True iff we find a solution to the rearrangement problem
        for next_object in self.next_object(old_node):
            new_node = old_node + (1 << next_object)
            if new_node in self.explored:
                continue

            # Detect which poses are occupied
            occupied_poses = copy.deepcopy(self.obstacle_lst)
            for i in self.start_poses.keys():
                if i == next_object:
                    continue
                elif ((old_node >> i) % 2):
                    occupied_poses.append(self.goal_poses[i])
                else:
                    occupied_poses.append(self.start_poses[i])

            path_index = self.transformation(occupied_poses, next_object)
            if path_index >= 0:
                self.path_option[new_node] = path_index
                self.parent[new_node] = old_node
                self.queue.append(new_node)
                self.explored[new_node] = True
                complete_node = 0
                for i in self.start_poses.keys():
                    complete_node += (1 << i)
                if new_node == complete_node:
                    return True
                FLAG = self.DFS_rec(new_node)
                if FLAG:
                    break
        return FLAG

    def next_object(self, index):
        for i in self.start_poses.keys():
            if ((index >> i) % 2):  # it has moved
                pass
            else:  # it is at the start pose
                yield i

    def generate_task_index(self, obj_set):
        task_index = 0
        for obj in obj_set:
            task_index += 2**obj
        return task_index

    def transformation(self, occupied_poses, obj):
        start = self.start_poses[obj]
        goal = self.goal_poses[obj]
        # dependency_dict_key = (min(start, goal), max(start, goal))
        dependency_dict_key = tuple(sorted([start, goal]))
        if dependency_dict_key not in self.dependency_dict:
            self.dependency_dict[dependency_dict_key] = []
            self.path_dict[dependency_dict_key] = []
        for path_index in range(len(self.dependency_dict[dependency_dict_key])):
            path = self.dependency_dict[dependency_dict_key][path_index]
            OCCUPIED = False
            for pose in path:
                if pose in occupied_poses:
                    OCCUPIED = True
                    break
            if not OCCUPIED:
                return path_index
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
            # print("Not accessable")
            return -1
        if (self.region_dict[self.obj_locations[start]] not in Available_Regions):
            # print("Not accessable")
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
        while (len(queue) > 0) and (not Found):
            # stack(-1) for DFS and queue(0) for BFS
            old_node = queue.pop(-1)
            if old_node in self.linked_list:
                for region in self.linked_list[old_node]:
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
            if dependency_dict_key[0] == start:
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
        for pose in region_tuple:
            try:
                value = int(pose)
            except ValueError:
                if (pose[0] in "SGB"):  # it was a string, not an int.
                    dependency_set.add(pose)
        return dependency_set


# DFS_rec non monotone where the start/goal poses are not necessarily 2*i/2*i+1
class DFS_Rec_for_Non_Monotone_General(object):
    def __init__(
        self, start_poses, goal_poses, dependency_dict, path_dict, object_locations, linked_list, region_dict,
        obj_buffer_dict
    ):
        ###### output   ############
        self.parent = {}
        self.path_option = {}
        self.mutation_nodes = []
        ######
        self.object_ordering = []
        self.path_selection_dict = {}
        self.obstacle_lst = []
        self.start_poses = {}
        self.goal_poses = {}
        for i in start_poses.keys():
            if start_poses[i] == goal_poses[i]:
                self.obstacle_lst.append(start_poses[i])
            else:
                self.start_poses[i] = start_poses[i]
                self.goal_poses[i] = goal_poses[i]
        self.n = len(start_poses)
        self.b = len(obj_buffer_dict)
        self.dependency_dict = copy.deepcopy(dependency_dict)
        self.path_dict = copy.deepcopy(path_dict)
        self.obj_locations = copy.deepcopy(object_locations)
        self.linked_list = copy.deepcopy(linked_list)
        self.region_dict = copy.deepcopy(region_dict)
        self.obj_buffer_dict = copy.deepcopy(obj_buffer_dict)
        self.buffer_objects = []
        for value in self.obj_buffer_dict.values():
            self.buffer_objects.append(value[0])
        self.dynamic_programming()

    def dynamic_programming(self):
        complete_index = 0
        for i in self.start_poses.keys():
            complete_index += (1 << i)
        for value in self.obj_buffer_dict.values():
            complete_index += (1 << value[0])
        self.explored = {}
        self.explored[0] = True
        # Recursion
        self.DFS_rec(0)

        task_index = complete_index
        if task_index in self.path_option:
            current_task = task_index
            object_ordering = []
            while current_task in self.parent:
                parent_task = self.parent[current_task]
                last_object = int(log(current_task - parent_task, 2))
                self.path_selection_dict[last_object] = self.path_option[current_task]
                if last_object in self.buffer_objects:
                    for key in self.obj_buffer_dict.keys():
                        if self.obj_buffer_dict[key][0] == last_object:
                            real_object = key
                            break
                    object_ordering.append(real_object)
                else:
                    object_ordering.append(last_object)

                current_task = parent_task
            self.object_ordering = list(reversed(object_ordering))
            return True
        else:
            if (len(self.obj_buffer_dict) > 0):
                ###### pick out nodes with mutations ######
                mutation_obj = self.obj_buffer_dict.keys()[0]
                for node in self.parent.keys():
                    if (((node >> mutation_obj) % 2) and (not ((node >> self.obj_buffer_dict[mutation_obj][0]) % 2))):
                        if (not ((self.parent[node] >> mutation_obj) % 2)):  # first mutation node in the branch
                            self.mutation_nodes.append(node)
                # print("mutation")
                # print(self.mutation_nodes)
            # print("Non-monotone")
            # exit(0)
            return False
            # print(MISTAKE)

    def DFS_rec(self, old_node):
        FLAG = False
        for next_object in self.next_object(old_node):
            new_node = old_node + (1 << next_object)
            if new_node in self.explored:
                continue

            # Detect which poses are occupied
            occupied_poses = copy.deepcopy(self.obstacle_lst)
            for i in self.start_poses.keys():
                if i == next_object:
                    continue
                elif i in self.obj_buffer_dict.keys():  # objects using buffers
                    if self.obj_buffer_dict[i][0] == next_object:
                        continue
                    elif ((old_node >> (self.obj_buffer_dict[i][0])) % 2):  # has been at the goal pose
                        occupied_poses.append(self.goal_poses[i])
                    elif ((old_node >> (i)) % 2):  # at the buffer
                        occupied_poses.append(self.obj_buffer_dict[i][1])
                    else:  # at the start pose
                        occupied_poses.append(self.start_poses[i])
                else:
                    if ((old_node >> i) % 2):
                        occupied_poses.append(self.goal_poses[i])
                    else:
                        occupied_poses.append(self.start_poses[i])

            path_index = self.transformation(occupied_poses, next_object)
            if path_index >= 0:
                self.path_option[new_node] = path_index
                self.parent[new_node] = old_node
                self.explored[new_node] = True
                complete_node = 0
                for i in self.start_poses.keys():
                    complete_node += (1 << i)
                for value in self.obj_buffer_dict.values():
                    complete_node += (1 << value[0])
                if new_node == complete_node:
                    return True
                FLAG = self.DFS_rec(new_node)
                if FLAG:
                    break
        return FLAG

    def next_object(self, index):
        for i in self.start_poses.keys():
            if ((index >> i) % 2):  # it has moved
                # it is at the buffer
                if (i in self.obj_buffer_dict) and (not ((index >> (self.obj_buffer_dict[i][0])) % 2)):
                    yield self.obj_buffer_dict[i][0]
            else:  # it is at the start pose
                yield i

    def generate_task_index(self, obj_set):
        task_index = 0
        for obj in obj_set:
            task_index += 2**obj
        return task_index

    def transformation(self, occupied_poses, obj):

        # print(self.obj_buffer_dict)
        if obj < self.n:
            # start = 2 * obj
            start = 'S' + str(obj)
            if obj in self.obj_buffer_dict:
                goal = self.obj_buffer_dict[obj][1]
            else:
                # goal = 2 * obj + 1
                goal = 'G' + str(obj)
        else:
            for key in self.obj_buffer_dict.keys():
                if self.obj_buffer_dict[key][0] == obj:
                    real_object = key
                    break
            start = self.obj_buffer_dict[real_object][1]
            # goal = 2 * real_object + 1
            goal = 'G' + str(real_object)
        # dependency_dict_key = (min(start, goal), max(start, goal))
        dependency_dict_key = tuple(sorted([start, goal]))
        if dependency_dict_key not in self.dependency_dict:
            self.dependency_dict[dependency_dict_key] = []
            self.path_dict[dependency_dict_key] = []
        for path_index in range(len(self.dependency_dict[dependency_dict_key])):
            path = self.dependency_dict[dependency_dict_key][path_index]
            OCCUPIED = False
            for pose in path:
                if pose in occupied_poses:
                    OCCUPIED = True
                    break
            if not OCCUPIED:
                return path_index
        Available_Regions = []
        for region in self.region_dict.keys():
            OCCUPIED = False
            for pose in region:
                if pose in occupied_poses:
                    OCCUPIED = True
                    break
            if not OCCUPIED:
                Available_Regions.append(self.region_dict[region])
        # print("Test: ", goal, self.obj_locations, self.region_dict)
        if (self.region_dict[self.obj_locations[goal]] not in Available_Regions):
            # print("Not accessable")
            return -1
        if (self.region_dict[self.obj_locations[start]] not in Available_Regions):
            # print("Not accessable")
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
        while (len(queue) > 0) and (not Found):
            # stack(-1) for DFS and queue(0) for BFS
            old_node = queue.pop(-1)
            if old_node in self.linked_list:
                for region in self.linked_list[old_node]:
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
            if dependency_dict_key[0] == start:
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
        for pose in region_tuple:
            try:
                value = int(pose)
            except ValueError:
                if (pose[0] in "SGB"):  # it was a string, not an int.
                    dependency_set.add(pose)
        return dependency_set


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
        # self.dependency_dict_conversion()

    def enumerate_cases(self):
        # enumerate possible cases
        FOUND = False
        for obj_num in range(self.n + 1):  # num of objects that need buffers
            print("number of objects that use buffers", obj_num)
            for obj_set in combinations(self.start_poses.keys(), obj_num):  # which objs need buffers
                # print(obj_set)
                for buffer_set in product(sorted(self.obj_locations.keys()), repeat=obj_num):  # which poses are buffers
                    obj_buffer_dict = {}
                    Degrade = False  # when an object uses its own start or goal pose as a buffer, Degrade = True.
                    for index in xrange(len(obj_set)):
                        obj = obj_set[index]
                        buffer = buffer_set[index]
                        if (buffer == self.start_poses[obj]) or (buffer == self.goal_poses[obj]):
                            Degrade = True
                            break
                        obj_buffer_dict[obj] = (self.n + index, buffer)
                    if Degrade:
                        continue
                    # print(self.start_poses)
                    # print(self.goal_poses)
                    # print(obj_buffer_dict)
                    DFS = DFS_Rec_for_Non_Monotone_General(
                        self.start_poses,
                        self.goal_poses,
                        self.dependency_dict,
                        self.path_dict,
                        self.obj_locations,
                        self.LL,
                        self.region_dict,
                        obj_buffer_dict,
                    )
                    self.dependency_dict = copy.deepcopy(DFS.dependency_dict)
                    self.path_dict = copy.deepcopy(DFS.path_dict)
                    if len(DFS.object_ordering) > 0:
                        print("Find a solution!")
                        FOUND = True
                        print("obj_buffer_dict", obj_buffer_dict)
                        print("DFS.object_ordering", DFS.object_ordering)
                        self.object_ordering = DFS.object_ordering
                        break
                if FOUND:
                    break
            if FOUND:
                break

    def linked_list_conversion(self, graph):
        # print("graph")
        # print(graph)
        self.region_dict = {}  # (1,2,'a'): 0
        self.LL = {}  # 0:[1,2,3]
        for key in graph:
            index = len(self.region_dict.keys())
            self.region_dict[key] = index
            self.LL[index] = []
        for key in graph:
            for v in graph[key]:
                self.LL[self.region_dict[key]].append(self.region_dict[v])
        # print("LL")
        # print(self.LL)
        # print("region dict")
        # print(self.region_dict)

    # def dependency_dict_conversion(self):
    #     for key in self.dependency_dict.keys():
    #         number_set_list = self.dependency_dict[key]
    #         pose_set_list = []
    #         for number_set in number_set_list:
    #             pose_set = set()
    #             for number in number_set:
    #                 pose_set = pose_set.union({(number // 2, number % 2)})
    #             pose_set_list.append(pose_set)
    #         self.dependency_dict[key] = pose_set_list


def linked_list_conversion(graph):
    # print("graph")
    # print(graph)
    region_dict = {}  # (1,2,'a'): 0
    LL = {}  # 0:[1,2,3]
    for key in graph:
        index = len(region_dict.keys())
        region_dict[key] = index
        LL[index] = []
    for key in graph:
        for v in graph[key]:
            LL[region_dict[key]].append(region_dict[v])
    # print(region_dict)
    return region_dict, LL
    # print("LL")
    # print(self.LL)
    # print("region dict")
    # print(self.region_dict)


################################################################################################

# path_dict = {}
# path_dict[1] = [
#     [(4,1), (2,0)],
#     [(4,0), (2,0)],
#     [(3,0), (2,0)]
# ]

# path_dict[2] = [
#     [(1,1), (3,0)],
#     [(1,1), (4,1), (1,0)],
#     [(1,1), (4,0), (1,0)]
# ]

# path_dict[3] = [
#     [(1,0)],
#     [(2,1)]
# ]

# path_dict[4] = [
#     []
# ]

if __name__ == "__main__":
    space = None
    numObjs = 5
    rad = 50
    height = 1000
    width = 1000
    if len(sys.argv) > 1:
        if sys.argv[1].isdigit():
            numObjs = int(sys.argv[1])
        else:
            space = loadEnv(sys.argv[1])

    if len(sys.argv) > 2:
        if space is None:
            rad = int(sys.argv[2])
        else:
            numObjs = int(sys.argv[2])

    if len(sys.argv) > 3:
        height = int(sys.argv[3])

    if len(sys.argv) > 4:
        width = int(sys.argv[4])

    if space is None:
        space = DiskCSpace(rad, {}, [], height, width)

    if len(space.poseMap) == 0:
        genPoses(numObjs, space)

    space.regionGraph()
    genBuffers(num_buffers, space, space.poseMap.keys(), 'random', 4)
    # genBuffers(num_buffers, space, space.poseMap.keys(), 'greedy_free')
    # genBuffers(num_buffers, space, space.poseMap.keys(), 'boundary_free')
    # genBuffers(num_buffers, space, filter(lambda x: x[0] == 'S', space.poseMap.keys()), 'object_feasible', 0, [1, 2, 0, 3, 4])
    space.regionGraph()

    outfile = sys.stderr
    if len(sys.argv) > 5:
        outfile = open(sys.argv[5], 'w')

    print(
        """DiskCSpace(
    rad={},
    height={},
    width={},""".format(
            rad,
            height,
            width,
        ),
        file=outfile,
    )
    print('    obstacles=[', file=outfile)
    for x in space.obstacles:
        print('        ', x, ',', sep='', file=outfile)
    print('    ],', file=outfile)

    print('    poseMap={', file=outfile)
    for k, v in space.poseMap.items():
        print("        '", k, "': ", v, ',', sep='', file=outfile)
    print('    },\n)', file=outfile)

    if outfile is not sys.stderr:
        outfile.close()
    # print(space.RGAdj.keys())

    # DGs = DG_Space(path_dict)
    # print(DGs.DGs)
    # IP = feekback_arc_ILP(path_dict)
    EXP = Experiments()
    set_max_memory(1.3 * 2**(34))  #2**34=16G

    EXP.single_instance(space)
