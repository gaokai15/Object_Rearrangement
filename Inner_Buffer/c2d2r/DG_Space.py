import os
import sys
import copy
from collections import OrderedDict
from cgraph import genCGraph, genDenseCGraph, loadCGraph, drawMotions, animatedMotions
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

import resource
my_path = os.path.abspath(os.path.dirname(__file__))

def set_max_memory(MAX):
    soft, hard = resource.getrlimit(resource.RLIMIT_AS)
    resource.setrlimit(resource.RLIMIT_AS, (MAX, hard))

class Experiments(object):
    def single_instance(self, numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, example_index):
        # OBJ_NUM = numObjs
        # RECORD is False when I'm debugging and don't want to rewrite the data
        # RECORD = False
        timeout = 10
        graph, paths, objects, color_pool, points, objectShape, object_locations = genDenseCGraph(
            numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, example_index
        )
        while (graph == False) & timeout >0:
            graph, paths, objects, color_pool, points, objectShape, object_locations = genDenseCGraph(
                numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, example_index
            )
            timeout -= 1

        if graph is paths is objects is False:
            print "Connectivity graph generation fails for 10 times..."
            print "Quit."
            return -1
        gpd = Dense_Path_Generation(graph, object_locations)
        keys = sorted(gpd.path_dict.keys())
        new_key = -1
        for key in keys:
            if key[0] != new_key:
                print key
                new_key = key[0]
            for path in gpd.path_dict[key]:
                print path
        # if RECORD:
        #     with open(os.path.join(my_path, "settings/W%s_H%s_n%s_iter%s_0301.pkl" %
        #                            (int(W_X / scaler), int(W_Y / scaler), OBJ_NUM, iter)),
        #               'wb') as output:
        #         pickle.dump((OR.start_pose, OR.goal_pose), output, pickle.HIGHEST_PROTOCOL)
        ### print dependency set and path set
        # print "Dependency dict(key: obj index; value: list of paths as dependencies)"
        # for key, value in gpd.dependency_dict.items():
        #     print key
        #     print value
        # for key, value in gpd.path_dict.items():
        #     print key
        #     for path in value:
        #         labelled_path = []
        #         for index in path:
        #             for key, value in gpd.region_dict.items():
        #                 if value == index:
        #                     labelled_path.append(key)
        #                     break
        #         print labelled_path

        # DGs = DG_Space(gpd.dependency_dict)
        # if RECORD:
        #     with open(os.path.join(my_path, "DG/W%s_H%s_n%s_iter%s_0301.pkl" %
        #                            (int(WIDTH), int(HEIGHT), OBJ_NUM, iter)), 'wb') as output:
        #         pickle.dump(DGs.DGs, output, pickle.HIGHEST_PROTOCOL)
        IP = Inner_Buffer_IQP(gpd.dependency_dict, len(objects)//2)
        print "ILP result(the smallest size of FAS):", IP.optimum
        # print "DGs(key: path indices; value: [total_num_constr, num_edges, FAS size])"
        num_of_actions, buffer_selection, path_selection, object_ordering = IP.optimum
        # print ind_opt, DGs.DGs[ind_opt]
        path_opts = gpd.path_dict

        new_paths = {}
        for r1, r2 in paths.keys():
            new_paths[(gpd.region_dict[r1], gpd.region_dict[r2])] = copy.deepcopy(paths[(r1, r2)])

        if display:
            rpaths = self.drawSolution(
                HEIGHT, WIDTH, numObjs, RAD, new_paths, path_opts, path_selection, buffer_selection, objects, color_pool, points,
                example_index, saveimage
            )

            animatedMotions(
                HEIGHT, WIDTH, numObjs, RAD, rpaths, color_pool, points, object_ordering, example_index, objectShape
            )

        return num_of_actions

    def multi_instances(self, numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, example_index):
        Trials = 10
        OBJ_NUM = [13]
        Radius = range(10, 25, 7)
        # Trials = 1
        # OBJ_NUM = [3]
        # Radius = [10]
        data_collection = []
        for numObjs_edited in OBJ_NUM:
            for RAD_edited in Radius:
                for i in range(Trials):
                    print "************ current **********"
                    print numObjs_edited, RAD_edited, i
                    timeout = 10
                    graph = False
                    try:
                        graph, paths, objects, color_pool, points, objectShape, object_locations = genDenseCGraph(
                            numObjs_edited, RAD_edited, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, example_index
                        )
                    except Exception:
                        pass
                    while (graph == False) & timeout >0:
                        try:
                            graph, paths, objects, color_pool, points, objectShape, object_locations = genDenseCGraph(
                                numObjs_edited, RAD_edited, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, example_index
                            )
                        except Exception:
                            pass
                        timeout -= 1

                    if graph is False:
                        print "Connectivity graph generation fails for 10 times..."
                        print "Quit."
                        continue
                    gpd = Dense_Path_Generation(graph, object_locations)
                    IP = Inner_Buffer_IQP(gpd.dependency_dict, len(objects)//2)
                    try:
                        print "ILP result(the smallest size of FAS):", IP.optimum
                        # print "DGs(key: path indices; value: [total_num_constr, num_edges, FAS size])"
                        num_of_actions, buffer_selection, path_selection, object_ordering = IP.optimum
                        # print ind_opt, DGs.DGs[ind_opt]
                        path_opts = gpd.path_dict
                    except Exception:
                        pass

                    print "data", [numObjs_edited, RAD_edited, i] + IP.data

                    data_collection.append([numObjs_edited, RAD_edited, i] + IP.data)

                    new_paths = {}
                    for r1, r2 in paths.keys():
                        new_paths[(gpd.region_dict[r1], gpd.region_dict[r2])] = copy.deepcopy(paths[(r1, r2)])

                    if display:
                        rpaths = self.drawSolution(
                            HEIGHT, WIDTH, numObjs_edited, RAD_edited, new_paths, path_opts, path_selection, buffer_selection, objects, color_pool, points,
                            example_index, saveimage
                        )

                        animatedMotions(
                            HEIGHT, WIDTH, numObjs_edited, RAD_edited, rpaths, color_pool, points, object_ordering, example_index, objectShape
                        )
        with open(os.path.join(my_path, "Experiments13.pkl"), 'wb') as output:
            pickle.dump(data_collection, output, pickle.HIGHEST_PROTOCOL)
        print data_collection

    def load_instance(self, savefile, repath, display, displayMore):

        # scaler = 1000.0
        numObjs, RAD, HEIGHT, WIDTH, points, objects, graph, paths = loadCGraph(savefile, repath, display, displayMore)
        print "linked list", graph
        gpd = Generate_Path_Dictionary(graph)
        print "Dependency dict(key: obj index; value: list of paths as dependencies)"
        print gpd.dependency_dict
        # DGs = DG_Space(gpd.dependency_dict)
        IP = feekback_vertex_ILP(gpd.dependency_dict)
        # print "ILP result(the smallest size of FAS):", IP.optimum
        # print "DGs(key: path indices; value: [total_num_constr, num_edges, FAS size])"
        # print DGs.DGs
        opt, ind_opt = IP.optimum
        # print ind_opt, DGs.DGs[ind_opt]
        path_opts = gpd.dependency_dict
        # if display:
        #     self.drawSolution(HEIGHT, WIDTH, paths, path_opts, ind_opt, objects)

    def drawSolution(
        self, HEIGHT, WIDTH, numObjs, RAD, paths, path_opts, ind_opt, buffer_selection, objects, color_pool, points, example_index,
        saveimage
    ):
        rpaths = OrderedDict()
        for pose in range(2*numObjs):
            iopt = ind_opt[pose//2 + (pose%2)*numObjs]
            if pose%2:
                start = buffer_selection[pose//2]
                goal = pose
            else:
                start = pose
                goal = buffer_selection[pose//2]

            if (start <= goal):
                dpath = path_opts[(start, goal)][iopt]
            else:
                dpath = list(reversed(path_opts[(goal, start)][iopt]))
            # deps = {2 * (x[0]) + x[1] for x in dpath}
            # deps = set(dpath)
            # print "dpath", dpath
            # print(deps)

            rpath = [points[start]]
            for i in range(len(dpath) - 2):
                region1 = dpath[i]
                region2 = dpath[i + 1]
                if (region1, region2) in paths.keys():
                    rpath += paths[(region1, region2)][:-1]
                elif (region2, region1) in paths.keys():
                    rpath += list(reversed(paths[(region2, region1)]))[:-1]
                elif region1 == region2:
                    rpath.append(points[start])
                else:
                    print "invalid path"
                    exit()
            if len(dpath) >= 2:
                region1 = dpath[-2]
                region2 = dpath[-1]
                if (region1, region2) in paths.keys():
                    rpath += paths[(region1, region2)]
                elif (region2, region1) in paths.keys():
                    rpath += list(reversed(paths[(region2, region1)]))
                elif region1 == region2:
                    rpath.append(points[start])
                else:
                    print "invalid path"
                    exit()
            rpath.append(points[goal])

            rpaths[(pose, buffer_selection[pose//2])] = rpath
        print(rpaths.keys())
        # drawMotions(HEIGHT, WIDTH, numObjs, RAD, rpaths, color_pool, points, example_index, saveimage, objects)

        return rpaths

    # try to create instances in different densities and object numbers.
    def density(self):
        OBJ_NUM = 5
        # RECORD is False when I'm debugging and don't want to rewrite the data
        RECORD = False
        Iteration_time = 10

        DG_num_matrix = np.zeros([5, 5])
        min_feedback_matrix = np.zeros([5, 5])
        scaler = 1000.0
        for W_X in range(3 * scaler, 8 * scaler, scaler):
            for W_Y in range(3 * scaler, 8 * scaler, scaler):
                for iter in range(Iteration_time):
                    graph, _ = genCGraph(OBJ_NUM, 0.3 * scaler, W_X, W_Y, False, False, False)
                    gpd = Generate_Path_Dictionary(graph)
                    # if RECORD:
                    #     with open(os.path.join(
                    #             my_path, "settings/W%s_H%s_n%s_iter%s_0301.pkl" %
                    #         (int(W_X / scaler), int(W_Y / scaler), OBJ_NUM, iter)),
                    #               'wb') as output:
                    #         pickle.dump(
                    #             (OR.start_pose, OR.goal_pose), output, pickle.HIGHEST_PROTOCOL
                    #         )
                    DGs = DG_Space(gpd.dependency_dict)
                    DG_num_matrix[int(W_X / scaler) - 3, int(W_Y / scaler) - 3] += len(DGs.DGs.keys())
                    if RECORD:
                        with open(os.path.join(my_path, "DG/W%s_H%s_n%s_iter%s_0301.pkl" %
                                               (int(W_X / scaler), int(W_Y / scaler), OBJ_NUM, iter)), 'wb') as output:
                            pickle.dump(DGs.DGs, output, pickle.HIGHEST_PROTOCOL)
                    IP = feekback_vertex_ILP(gpd.dependency_dict)
                    # IP = feekback_arc_ILP(gpd.dependency_dict)
                    min_feedback_matrix[int(W_X / scaler) - 3, int(W_Y / scaler) - 3] += IP.optimum
                DG_num_matrix[int(W_X / scaler) - 3, int(W_Y / scaler) - 3] /= Iteration_time
                min_feedback_matrix[int(W_X / scaler) - 3, int(W_Y / scaler) - 3] /= Iteration_time

        fig = plt.figure()
        ax = fig.gca(projection='3d')

        # Make data.
        X = np.arange(3, 8, 1)
        Y = np.arange(3, 8, 1)
        X, Y = np.meshgrid(X, Y)

        # Plot the surface.
        surf = ax.plot_surface(X, Y, DG_num_matrix, cmap=cm.coolwarm, linewidth=0, antialiased=False)

        # Add a color bar which maps values to colors.
        fig.colorbar(surf, shrink=0.5, aspect=5)

        plt.savefig(my_path + "/pictures/DG_num_n%s.png" % OBJ_NUM)

        fig = plt.figure()
        ax = fig.gca(projection='3d')

        surf = ax.plot_surface(X, Y, min_feedback_matrix, cmap=cm.coolwarm, linewidth=0, antialiased=False)

        # Add a color bar which maps values to colors.
        fig.colorbar(surf, shrink=0.5, aspect=5)

        plt.savefig(my_path + "/pictures/min_feedback_n%s.png" % OBJ_NUM)

    # In this experiment, I'm trying to see the relationship between the number
    # of edges and DG quality.
    def edge_and_DG(self):
        Iteration_time = 10
        OBJ_NUM = 5
        # Points = []
        edge_arcs_dict = {}
        M = np.zeros([21, 21])
        count_DG = 0
        for W_X in range(3, 8):
            for W_Y in range(3, 8):
                for iter in range(Iteration_time):
                    with open(os.path.join(my_path, "DG/W%s_H%s_n%s_iter%s_0301.pkl" % (W_X, W_Y, OBJ_NUM, iter)),
                              'rb') as input:
                        DGs = pickle.load(input)
                    for stat in DGs.values():
                        count_DG += 1
                        M[int(stat[1]), int(stat[2])] += 1
                        if stat[2] not in edge_arcs_dict:
                            edge_arcs_dict[stat[2]] = []
                        edge_arcs_dict[stat[2]].append(stat[1])
        print "Num DGs", count_DG

        edge_arcs_avg_list = []
        for key in range(0, 11):
            if key in edge_arcs_dict:
                edge_arcs_avg_list.append(np.average(edge_arcs_dict[key]))
            else:
                edge_arcs_avg_list.append(0)

        edge_arcs_std_list = []
        for key in range(0, 11):
            if key in edge_arcs_dict:
                edge_arcs_std_list.append(np.std(edge_arcs_dict[key]))
            else:
                edge_arcs_std_list.append(0)

        # A = np.zeros([1,11])
        # for i in range(11):
        #     for j in range(21):
        #         A[0,i] += M[j,i]

        # width = 0.2
        fig, ax = plt.subplots()
        # ax2 = ax.twinx()

        # p0 = ax.bar(range(0, 11), edge_arcs_avg_list, width, label='edge_num_avg')
        # p1 = ax.bar(
        #     range(0, 11),
        #     edge_arcs_std_list,
        #     width,
        #     bottom=edge_arcs_avg_list,
        #     label='edge_num_std'
        # )

        # p0 = ax.bar([x for x in range(11)], A[0,:].T, width)

        # fig = plt.figure()
        # ax = fig.gca(projection='3d')

        # X = np.arange(0,21,1)
        # Y = np.arange(0,21,1)

        # X, Y = np.meshgrid(X, Y)

        # print M

        # surf = ax.plot_surface(X,Y,M, cmap=cm.coolwarm, linewidth=0, antialiased=False)

        # # Add a color bar which maps values to colors.
        # fig.colorbar(surf, shrink=0.5, aspect=5)
        plt.legend()
        plt.xlabel("Minimum Feedback Arcs")
        plt.ylabel("Corresponding Edges")
        plt.savefig(my_path + "/pictures/edge_arcs_n%s.png" % OBJ_NUM)
        plt.show()
        # plt.scatter(
        #     [Points[i][0] for i in xrange(len(Points))],
        #     [Points[i][1] for i in xrange(len(Points))]
        # )
        # plt.savefig(my_path + "/pictures/edge_arcs_n%s.png"%OBJ_NUM)


class DG_Space(object):
    def __init__(self, path_dict):
        self.path_dict = path_dict
        self.DG_space_construction()
        # print("Finish DG space construction.\n")

    def DG_space_construction(self):
        self.DGs = {}
        objs = self.path_dict.keys()
        path_choices = [len(self.path_dict[obj]) - 1 for obj in objs]
        for path_set in self.choice_generator(path_choices):
            self.DGs[tuple(path_set)] = self.DG_construction(path_set)
        # print("objs: ", objs)
        # print("path choices: ", path_choices)
        # print("All dependency graphs: ")
        # self.printDGs()

    def printDGs(self):
        for path_comb in self.DGs:
            print(str(path_comb) + ": " + str(self.DGs[path_comb]))

    def choice_generator(self, path_choices):
        n = len(path_choices)
        choice = [0 for i in range(n)]
        while 1:
            pointer = n - 1
            while (pointer > 0) and (choice[pointer] == path_choices[pointer]):
                pointer -= 1
            if (pointer == 0) and (choice[pointer] == path_choices[pointer]):
                yield path_choices
                break
            else:
                yield choice
                choice[pointer] += 1
                for i in range(pointer + 1, n):
                    choice[i] = 0

    def DG_construction(self, path_set):
        n = len(path_set)
        M = np.zeros([n, n])
        total_num_constr = 0
        objs = self.path_dict.keys()
        for i in range(n):
            key = objs[i]
            total_num_constr += len(self.path_dict[key][path_set[i]])
            for constr in self.path_dict[key][path_set[i]]:
                if constr[1] == 0:
                    M[i, objs.index(constr[0])] = 1
                else:
                    M[objs.index(constr[0]), i] = 1
        num_constr = np.count_nonzero(M)
        # num_feedback = self.count_feedback_arc(M)
        num_feedback, vertices_to_be_removed, final_order = self.count_feedback_vertex(M)
        # return [total_num_constr, num_constr, num_feedback]
        return [total_num_constr, num_constr, num_feedback, vertices_to_be_removed, final_order]

    def count_feedback_vertex(self, M):
        n = M.shape[0]
        m = gp.Model()
        ### The first step is to add the inner edges
        for i in range(n):
            M[i, i] = 1

        m.setParam('OutputFlag', 0)
        m.modelSense = GRB.MINIMIZE

        y = m.addVars(2 * n, 2 * n, vtype=GRB.BINARY)
        m.setObjective(
            sum(
                sum(M[k, i] * y[k, i + n] for k in range(i)) + M[i, i] * (1 - y[i, i + n]) +
                sum(M[l, i] * y[l, i + n] for l in range(i + 1, n)) for i in range(n)
            )
        )
        for i in range(2 * n):
            for j in range(i + 1, 2 * n):
                for k in range(j + 1, 2 * n):
                    m.addConstr(y[i, j] + y[j, k] + y[k, i] >= 1)
                    m.addConstr(y[i, j] + y[j, k] + y[k, i] <= 2)
        for i in range(2 * n):
            for j in range(i + 1, 2 * n):
                m.addConstr(y[i, j] + y[j, i] == 1)

        m.optimize()
        obj = m.getObjective()

        vertices_to_be_removed = []
        for i in range(n):
            for j in range(n):
                if (M[i, j] == 1.0):
                    if (i != j) and (y[i, j + n].x == 1.0):
                        vertices_to_be_removed.append((i, j))
                    if (i == j) and (y[i, j + n].x == 0.0):
                        vertices_to_be_removed.append((i, j))

        Y = np.zeros([n, 2 * n])
        for i in range(n):
            for j in range(2 * n):
                Y[i, j] = y[i, j].x
        obj_indexes = [i for i in range(n)]
        order_count = np.sum(Y, axis=1)
        final_order = [obj_index for _, obj_index in sorted(zip(order_count, obj_indexes), reverse=True)]

        return obj.getValue(), vertices_to_be_removed, final_order

    def count_feedback_arc(self, M):
        n = M.shape[0]
        m = gp.Model()

        m.setParam('OutputFlag', 0)
        m.modelSense = GRB.MINIMIZE

        y = m.addVars(n, n, vtype=GRB.BINARY)
        m.setObjective(
            sum(
                sum(M[k, j] * y[k, j]
                    for k in range(j)) + sum(M[l, j] * (1 - y[j, l])
                                             for l in range(j + 1, n))
                for j in range(n)
            )
        )
        for i in range(n):
            for j in range(i + 1, n):
                for k in range(j + 1, n):
                    m.addConstr(y[i, j] + y[j, k] - y[i, k] <= 1)
                    m.addConstr(-y[i, j] - y[j, k] + y[i, k] <= 0)

        m.optimize()
        obj = m.getObjective()
        return obj.getValue()


class feekback_vertex_ILP(object):
    def __init__(self, path_dict):
        self.path_dict = path_dict
        self.optimum = self.run_vertex_ILP()
        # self.optimum = self.run_arc_ILP()

    def run_vertex_ILP(self):
        MOST_PATH = 0
        for paths in self.path_dict.values():
            MOST_PATH = max(MOST_PATH, len(paths))
        # print "most paths", MOST_PATH
        n = len(self.path_dict)
        objs = self.path_dict.keys()
        dependency_dict = {}
        for i in range(n):
            key = objs[i]
            for path_index in range(len(self.path_dict[key])):
                for constr in self.path_dict[key][path_index]:
                    if constr[1] == 0:
                        if (i, objs.index(constr[0])) not in dependency_dict:
                            dependency_dict[(i, objs.index(constr[0]))] = []
                        dependency_dict[(i, objs.index(constr[0]))].append([key, path_index])
                    else:
                        if (objs.index(constr[0]), i) not in dependency_dict:
                            dependency_dict[(objs.index(constr[0]), i)] = []
                        dependency_dict[(objs.index(constr[0]), i)].append([key, path_index])

        m = gp.Model()

        m.setParam('OutputFlag', 0)
        m.modelSense = GRB.MINIMIZE

        ###### Minimum feedback vertex ILP formulation ######
        ### variables
        y = m.addVars(2 * n, 2 * n, vtype=GRB.BINARY)
        M = m.addVars(n, n, vtype=GRB.BINARY)
        x = m.addVars(n, MOST_PATH, vtype=GRB.BINARY)
        ### Objective function ###
        m.setObjective(
            sum(
                sum(M[k, i] * y[k, i + n] for k in range(i)) + M[i, i] * (1 - y[i, i + n]) +
                sum(M[l, i] * y[l, i + n] for l in range(i + 1, n)) for i in range(n)
            )
        )
        ### constraints ###
        for i in range(n):
            m.addConstr(M[i, i] == 1)

        for i in range(2 * n):
            for j in range(i + 1, 2 * n):
                for k in range(j + 1, 2 * n):
                    m.addConstr(y[i, j] + y[j, k] + y[k, i] >= 1)
                    m.addConstr(y[i, j] + y[j, k] + y[k, i] <= 2)
        for i in range(2 * n):
            for j in range(i + 1, 2 * n):
                m.addConstr(y[i, j] + y[j, i] == 1)

        for i in range(n):
            for j in range(len(self.path_dict[objs[i]]), MOST_PATH):
                m.addConstr(x[i, j] == 0)
        for i in range(n):
            m.addConstr(sum(x[i, u] for u in range(MOST_PATH)) == 1)
        m.addConstrs(
            M[j, k] <= sum(x[objs.index(i), u]
                           for (i, u) in dependency_dict[(j, k)])
            for (j, k) in dependency_dict.keys()
        )
        for (j, k) in dependency_dict.keys():
            for (i, u) in dependency_dict[(j, k)]:
                m.addConstr(M[j, k] >= x[objs.index(i), u])

        m.optimize()
        obj = m.getObjective()

        # print("The useful information below: ")

        ### figure out what path options are chosen for each object
        path_selection = []
        for i in range(n):
            for j in range(MOST_PATH):
                if x[i, j].x > 0.5:
                    path_selection.append(j)
        # print("path_selection: ", path_selection)

        ### figure out the vertices and constraints to be removed
        vertices_to_be_removed = []
        for i in range(n):
            for j in range(n):
                if (M[i, j].x == 1.0):
                    if (i != j) and (y[i, j + n].x == 1.0):
                        vertices_to_be_removed.append((i, j))
                    if (i == j) and (y[i, j + n].x == 0.0):
                        vertices_to_be_removed.append((i, j))

        ### display the dependency graph as a matrix C
        C = np.zeros([n, n])
        for i in range(n):
            for j in range(n):
                if M[i, j].x == 1.0:
                    if i != j:
                        C[i, j] = 1
        # print("The dependecy graph is: ")
        # print(C)

        Y = np.zeros([n, 2 * n])
        for i in range(n):
            for j in range(2 * n):
                Y[i, j] = y[i, j].x
        obj_indexes = [i for i in range(n)]
        order_count = np.sum(Y, axis=1)
        final_order = [obj_index for _, obj_index in sorted(zip(order_count, obj_indexes), reverse=True)]
        print("Final objects order: ")
        print(final_order)

        # print("order: ", y)
        # print("\n")
        # print("DG: " , M)
        # print("\n")
        # print("Number of vertex to be removed: ", obj.getValue())
        # print("Vertices to be removed: ", vertices_to_be_removed)

        return obj.getValue(), vertices_to_be_removed, tuple(path_selection), final_order

    def run_arc_ILP(self):
        MOST_PATH = 0
        for paths in self.path_dict.values():
            MOST_PATH = max(MOST_PATH, len(paths))
        print "most paths", MOST_PATH
        n = len(self.path_dict)
        objs = self.path_dict.keys()
        dependency_dict = {}
        for i in range(n):
            key = objs[i]
            for path_index in range(len(self.path_dict[key])):
                for constr in self.path_dict[key][path_index]:
                    if constr[1] == 0:
                        if (i, objs.index(constr[0])) not in dependency_dict:
                            dependency_dict[(i, objs.index(constr[0]))] = []
                        dependency_dict[(i, objs.index(constr[0]))].append([key, path_index])
                    else:
                        if (objs.index(constr[0]), i) not in dependency_dict:
                            dependency_dict[(objs.index(constr[0]), i)] = []
                        dependency_dict[(objs.index(constr[0]), i)].append([key, path_index])

        m = gp.Model()

        m.setParam('OutputFlag', 0)
        m.modelSense = GRB.MINIMIZE

        y = m.addVars(n, n, vtype=GRB.BINARY)
        M = m.addVars(n, n, vtype=GRB.BINARY)
        x = m.addVars(n, MOST_PATH, vtype=GRB.BINARY)
        m.setObjective(
            sum(
                sum(M[k, j] * y[k, j]
                    for k in range(j)) + sum(M[l, j] * (1 - y[j, l])
                                             for l in range(j + 1, n))
                for j in range(n)
            )
        )

        for i in range(n):
            for j in range(i + 1, n):
                for k in range(j + 1, n):
                    m.addConstr(y[i, j] + y[j, k] - y[i, k] <= 1)
                    m.addConstr(-y[i, j] - y[j, k] + y[i, k] <= 0)
        for i in range(n):
            for j in range(len(self.path_dict[objs[i]]), MOST_PATH):
                m.addConstr(x[i, j] == 0)
        for i in range(n):
            m.addConstr(sum(x[i, j] for j in range(MOST_PATH)) == 1)
        m.addConstrs(
            M[j, k] >= sum(x[objs.index(u), i]
                           for (u, i) in dependency_dict[(j, k)]) / (2 * MOST_PATH)
            for (j, k) in dependency_dict.keys()
        )

        m.optimize()
        obj = m.getObjective()
        DG_index = []
        for i in range(n):
            for j in range(MOST_PATH):
                if x[i, j].x > 0.5:
                    DG_index.append(j)
        print "DG_index", DG_index
        # print "path selection", x
        # print "order", y
        # print "DG", M
        return obj.getValue()

class Inner_Buffer_IQP(object):
    def __init__(self, path_dict, n):
        self.n = n
        self.path_dict = path_dict
        self.data = [True]
        start = time.time()
        try:
            self.optimum = self.run_single_buffer_IQP()
        except Exception:
            self.data[0] = False
            self.optimum = []
            print "Fail to solve the problem!"
        stop = time.time()
        self.data.append(stop-start)
    
    def run_single_buffer_IQP(self):
        MOST_PATH = 0
        Total_Path = 0
        for paths in self.path_dict.values():
            MOST_PATH = max(MOST_PATH, len(paths))
            Total_Path += len(paths)

        self.data.append(MOST_PATH)
        self.data.append(Total_Path)

        print "MOST PATH", MOST_PATH
        print "total paths", Total_Path
        m = gp.Model()

        m.Params.NodefileStart = 0.5
        m.Params.TimeLimit = 200.0
        m.Params.Threads = 8
        # m.Params.MIPFocus = 1
        # m.Params.MinRelNodes = 1
        m.Params.Method = 1
        # m.setParam(m.Params.Method, 1)

        # m.setParam('OutputFlag',0)
        m.modelSense = GRB.MAXIMIZE

        ######### IQP #############
        ### Variables
        z1 = m.addVar(vtype = GRB.INTEGER, ub = self.n)
        z2 = m.addVar(vtype = GRB.INTEGER, lb = 0)
        y = m.addVars(2*self.n, 2*self.n, vtype = GRB.BINARY)
        c = m.addVars(2*self.n, 2*self.n, vtype = GRB.BINARY)
        x = m.addVars(2*self.n, 2*self.n, MOST_PATH, vtype = GRB.BINARY)
        ### Objective function ###
        m.setObjective(
            z1 - (self.n+1)*z2
        )
        ### constraints ###
        
        m.addConstr(
            sum(
                sum(x[2*i, 2*i+1,k] for k in range(MOST_PATH)) for i in range(self.n)
            ) >=z1

        )

        for i in range(self.n):
            # each object choose one buffer
            m.addConstr(
                sum(
                sum(x[2*i+1, p,k] for k in range(MOST_PATH))
                for p in range(2*self.n))==1
                )
            # obj i goes before obj n+i
            m.addConstr(
                c[self.n+i, i] == 1
            )

        # |FAS| = 0
        m.addConstr(
            sum(
                (
                    sum(
                        c[k,i]*y[k,i] for k in range(i)
                    ) + sum(
                        c[l,i]*(1-y[i,l]) for l in range(i, 2*self.n)
                    )
                ) for i in range(2*self.n)
            ) <= z2
        )

        for k in range(2*self.n):
            for i in range(k+1, 2*self.n):
                for l in range(i+1, 2*self.n):
                    m.addConstr(y[k,i] + y[i,l] - y[k,l]>= 0)
                    m.addConstr(y[k,i] + y[i,l] - y[k,l]<= 1)
    
        for i in range(self.n):
            for p in range(2*self.n):
                # where the obj n+i starts is where the obj i ends
                m.addConstr(
                sum(
                    x[2*i, p, k] for k in range(MOST_PATH)
                )-sum(
                    x[2*i+1, p, k] for k in range(MOST_PATH)
                ) == 0)

            # obj i cannot use Si to be the buffer
            m.addConstr(
                sum(x[2*i,2*i,k] for k in range(MOST_PATH))<=0
            )
        
        for key in self.path_dict.keys():
            path_key_list = []
            if (key[0] == key[1]) and (key[0]%2):
                path_key_list = [(key[0], key[0])]
            elif (key[0] != key[1]):
                path_key_list = [(key[0], key[1]), (key[1], key[0])]
            for path_key in path_key_list:
                obj_index = path_key[0]//2 + (path_key[0]%2)*self.n 
                for k in range(len(self.path_dict[key])):
                    dependency_set = self.path_dict[key][k]
                    for pose in dependency_set:
                        #collides with start poses
                        if (pose[1]==0):
                            m.addConstr(x[path_key[0], path_key[1], k] - c[obj_index,pose[0]]<=0)
                            for l in range(self.n):
                                m.addConstr(
                                    sum(x[2*l, 2*pose[0], k] for k in range(MOST_PATH))*x[path_key[0], path_key[1], k]-(c[l,obj_index]+c[obj_index,self.n+l])<= 0
                                    )
                        # collides with goal poses
                        if (pose[1]==1):
                            m.addConstr(x[path_key[0], path_key[1], k] - c[self.n+pose[0],obj_index]<=0)
                            for l in range(self.n):
                                m.addConstr(
                                    sum(x[2*l, 2*pose[0]+1, k] for k in range(MOST_PATH))*x[path_key[0], path_key[1], k]-(c[l,obj_index]+c[obj_index,self.n+l])<= 0
                                    )
                # if there is no such path, you cannot use it
                for k in range(len(self.path_dict[key]), MOST_PATH):
                    m.addConstr(x[path_key[0], path_key[1], k]<=0)
        

        m.optimize()
        obj = m.getObjective()
        if obj.getValue()>=0:
            self.data.append(True)
        else:
            self.data.append(False)
        try:
            self.data.append(self.n - obj.getValue())
        except Exception:
            self.data.append(-1)

        ### figure out what path options are chosen
        path_selection = {}
        buffer_selection = {}
        for i in range(self.n):
            FLAG = False
            for p in range(2*self.n):
                for k in range(MOST_PATH):
                    if x[2*i,p,k].x == 1:
                        FLAG = True
                        buffer_selection[i] = p
                        path_selection[i]=k
                        break
                if FLAG:
                    for j in range(MOST_PATH):
                        if x[2*i+1, p ,j].x == 1:
                            print (2*i+1, p ,j)
                            path_selection[i+self.n] = j
                            break
                    break
        
        # print x and c
        # for i in range(2*self.n):
        #     for j in range(2*self.n):
        #         lst = [x[i,j,k].x for k in range(MOST_PATH)]
        #         print "x", [i,j], lst

        # for i in range(2*self.n):
        #     for j in range(2*self.n):
        #         if c[i,j].x>=0.5:
        #             print i,j

        ### figure out the order
        order = []
        for i in range(2*self.n):
            ranking = 0
            for j in range(i):
                ranking += y[j,i].x
            for j in range(i+1, 2*self.n):
                ranking += (1-y[i,j].x)
            order.append((ranking,i))
        sorted_order = sorted(order)
        final_order = [object_ranking[1] for object_ranking in sorted_order]

        print "path_selection", path_selection
        print "buffer_selection", buffer_selection
        print "final order", final_order
        return 2*self.n - obj.getValue(), buffer_selection, tuple([path_selection[i] for i in range(2*self.n)]), final_order

class Object_Rearrangement(object):
    def __init__(self, W_X, W_Y, OBJ_NUM):
        self.XDIM = W_X
        self.YDIM = W_Y
        self.n = OBJ_NUM
        self.radius = 0.5
        self.path_dict = {}
        self.dependency_dict = {}
        self.start_pose = self.randomize_instance()
        self.goal_pose = self.randomize_instance()
        self.plot_instance()
        self.check_overlap()
        self.construct_roadmap()
        # I have LL now
        self.plot_roadmap()
        self.construct_path_dict()

    def plot_instance(self):
        circles = []
        fig, ax = plt.subplots(figsize=(8, 8))
        for pose_key in self.goal_pose.keys():
            pose = self.goal_pose[pose_key]
            circles.append(plt.Circle(pose, self.radius, color='r'))
            plt.text(pose[0], pose[1], 'G_' + str(pose_key), dict(size=10))
        for pose_key in self.start_pose.keys():
            pose = self.start_pose[pose_key]
            circles.append(plt.Circle(pose, self.radius, color='b'))
            plt.text(pose[0], pose[1], 'S_' + str(pose_key), dict(size=10))
        for circle in circles:
            ax.add_artist(circle)
        plt.xlim((0 - self.radius, self.XDIM + self.radius))
        plt.ylim((0 - self.radius, self.YDIM + self.radius))
        plt.savefig(my_path + "/pictures/instance_test.png")
        # plt.show()

    def construct_path_dict(self):
        for key in self.start_pose.keys():
            self.pruning_search(key)
            if len(self.dependency_dict[key]) == 0:
                print "isolation occurs, key = ", key
                self.add_trivial_path(key)

    def add_trivial_path(self, key):
        pose1_loc = self.start_pose[key]
        pose2_loc = self.goal_pose[key]
        path = [(key, 0), (key, 1)]
        dependency_set = set()
        for pose in [(i, j) for i in self.start_pose.keys() for j in range(2)]:
            if (pose == (key, 0)) or (pose == (key, 0)):
                continue
            if pose[1]:
                pose_loc = self.goal_pose[pose[0]]
            else:
                pose_loc = self.start_pose[pose[0]]
            dist = self.find_dist_p_2_l(pose1_loc, pose2_loc, pose_loc)
            if (dist < 2 * self.radius):
                dependency_set.add(pose)
            self.path_dict[key].append(path)
            self.dependency_dict[key].append(dependency_set)

    def pruning_search(self, key):
        self.path_dict[key] = []
        self.dependency_dict[key] = []
        nodes = {}
        parents = {}
        pruning = {}
        goal_nodes = []
        nodes[1] = (key, 0)
        queue = [1]
        while len(queue) > 0:
            node = queue.pop()
            if nodes[node] == (key, 1):
                goal_nodes.append(node)
                continue
            if node in parents:
                pruning_set = pruning[parents[node]]
            else:
                pruning_set = {nodes[node]}
            if nodes[node] in self.LL:
                for pose in self.LL[nodes[node]]:
                    if pose not in pruning_set:
                        index = len(nodes) + 1
                        nodes[index] = pose
                        queue.append(index)
                        parents[index] = node
                pruning[node] = pruning_set.union(self.LL[nodes[node]])
        # print "parents", parents
        # print "goal", goal_nodes
        # print "nodes", nodes
        for node in goal_nodes:
            current_node = node
            path = []
            dependency_set = set()
            while current_node in parents:
                path.append(nodes[current_node])
                if nodes[current_node] in self.pose_overlap:
                    dependency_set = dependency_set.union(set(self.pose_overlap[nodes[current_node]]))
                dependency_set = dependency_set.union({nodes[current_node]})
                current_node = parents[current_node]
            path.append(current_node)
            if nodes[current_node] in self.pose_overlap:
                dependency_set = dependency_set.union(set(self.pose_overlap[nodes[current_node]]))
            dependency_set = dependency_set.union({nodes[current_node]})
            dependency_set = dependency_set.difference({(key, 0), (key, 1)})
            self.path_dict[key].append(list(reversed(path)))
            self.dependency_dict[key].append(dependency_set)

    def check_overlap(self):
        self.pose_overlap = {}
        for s_key in self.start_pose.keys():
            s = self.start_pose[s_key]
            for g_key in self.goal_pose.keys():
                g = self.goal_pose[g_key]
                dist = math.sqrt((s[0] - g[0])**2 + (s[1] - g[1])**2)
                if dist < 2 * self.radius:
                    if (s_key, 0) not in self.pose_overlap:
                        self.pose_overlap[(s_key, 0)] = []
                    if (g_key, 1) not in self.pose_overlap:
                        self.pose_overlap[(g_key, 1)] = []
                    self.pose_overlap[(s_key, 0)].append((g_key, 1))
                    self.pose_overlap[(g_key, 1)].append((s_key, 0))

    def construct_roadmap(self):
        self.LL = {}
        for pose1 in [(i, j) for i in self.start_pose.keys() for j in range(2)]:
            for pose2 in [(i, j) for i in self.start_pose.keys() for j in range(2)]:
                if pose1 == pose2:
                    continue
                if pose2[0] < pose1[0]:
                    continue
                if (pose1[0] == pose2[0]) and (pose1[1] > pose2[1]):
                    continue
                if self.check_collision_without_overlap(pose1, pose2):
                    if pose1 not in self.LL:
                        self.LL[pose1] = []
                    if pose2 not in self.LL:
                        self.LL[pose2] = []
                    self.LL[pose1].append(pose2)
                    self.LL[pose2].append(pose1)

    def plot_roadmap(self):
        circles = []
        fig, ax = plt.subplots(figsize=(8, 8))
        for pose_key in self.goal_pose.keys():
            pose = self.goal_pose[pose_key]
            circles.append(plt.Circle(pose, self.radius, color='r'))
            plt.text(pose[0], pose[1], 'G_' + str(pose_key), dict(size=10))
        for pose_key in self.start_pose.keys():
            pose = self.start_pose[pose_key]
            circles.append(plt.Circle(pose, self.radius, color='b'))
            plt.text(pose[0], pose[1], 'S_' + str(pose_key), dict(size=10))
        for circle in circles:
            ax.add_artist(circle)
        for pose1 in self.LL.keys():
            if pose1[1]:
                pose1_loc = self.goal_pose[pose1[0]]
            else:
                pose1_loc = self.start_pose[pose1[0]]
            for pose2 in self.LL[pose1]:
                if pose2[1]:
                    pose2_loc = self.goal_pose[pose2[0]]
                else:
                    pose2_loc = self.start_pose[pose2[0]]
                plt.plot([pose1_loc[0], pose2_loc[0]], [pose1_loc[1], pose2_loc[1]], color='g')
        plt.xlim((0 - self.radius, self.XDIM + self.radius))
        plt.ylim((0 - self.radius, self.YDIM + self.radius))
        plt.savefig(my_path + "/pictures/roadmap_test.png")
        # plt.show()

    def check_collision_without_overlap(self, pose1, pose2):
        if pose1[1]:
            pose1_loc = self.goal_pose[pose1[0]]
        else:
            pose1_loc = self.start_pose[pose1[0]]

        if pose2[1]:
            pose2_loc = self.goal_pose[pose2[0]]
        else:
            pose2_loc = self.start_pose[pose2[0]]

        for pose in [(i, j) for i in self.start_pose.keys() for j in range(2)]:
            if (pose == pose1) or (pose == pose2):
                continue
            if (((pose1 not in self.pose_overlap) or ((pose1 in self.pose_overlap) and
                                                      (pose not in self.pose_overlap[pose1])))
                    and ((pose2 not in self.pose_overlap) or ((pose2 in self.pose_overlap) and
                                                              (pose not in self.pose_overlap[pose2])))):
                if pose[1]:
                    pose_loc = self.goal_pose[pose[0]]
                else:
                    pose_loc = self.start_pose[pose[0]]
                dist = self.find_dist_p_2_l(pose1_loc, pose2_loc, pose_loc)
                if (dist < 2 * self.radius):
                    return False
        return True

    def find_dist_p_2_l(self, l1, l2, p):
        if l1 == l2:
            return (math.sqrt((l1[0] - p[0])**2 + (l1[1] - p[1])**2))
        alpha = ((l2[0] - l1[0]) * (p[0] - l1[0]) + (l2[1] - l1[1]) *
                 (p[1] - l1[1])) / ((l1[0] - l2[0])**2 + (l1[1] - l2[1])**2)
        if alpha >= 1:
            alpha = 1
        if alpha <= 0:
            alpha = 0
        v = (alpha * l2[0] + (1 - alpha) * l1[0], alpha * l2[1] + (1 - alpha) * l1[1])
        return math.sqrt((v[0] - p[0])**2 + (v[1] - p[1])**2)

    def randomize_instance(self):
        obj_location = {}
        for i in range(self.n):
            index = i + 1
            while 1:
                x = np.random.uniform() * self.XDIM
                y = np.random.uniform() * self.YDIM
                valid = True
                for obj in obj_location.values():
                    dist = math.sqrt((obj[0] - x)**2 + (obj[1] - y)**2)
                    if dist <= 2 * self.radius:
                        valid = False
                        break
                if valid:
                    obj_location[index] = [x, y]
                    break
        return obj_location


class Generate_Path_Dictionary(object):
    # Input: the linked list of the Connectivity Graph.
    # Output: the dependency dictionary
    def __init__(self, graph):
        self.path_dict = {}
        self.dependency_dict = {}
        self.n = len(graph) / 2
        # print "the number of objects:", self.n
        self.start_pose = range(1, self.n + 1)
        self.linked_list_conversion(graph)
        self.construct_path_dict()

    def linked_list_conversion(self, graph):
        self.LL = {}
        for key in graph:
            self.LL[(key // 2 + 1, key % 2)] = []
            for pose in graph[key]:
                self.LL[(key // 2 + 1, key % 2)].append((pose // 2 + 1, pose % 2))

    def construct_path_dict(self):
        for key in self.start_pose:
            self.pruning_search(key)
            # if len(self.dependency_dict[key])==0:
            #     print "isolation occurs, key = ", key
            #     self.add_trivial_path(key)

    def pruning_search(self, key):
        self.path_dict[key] = []
        self.dependency_dict[key] = []
        nodes = {}
        parents = {}
        pruning = {}
        goal_nodes = []
        nodes[1] = (key, 0)
        queue = [1]
        while len(queue) > 0:
            node = queue.pop()
            if nodes[node] == (key, 1):
                goal_nodes.append(node)
                continue
            if node in parents:
                pruning_set = pruning[parents[node]]
            else:
                pruning_set = set(nodes[node])
            if nodes[node] in self.LL:
                for pose in self.LL[nodes[node]]:
                    if pose not in pruning_set:
                        index = len(nodes) + 1
                        nodes[index] = pose
                        queue.append(index)
                        parents[index] = node
                pruning[node] = pruning_set.union(self.LL[nodes[node]])
        # print "parents", parents
        # print "goal", goal_nodes
        # print "nodes", nodes
        for node in goal_nodes:
            current_node = node
            path = []
            dependency_set = set()
            while current_node in parents:
                path.append(nodes[current_node])
                dependency_set = dependency_set.union({nodes[current_node]})
                current_node = parents[current_node]
            path.append(current_node)
            dependency_set = dependency_set.union({nodes[current_node]})
            dependency_set = dependency_set.difference({(key, 0), (key, 1)})
            self.path_dict[key].append(list(reversed(path)))
            self.dependency_dict[key].append(dependency_set)


class Dense_Path_Generation(object):
    # Input: Danniel's region connectivity graph
    # Output: the dependency dictionary
    def __init__(self, graph, obj_locations):
        self.obj_locations = obj_locations
        self.path_dict = {}
        self.dependency_dict = {}
        self.n = len(obj_locations) / 2
        # print "the number of objects:", self.n
        self.poses = range(0, 2*self.n)
        # self.start_pose = range(0, self.n)
        self.linked_list_conversion(graph)
        self.construct_path_dict()
        self.dependency_dict_conversion()
        # dependency_dict: key:(5,8) value: [set((2,1), (1,0), ..., (4,0)), set(...)]

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

    def construct_path_dict(self):
        for i in range(len(self.poses)):
            key1 = self.poses[i]
            for j in range(i,len(self.poses)):
                if i == j:
                    if j%2:
                        self.dependency_dict[(key1, key1)] = [self.get_dependency_set_from_index(self.region_dict[self.obj_locations[key1]])]
                        self.path_dict[(key1, key1)] = [[self.region_dict[self.obj_locations[key1]], self.region_dict[self.obj_locations[key1]]]]
                    continue
                key2 = self.poses[j]
                key = (key1, key2)
                self.dependency_set_pruning_search(key)

    def dependency_set_pruning_search(self, key):
        self.path_dict[key] = []
        self.dependency_dict[key] = []  # key:obj,obj, value: a list of dependency set
        vertex2node_dict = {}
        for region in self.region_dict.values():
            vertex2node_dict[region] = []
        node_dependency_set_dict = {}  # the dictionary for the dependency set of each node in the search tree
        nodes = {}
        parents = {}
        nodes[1] = self.region_dict[self.obj_locations[key[0]]]
        node_dependency_set_dict[1] = self.get_dependency_set_from_index(nodes[1])
        vertex2node_dict[self.region_dict[self.obj_locations[key[0]]]].append(1)
        queue = [1]
        while len(queue) > 0:
            old_node = queue.pop()
            if nodes[old_node] in self.LL:  # if it has neighbors
                for pose in self.LL[nodes[old_node]]:
                    current_dependency_set = node_dependency_set_dict[old_node].union(
                        self.get_dependency_set_from_index(pose)
                    )
                    Abandoned = False
                    remove_list = []
                    for n in vertex2node_dict[pose]:
                        if current_dependency_set.issuperset(node_dependency_set_dict[n]):
                            Abandoned = True
                            break
                        if node_dependency_set_dict[n].issuperset(current_dependency_set):
                            remove_list.append(n)
                    for n in remove_list:
                        vertex2node_dict[pose].remove(n)
                    if not Abandoned:
                        node = len(nodes) + 1
                        nodes[node] = pose
                        parents[node] = old_node
                        vertex2node_dict[pose].append(node)
                        queue.append(node)
                        node_dependency_set_dict[node] = current_dependency_set

        goal_nodes = vertex2node_dict[self.region_dict[self.obj_locations[key[1]]]]

        for node in goal_nodes:
            current_node = node
            path = []
            while current_node in parents:  # while it is not the root(start pose).
                path.append(nodes[current_node])
                current_node = parents[current_node]
            path.append(nodes[current_node])
            # In this model we should keep the start and goal poses in the dependency graph, because they could be occupied
            # node_dependency_set_dict[node] = node_dependency_set_dict[node].difference({key[0], key[1]})
            self.path_dict[key].append(list(reversed(path)))
            self.dependency_dict[key].append(node_dependency_set_dict[node])

        # print "parents", parents
        # print "goal", goal_nodes
        # print "nodes", nodes
        # print "node_dependency_set_dict"
        # print node_dependency_set_dict
        # print "vertex2node_dict"
        # print vertex2node_dict

    def get_path_dependency_set(self, node, parents, nodes, key):
        current_node = node
        path = []
        dependency_set = set()
        while current_node in parents:  # while it is not the root(start pose).
            path.append(nodes[current_node])
            dependency_set = dependency_set.union(self.get_dependency_set_from_index(nodes[current_node]))
            current_node = parents[current_node]
        path.append(current_node)
        dependency_set = dependency_set.union(self.get_dependency_set_from_index(nodes[current_node]))
        dependency_set = dependency_set.difference({2 * key, 2 * key + 1})
        self.path_dict[key].append(list(reversed(path)))
        self.dependency_dict[key].append(dependency_set)

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
    if (len(sys.argv) < 5):
        print(
            '''Error: deptree.py: <# objects> <height> <width> <radius>
            [display?: (y/n)] [display more?: (y/n)] [save file] [load file?:(y/n)]'''
        )
        exit()

    try:
        numObjs = int(sys.argv[1])
        RAD = float(sys.argv[2])
        HEIGHT = int(sys.argv[3])
        WIDTH = int(sys.argv[4])
    except ValueError:
        print(
            '''Error: deptree.py: <# objects> <height> <width> <radius>
            [display?: (y/n)] [display more?: (y/n)] [save file] [load file?:(y/n)]'''
        )
        exit()

    display = False
    if (len(sys.argv) > 5):
        display = sys.argv[5] not in ('n', 'N')

    displayMore = False
    if (len(sys.argv) > 6):
        displayMore = sys.argv[6] not in ('n', 'N')

    savefile = False
    if (len(sys.argv) > 7):
        savefile = sys.argv[7]

    loadfile = False
    if (len(sys.argv) > 8):
        loadfile = sys.argv[8] not in ('n', 'N')
        if sys.argv[8] in ('x', 'X'):
            savefile = False
            loadfile = False

    saveimage = False
    if (len(sys.argv) > 9):
        saveimage = sys.argv[9] not in ('n', 'N')

    example_index = int(sys.argv[10])

    # DGs = DG_Space(path_dict)
    # print DGs.DGs
    # IP = feekback_arc_ILP(path_dict)
    EXP = Experiments()
    set_max_memory(1.3*2**(34))#2**34=16G
    if loadfile:
        EXP.load_instance(savefile, True, display, displayMore)
    else:
        EXP.single_instance(numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, example_index)
        

