import os
import sys
import copy
from collections import OrderedDict
from cgraph import genDenseCGraph, drawMotions, animatedMotions
# from cgraph import genCGraph, genDenseCGraph, loadCGraph, drawMotions, animatedMotions
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

my_path = os.path.abspath(os.path.dirname(__file__))

class Experiments(object):
    def single_instance(self, numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, example_index):

        # print "\nFirst generate the instance and get the dense cgraph..."
        graph, paths, objects, color_pool, points, objectShape, object_locations = genDenseCGraph(
            numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, example_index
        )
        ### keep generating instances until it is successfully generated
        while graph == False:
            graph, paths, objects, color_pool, points, objectShape, object_locations = genDenseCGraph(
                numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, example_index
            )

        if graph is paths is objects is False:
            return -1

        # # print "graph: " + str(graph)
        # # print "paths: " + str(paths)
        ### Generate path options
        gpd = Dense_Path_Generation(graph, object_locations)
        # IP_vertex = feedback_vertex_ILP(gpd.dependency_dict)
        # print "ILP result(the smallest size of FVS):", IP_vertex.optimum
        # IP_arc = feedback_arc_ILP(gpd.dependency_dict)
        # print "ILP result(the smallest size of FAS):", IP_arc.optimum

        # arc_setSize, arcs, path_selection, object_ordering = IP_arc.optimum
        # path_opts = gpd.path_dict
        # new_paths = {}
        # for r1, r2 in paths.keys():
        #     new_paths[(gpd.region_dict[r1], gpd.region_dict[r2])] = copy.deepcopy(paths[(r1, r2)])

        # if display:
        #     rpaths = self.drawSolution(
        #         HEIGHT, WIDTH, numObjs, RAD, 
        #         new_paths, path_opts, path_selection,
        #         objects, color_pool, points, 
        #         example_index, saveimage
        #     )
        #     print "display the animation of objects movement"
        #     animatedMotions(
        #         HEIGHT, WIDTH, numObjs, RAD,
        #         rpaths, color_pool, points,
        #         object_ordering, example_index, objectShape
        #     )

    def drawSolution(
        self, HEIGHT, WIDTH, numObjs, RAD, 
        paths, path_opts, ind_opt, 
        objects, color_pool, points, 
        example_index, saveimage
    ):
        rpaths = OrderedDict()
        for obj, iopt in enumerate(ind_opt, 0):
            dpath = path_opts[obj][iopt]
            # deps = {2 * (x[0]) + x[1] for x in dpath}
            # deps = set(dpath)
            # print "dpath", dpath
            start = 2 * obj
            goal = 2 * obj + 1
            # print(deps)

            rpath = [points[start]]
            for i in range(len(dpath) - 2):
                region1 = dpath[i]
                region2 = dpath[i + 1]
                if (region1, region2) in paths.keys():
                    rpath += paths[(region1, region2)][:-1]
                elif (region2, region1) in paths.keys():
                    rpath += list(reversed(paths[(region2, region1)]))[:-1]
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
                else:
                    print "invalid path"
                    exit()
            rpath.append(points[goal])
            rpaths[(start, goal)] = rpath
        # print "rpaths: "
        # print(rpaths)
        print "\ndisplay the paths solution"      
        drawMotions(HEIGHT, WIDTH, numObjs, RAD, rpaths, color_pool, points, example_index, saveimage, objects)

        return rpaths

class Dense_Path_Generation(object):
    ### Input: Dan's region connectivity graph
    ### Output: the dependency dictionary
    def __init__(self, graph, obj_locations):
        self.obj_locations = obj_locations
        self.path_dict = {}
        self.dependency_dict = {}
        self.n = len(obj_locations) / 2
        # print "the number of objects:", self.n
        self.start_pose = range(0, self.n)
        self.linked_list_conversion(graph)
        self.construct_path_dict()
        self.dependency_dict_conversion()

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
        for key in self.start_pose:
            self.dependency_set_pruning_search(key)
            # if len(self.dependency_dict[key])==0:
            #     print "isolation occurs, key = ", key
            #     self.add_trivial_path(key)

    def pruning_search(self, key):
        self.path_dict[key] = []
        self.dependency_dict[key] = []  # key:obj, value: a list of dependency set
        region_depandency_on_path = {}
        for i in self.region_dict.values():
            region_depandency_on_path[i] = []
        nodes = {}
        parents = {}
        pruning = {}
        goal_nodes = []
        nodes[1] = self.region_dict[self.obj_locations[2 * key]]
        queue = [1]
        current_dependency_set = self.get_dependency_set_from_index(self.region_dict[self.obj_locations[2 * key]])
        region_depandency_on_path[self.region_dict[self.obj_locations[2 * key]]].append(current_dependency_set)
        while len(queue) > 0:
            node = queue.pop()
            if nodes[node] == self.region_dict[self.obj_locations[2 * key + 1]]:
                goal_nodes.append(node)
                continue
            if node in parents:  # if the node is not the root
                pruning_set = pruning[parents[node]]
            else:
                pruning_set = {nodes[node]}
            if nodes[node] in self.LL:  # if it has neighbor
                for pose in self.LL[nodes[node]]:
                    if pose not in pruning_set:  # if the path should not be pruned.
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
            while current_node in parents:  # while it is not the root(start pose).
                path.append(nodes[current_node])
                dependency_set = dependency_set.union(self.get_dependency_set_from_index(nodes[current_node]))
                current_node = parents[current_node]
            path.append(current_node)
            dependency_set = dependency_set.union(self.get_dependency_set_from_index(nodes[current_node]))
            dependency_set = dependency_set.difference({2 * key, 2 * key + 1})
            self.path_dict[key].append(list(reversed(path)))
            self.dependency_dict[key].append(dependency_set)

    def dependency_set_pruning_search(self, key):
        self.path_dict[key] = []
        self.dependency_dict[key] = []  # key:obj, value: a list of dependency set
        vertex2node_dict = {}
        for region in self.region_dict.values():
            vertex2node_dict[region] = []
        node_dependency_set_dict = {}  # the dictionary for the dependency set of each node in the search tree
        nodes = {}
        parents = {}
        nodes[1] = self.region_dict[self.obj_locations[2 * key]]
        node_dependency_set_dict[1] = self.get_dependency_set_from_index(nodes[1])
        vertex2node_dict[self.region_dict[self.obj_locations[2 * key]]].append(1)
        queue = [1]
        while len(queue) > 0:
            old_node = queue.pop()
            if nodes[old_node] in self.LL:  # if it has neighbor
                for pose in self.LL[nodes[old_node]]:
                    current_dependency_set = node_dependency_set_dict[old_node].union(
                        self.get_dependency_set_from_index(pose)
                    )
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
                        node_dependency_set_dict[node] = current_dependency_set

        goal_nodes = vertex2node_dict[self.region_dict[self.obj_locations[2 * key + 1]]]

        for node in goal_nodes:
            current_node = node
            path = []
            while current_node in parents:  # while it is not the root(start pose).
                path.append(nodes[current_node])
                current_node = parents[current_node]
            path.append(nodes[current_node])
            node_dependency_set_dict[node] = node_dependency_set_dict[node].difference({2 * key, 2 * key + 1})
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


class feedback_arc_ILP(object):
    def __init__(self, path_dict):
        self.path_dict = path_dict
        self.optimum = self.run_arc_ILP()

    def run_arc_ILP(self):
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

        ###### Minimum feedback arc ILP formulation ######
        ### variables
        y = m.addVars(n, n, vtype=GRB.BINARY) ### ordering indicator
        C = m.addVars(n, n, vtype=GRB.BINARY) ### dependency graph indicator
        x = m.addVars(n, MOST_PATH, vtype=GRB.BINARY) ### path selection indicator
        ### Objective function ###
        m.setObjective(
            sum(
                sum(C[k, i] * y[k, i] for k in range(i)) + sum(C[l, i] * y[l, i] for l in range(i+1,n))
                for i in range(n)
            )
        )
        ### constraints ###
        ### constraint 1: no cycle in ordering
        for i in range(n):
            for j in range(i + 1, n):
                for k in range(j + 1, n):
                    m.addConstr(y[i, j] + y[j, k] + y[k, i] >= 1)
                    m.addConstr(y[i, j] + y[j, k] + y[k, i] <= 2)
        ### constraint 2: validity of ordering
        for i in range(n):
            for j in range(i + 1, n):
                m.addConstr(y[i, j] + y[j, i] == 1)

        ### constraint 3: path selections
        for i in range(n):
            for j in range(len(self.path_dict[objs[i]]), MOST_PATH):
                m.addConstr(x[i, j] == 0)
        for i in range(n):
            m.addConstr(sum(x[i, u] for u in range(MOST_PATH)) == 1)
        m.addConstrs(
            C[j, k] <= sum(x[objs.index(i), u]
                            for (i, u) in dependency_dict[(j, k)])
            for (j, k) in dependency_dict.keys()
        )
        for (j, k) in dependency_dict.keys():
            for (i, u) in dependency_dict[(j, k)]:
                m.addConstr(C[j, k] >= x[objs.index(i), u])

        m.addConstrs(
            C[i, i] == 0 for i in range(n)
        )

        m.optimize()
        obj = m.getObjective()

        print("\nFAS output: ")

        ### figure out what path options are chosen for each object
        print "path_selection: "
        path_selection = []
        for i in range(n):
            for j in range(MOST_PATH):
                if x[i, j].x > 0.5:
                    path_selection.append(j)
                    print "Object " + str(i) + ": " + str(self.path_dict[i][j])
        print("path_selection: ", path_selection)


        ### figure out the edges/constraints to be removed to break the cycle
        edges_to_be_removed = []
        for i in range(n):
            for j in range(n):
                if (C[i, j].x == 1.0):
                    if (y[i, j].x == 1.0):
                        edges_to_be_removed.append((i, j))

        ### print the dependency graph
        DG = np.zeros([n, n])
        for i in range(n):
            for j in range(n):
                if C[i, j].x == 1.0:
                    DG[i, j] = 1
        print("The dependecy graph is: ")
        print(DG)

        ### print final ordering
        Y = np.zeros([n, n])
        for i in range(n):
            for j in range(n):
                Y[i, j] = y[i, j].x
        obj_indexes = [i for i in range(n)]
        order_count = np.sum(Y, axis=1)
        final_order = [obj_index for _, obj_index in sorted(zip(order_count, obj_indexes), reverse=True)]
        print("Final objects order: ")
        print(final_order)


        return obj.getValue(), edges_to_be_removed, tuple(path_selection), final_order

################################################################################################

class feedback_vertex_ILP(object):
    def __init__(self, path_dict):
        self.path_dict = path_dict
        self.optimum = self.run_vertex_ILP()

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

        print("\nFVS output: ")

        ### figure out what path options are chosen for each object
        print "path_selection: "
        path_selection = []
        for i in range(n):
            for j in range(MOST_PATH):
                if x[i, j].x > 0.5:
                    path_selection.append(j)
                    print "Object " + str(i) + ": " + str(self.path_dict[i][j])
        print("path_selection: ", path_selection)


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
        print("The dependecy graph is: ")
        print(C)

        Y = np.zeros([n, 2 * n])
        for i in range(n):
            for j in range(2 * n):
                Y[i, j] = y[i, j].x
        obj_indexes = [i for i in range(n)]
        order_count = np.sum(Y, axis=1)
        final_order = [obj_index for _, obj_index in sorted(zip(order_count, obj_indexes), reverse=True)]
        print("Final objects order: ")
        print(final_order)


        return obj.getValue(), vertices_to_be_removed, tuple(path_selection), final_order

################################################################################################

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

    EXP = Experiments()
    print "Number of objects: ", numObjs
    print "Radius: ", RAD
    print "Environment size: ", HEIGHT, WIDTH

    if loadfile:
        EXP.load_instance(savefile, True, display, displayMore)
    else:
        EXP.single_instance(numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, example_index)






    # ### an example to check the correctness of FAS and FVS
    # path_dict = {}
    # path_dict[0] = [
    #     [(1,1), (2,0)]
    # ]

    # path_dict[1] = [
    #     [(0,0), (2,1)]
    # ]

    # path_dict[2] = [
    #     [(0,1), (3,0)]
    # ]

    # path_dict[3] = [
    #     [(4,0)]
    # ]

    # path_dict[4] = [
    #     [(2,0)]
    # ]

    # IP_vertex = feedback_vertex_ILP(path_dict)
    # print "ILP result(the smallest size of FVS):", IP_vertex.optimum
    # IP_arc = feedback_arc_ILP(path_dict)
    # print "ILP result(the smallest size of FAS):", IP_arc.optimum
        