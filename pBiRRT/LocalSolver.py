from __future__ import division

import numpy as np
import gurobipy as gp
from gurobipy import GRB
import math

class feedback_arc_ILP_buffers(object):
    def __init__(self, path_dict):
        self.path_dict = path_dict
        self.optimum = self.run_arc_ILP_buffers()

    def run_arc_ILP_buffers(self):
        MOST_PATH = 0
        for paths in self.path_dict.values():
            MOST_PATH = max(MOST_PATH, len(paths))
        # print "most paths", MOST_PATH
        n = len(self.path_dict)
        # print "number of object in ILP local solver: " + str(n)
        dependency_dict = {}
        for i in range(n):
            for path_index in range(len(self.path_dict[i])):
                for constr in self.path_dict[i][path_index]:
                    if constr[1] == 0:
                        if (i, constr[0]) not in dependency_dict:
                            dependency_dict[(i, constr[0])] = []
                        dependency_dict[(i, constr[0])].append([i, path_index])
                    else:
                        if (constr[0], i) not in dependency_dict:
                            dependency_dict[(constr[0], i)] = []
                        dependency_dict[(constr[0], i)].append([i, path_index])
        
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
            for j in range(len(self.path_dict[i]), MOST_PATH):
                m.addConstr(x[i, j] == 0)
        for i in range(n):
            m.addConstr(sum(x[i, u] for u in range(MOST_PATH)) == 1)
        m.addConstrs(
            C[j, k] <= sum(x[i, u]
                            for (i, u) in dependency_dict[(j, k)])
            for (j, k) in dependency_dict.keys()
        )
        for (j, k) in dependency_dict.keys():
            for (i, u) in dependency_dict[(j, k)]:
                m.addConstr(C[j, k] >= x[i, u])

        m.addConstrs(
            C[i, i] == 0 for i in range(n)
        )

        m.optimize()
        obj = m.getObjective()

        # print("MOST_PATH: " + str(MOST_PATH))
        # print("x: " + str(x))

        # print("\nFAS output: ")

        ### figure out what path options are chosen for each object
        # print "path_selection: "
        path_selection = []
        for i in range(n):
            for j in range(MOST_PATH):
                if x[i, j].x > 0.5:
                    path_selection.append(j)
                    # print "Object " + str(i) + ": " + str(self.path_dict[i][j])
        # print("path_selection: ", path_selection)


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
        # print("The dependecy graph is: ")
        # print(DG)

        ### print final ordering
        Y = np.zeros([n, n])
        for i in range(n):
            for j in range(n):
                Y[i, j] = y[i, j].x
        obj_indexes = [i for i in range(n)]
        order_count = np.sum(Y, axis=1)
        final_order = [obj_index for _, obj_index in sorted(zip(order_count, obj_indexes), reverse=True)]
        # print("Final objects order: ")
        # print(final_order)


        return obj.getValue(), edges_to_be_removed, tuple(path_selection), final_order
        

################################################################################################