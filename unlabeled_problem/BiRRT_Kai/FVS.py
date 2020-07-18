import numpy as np
import gurobipy as gp
from gurobipy import GRB
import math
import copy



class feekback_vertex_ILP(object):
    def __init__(self, path_dict, n):
        self.n = n
        self.dependency_dict = copy.deepcopy(path_dict)
        self.path_dict = self.path_dict_transformation()
        self.optimum = self.run_vertex_ILP()
        # self.optimum = self.run_arc_ILP()

    def path_dict_transformation(self):
        query_arrangement = [2*i for i in range(self.n)]
        nearest_arrangement = [2*i+1 for i in range(self.n)]
        return self.object_dependency_opts_generate(query_arrangement, nearest_arrangement)

    def object_dependency_opts_generate(self, query_arrangement, nearest_arrangement):
        num_objs = len(query_arrangement)
        object_dependency_opts = {}
        for obj_idx in range(num_objs):
            if obj_idx not in object_dependency_opts.keys():
                object_dependency_opts[obj_idx] = []
            ### look into the path option for the current object's current and nearest pose
            pose_key1 = min(query_arrangement[obj_idx], nearest_arrangement[obj_idx])
            pose_key2 = max(query_arrangement[obj_idx], nearest_arrangement[obj_idx])
            for path in self.dependency_dict[(pose_key1, pose_key2)]:
                path_set = set()
                for constr in path:
                    ### check the constraint based on query_arrangement and nearest_arrangement
                    ### if the constraint is caused by current pose of an object
                    if (constr in query_arrangement) and (query_arrangement.index(constr) != obj_idx):
                        path_set = path_set.union({(query_arrangement.index(constr), 0)})
                    ### if the constraint is caused by final pose of an object
                    if (constr in nearest_arrangement) and (nearest_arrangement.index(constr) != obj_idx):
                        path_set = path_set.union({(nearest_arrangement.index(constr), 1)})
                    ### Otherwise, it is a buffer and do nothing
                object_dependency_opts[obj_idx].append(path_set)

        return object_dependency_opts

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
        print("The dependecy graph is: ")
        print(C)

        Y = np.zeros([n, 2 * n])
        for i in range(n):
            for j in range(2 * n):
                Y[i, j] = y[i, j].x
        obj_indexes = [i for i in range(n)]
        order_count = np.sum(Y, axis=1)
        final_order = [
            obj_index for _, obj_index in sorted(zip(order_count, obj_indexes), reverse=True)
        ]
        print("Final objects order: ")
        print(final_order)

        # print("order: ", y)
        # print("\n")
        # print("DG: " , M)
        # print("\n")
        print("Number of vertex to be removed: ", obj.getValue())
        print("Vertices to be removed: ", vertices_to_be_removed)

        return obj.getValue()
