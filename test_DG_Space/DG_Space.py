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
import os
import cPickle as pickle

my_path = os.path.abspath(os.path.dirname(__file__))

class Experiments(object):
    # try to create instances in different areas and object numbers.
    def density(self):
        OBJ_NUM = 5
        # RECORD is False when I'm debugging and don't want to rewrite the data
        RECORD = True
        Iteration_time = 10

        DG_num_matrix = np.zeros([5,5])
        min_feedback_matrix = np.zeros([5,5])
        for W_X in range(3,8):
            for W_Y in range(3,8):
                for iter in range(Iteration_time):
                    OR = Object_Rearrangement(W_X, W_Y, OBJ_NUM)
                    if RECORD:
                        with open(os.path.join(my_path, "settings/W%s_H%s_n%s_iter%s_0301.pkl"%(W_X,W_Y,OBJ_NUM,iter)), 'wb') as output:
                            pickle.dump((OR.start_pose, OR.goal_pose), output, pickle.HIGHEST_PROTOCOL)
                    DGs = DG_Space(OR.dependency_dict)
                    DG_num_matrix[W_X-3, W_Y-3] += len(DGs.DGs.keys())
                    if RECORD:
                        with open(os.path.join(my_path, "DG/W%s_H%s_n%s_iter%s_0301.pkl"%(W_X, W_Y, OBJ_NUM,iter)), 'wb') as output:
                            pickle.dump(DGs.DGs, output, pickle.HIGHEST_PROTOCOL)
                    IP = feekback_arc_ILP(OR.dependency_dict)
                    min_feedback_matrix[W_X-3, W_Y-3] += IP.optimum
                DG_num_matrix[W_X-3, W_Y-3] /= Iteration_time
                min_feedback_matrix[W_X-3, W_Y-3] /= Iteration_time
        
        fig = plt.figure()
        ax = fig.gca(projection='3d')

        # Make data.
        X = np.arange(3, 8, 1)
        Y = np.arange(3, 8, 1)
        X, Y = np.meshgrid(X, Y)

        # Plot the surface.
        surf = ax.plot_surface(X, Y, DG_num_matrix, cmap=cm.coolwarm,
                            linewidth=0, antialiased=False)

        # Add a color bar which maps values to colors.
        fig.colorbar(surf, shrink=0.5, aspect=5)

        plt.savefig(my_path + "/pictures/DG_num_n%s.png"%OBJ_NUM)

        fig = plt.figure()
        ax = fig.gca(projection='3d')

        surf = ax.plot_surface(X, Y, min_feedback_matrix, cmap=cm.coolwarm,
                    linewidth=0, antialiased=False)
        
        # Add a color bar which maps values to colors.
        fig.colorbar(surf, shrink=0.5, aspect=5)

        plt.savefig(my_path + "/pictures/min_feedback_n%s.png"%OBJ_NUM)

    # In this experiment, I'm trying to see the relationship between the number of edges and DG quality.
    def edge_and_DG(self):
        Iteration_time = 10
        OBJ_NUM = 5
        M = np.zeros([21,21])
        for W_X in range(3,8):
            for W_Y in range(3,8):
                for iter in range(Iteration_time):
                    with open(os.path.join(my_path, "DG/W%s_H%s_n%s_iter%s_0301.pkl"%(W_X, W_Y, OBJ_NUM,iter)), 'rb') as input:
                           DGs = pickle.load(input)
                    for stat in DGs.values():
                        M[int(stat[1]),int(stat[2])] += 1
        
        A = np.zeros([1,11])
        for i in range(11):
            for j in range(21):
                A[0,i] += M[j,i]

        width = 0.2
        fig, ax = plt.subplots()

        p0 = ax.bar([x for x in range(11)], A[0,:].T, width)
        plt.xlabel("Minimum Feedback Arcs")
        plt.ylabel("Num")
        plt.savefig(my_path + "/pictures/arcs_distribution_n%s.png"%OBJ_NUM)



class DG_Space(object):
    def __init__(self, path_dict):
        self.path_dict = path_dict
        self.DG_space_construction()

    def DG_space_construction(self):
        self.DGs = {}
        objs = self.path_dict.keys()
        path_choices = [len(self.path_dict[obj])-1 for obj in objs]
        for path_set in self.choice_generator(path_choices):
            self.DGs[tuple(path_set)] = self.DG_construction( path_set)

    def choice_generator(self, path_choices):
        n = len(path_choices)
        choice = [0 for i in range(n)]
        while 1:
            pointer = n-1
            while (pointer > 0) and (choice[pointer] == path_choices[pointer]):
                pointer -= 1
            if (pointer == 0) and (choice[pointer] == path_choices[pointer]):
                yield path_choices
                break
            else:
                yield choice
                choice[pointer] += 1
                for i in range(pointer+1, n):
                    choice[i] = 0

    def DG_construction(self, path_set):
        n = len(path_set)
        M = np.zeros([n,n])
        total_num_constr = 0
        objs = self.path_dict.keys()
        for i in range(n):
            key = objs[i]
            total_num_constr += len(self.path_dict[key][path_set[i]])
            for constr in self.path_dict[key][path_set[i]]:
                if constr[1] == 0:
                    M[ i, objs.index(constr[0])] = 1
                else:
                    M[ objs.index(constr[0]), i] = 1
        num_constr = np.count_nonzero(M)
        num_feedback = self.count_feedback_arc(M)
        return [total_num_constr, num_constr, num_feedback]

    def count_feedback_arc(self,M):
        n = M.shape[0]
        m = gp.Model()
        
        m.setParam('OutputFlag', 0)
        m.modelSense=GRB.MINIMIZE

        y = m.addVars(n,n, vtype=GRB.BINARY)
        m.setObjective(sum(sum(M[k,j]*y[k,j] for k in range(j)) + sum(M[l,j]*(1-y[j,l]) for l in range(j+1, n)) for j in range(n)))
        for i in range(n):
            for j in range(i+1, n):
                for k in range(j+1, n):
                    m.addConstr(y[i,j]+y[j,k]-y[i,k]<=1)
                    m.addConstr(-y[i,j]-y[j,k]+y[i,k]<=0) 

        m.optimize()
        obj = m.getObjective()
        return obj.getValue()

class feekback_arc_ILP(object):
    def __init__(self, path_dict):
        self.path_dict = path_dict
        self.optimum = self.run_ILP()

    def run_ILP(self):
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
                        if (i,objs.index(constr[0])) not in dependency_dict:
                            dependency_dict[(i,objs.index(constr[0]))] = []
                        dependency_dict[(i,objs.index(constr[0]))].append([key, path_index])
                    else:
                        if not dependency_dict.has_key((objs.index(constr[0]), i)):
                            dependency_dict[( objs.index(constr[0]), i)] = []
                        dependency_dict[(objs.index(constr[0]), i)].append([key, path_index])

        m = gp.Model()
        
        m.setParam('OutputFlag', 0)
        m.modelSense=GRB.MINIMIZE

        y = m.addVars(n,n, vtype=GRB.BINARY)
        M = m.addVars(n,n, vtype=GRB.BINARY)
        x = m.addVars(n,MOST_PATH, vtype=GRB.BINARY)
        m.setObjective(sum(sum(M[k,j]*y[k,j] for k in range(j)) + sum(M[l,j]*(1-y[j,l]) for l in range(j+1, n)) for j in range(n)))
        
        for i in range(n):
            for j in range(i+1, n):
                for k in range(j+1, n):
                    m.addConstr(y[i,j]+y[j,k]-y[i,k]<=1)
                    m.addConstr(-y[i,j]-y[j,k]+y[i,k]<=0)
        for i in range(n):
            for j in range(len(self.path_dict[objs[i]]), MOST_PATH):
                m.addConstr(x[i,j]==0)
        for i in range(n):
            m.addConstr(sum(x[i,j] for j in range(MOST_PATH))==1)
        m.addConstrs(M[j,k]>=sum(x[objs.index(u),i] for (u,i) in dependency_dict[(j,k)] )/(2*MOST_PATH) for (j,k) in dependency_dict.keys())

        m.optimize()
        obj = m.getObjective()
        DG_index = []
        for i in range(n):
            for j in range(MOST_PATH):
                if x[i,j].x >0.5:
                    DG_index.append(j)
        print "DG_index", DG_index
        # print "path selection", x
        # print "order", y
        # print "DG", M
        return obj.getValue()

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
        self.plot_roadmap()
        self.construct_path_dict()

    def plot_instance(self):
        circles = []
        fig, ax = plt.subplots(figsize=(8, 8))
        for pose_key in self.goal_pose.keys():
            pose = self.goal_pose[pose_key]
            circles.append(plt.Circle(pose, self.radius, color = 'r'))
            plt.text(pose[0], pose[1], 'G_' + str(pose_key), dict(size=10))
        for pose_key in self.start_pose.keys():
            pose = self.start_pose[pose_key]
            circles.append(plt.Circle(pose, self.radius, color = 'b'))
            plt.text(pose[0], pose[1], 'S_' + str(pose_key), dict(size=10))
        for circle in circles:
            ax.add_artist(circle)
        plt.xlim((0-self.radius,self.XDIM+self.radius))
        plt.ylim((0-self.radius,self.YDIM+self.radius))
        plt.savefig(my_path + "/pictures/instance_test.png")
        # plt.show()

    def construct_path_dict(self):
        for key in self.start_pose.keys():
            self.pruning_search(key)
            if len(self.dependency_dict[key])==0:
                print "isolation occurs, key = ", key
                self.add_trivial_path(key)
    
    def add_trivial_path(self,key):
        pose1_loc = self.start_pose[key]
        pose2_loc = self.goal_pose[key]
        path = [(key,0), (key,1)]
        dependency_set = set()
        for pose in [(i,j) for i in self.start_pose.keys() for j in range(2)]:
            if (pose == (key,0)) or (pose == (key,0)):
                continue
            if pose[1]:
                pose_loc = self.goal_pose[pose[0]]
            else:
                pose_loc = self.start_pose[pose[0]]
            dist = self.find_dist_p_2_l(pose1_loc, pose2_loc, pose_loc)
            if (dist <2*self.radius):
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
        nodes[1] = (key,0)
        queue = [1]
        while len(queue) >0:
            node = queue.pop()
            if nodes[node] == (key,1):
                goal_nodes.append(node)
                continue
            if parents.has_key(node):
                pruning_set = pruning[parents[node]]
            else:
                pruning_set = {nodes[node]}
            if self.LL.has_key(nodes[node]):
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
            while parents.has_key(current_node):
                path.append(nodes[current_node])
                if self.pose_overlap.has_key(nodes[current_node]):
                    dependency_set = dependency_set.union(set(self.pose_overlap[nodes[current_node]]))
                dependency_set = dependency_set.union({nodes[current_node]})
                current_node = parents[current_node]
            path.append(current_node)
            if self.pose_overlap.has_key(nodes[current_node]):
                dependency_set = dependency_set.union(set(self.pose_overlap[nodes[current_node]]))
            dependency_set = dependency_set.union({nodes[current_node]})
            dependency_set = dependency_set.difference({(key,0),(key,1)})
            self.path_dict[key].append(path.reverse())
            self.dependency_dict[key].append(dependency_set)

    def check_overlap(self):
        self.pose_overlap = {}
        for s_key in self.start_pose.keys():
            s = self.start_pose[s_key]
            for g_key in self.goal_pose.keys():
                g = self.goal_pose[g_key]
                dist = math.sqrt((s[0]-g[0])**2+(s[1]-g[1])**2)
                if dist < 2* self.radius:
                    if not self.pose_overlap.has_key((s_key,0)):
                        self.pose_overlap[(s_key,0)] = []
                    if not self.pose_overlap.has_key((g_key,1)):
                        self.pose_overlap[(g_key,1)] = []
                    self.pose_overlap[(s_key,0)].append((g_key,1))
                    self.pose_overlap[(g_key,1)].append((s_key,0))
                
    def construct_roadmap(self):
        self.LL = {}
        for pose1 in [(i,j) for i in self.start_pose.keys() for j in range(2)]:
            for pose2 in [(i,j) for i in self.start_pose.keys() for j in range(2)]:
                if pose1 == pose2:
                    continue
                if pose2[0]<pose1[0]:
                    continue
                if (pose1[0] == pose2[0]) and (pose1[1]>pose2[1]):
                    continue
                if self.check_collision_without_overlap(pose1, pose2):
                    if not self.LL.has_key(pose1):
                        self.LL[pose1] = []
                    if not self.LL.has_key(pose2):
                        self.LL[pose2] = []
                    self.LL[pose1].append(pose2)
                    self.LL[pose2].append(pose1)
    
    def plot_roadmap(self):
        circles = []
        fig, ax = plt.subplots(figsize=(8, 8))
        for pose_key in self.goal_pose.keys():
            pose = self.goal_pose[pose_key]
            circles.append(plt.Circle(pose, self.radius, color = 'r'))
            plt.text(pose[0], pose[1], 'G_' + str(pose_key), dict(size=10))
        for pose_key in self.start_pose.keys():
            pose = self.start_pose[pose_key]
            circles.append(plt.Circle(pose, self.radius, color = 'b'))
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
        plt.xlim((0-self.radius,self.XDIM+self.radius))
        plt.ylim((0-self.radius,self.YDIM+self.radius))
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

        for pose in [(i,j) for i in self.start_pose.keys() for j in range(2)]:
            if (pose == pose1) or (pose == pose2):
                continue
            if (( (not self.pose_overlap.has_key(pose1)) or ((self.pose_overlap.has_key(pose1)) and (pose not in self.pose_overlap[pose1]))) and ((not self.pose_overlap.has_key(pose2)) or((self.pose_overlap.has_key(pose2)) and (pose not in self.pose_overlap[pose2])))):
                if pose[1]:
                    pose_loc = self.goal_pose[pose[0]]
                else:
                    pose_loc = self.start_pose[pose[0]]
                dist = self.find_dist_p_2_l(pose1_loc, pose2_loc, pose_loc)
                if (dist <2*self.radius):
                    return False
        return True
                  
    def find_dist_p_2_l(self, l1, l2, p):
        if l1 == l2:
            return(math.sqrt((l1[0]-p[0])**2 + (l1[1]-p[1])**2))
        alpha = ((l2[0]-l1[0])*(p[0]-l1[0])+(l2[1]-l1[1])*(p[1]-l1[1]))/((l1[0]-l2[0])**2 + (l1[1]-l2[1])**2)
        if alpha >= 1:
            alpha = 1
        if alpha <= 0:
            alpha = 0
        v = (alpha*l2[0]+(1-alpha)*l1[0], alpha*l2[1]+(1-alpha)*l1[1])
        return math.sqrt((v[0]-p[0])**2 + (v[1]-p[1])**2)

    def randomize_instance(self):
        obj_location = {}
        for i in range(self.n):
            index = i+1
            while 1:
                x = np.random.uniform()*self.XDIM
                y = np.random.uniform()*self.YDIM
                valid = True
                for obj in obj_location.values():
                    dist = math.sqrt((obj[0]-x)**2 + (obj[1]-y)**2)
                    if dist <= 2*self.radius:
                        valid = False
                        break
                if valid:
                    obj_location[index] = [x,y]
                    break
        return obj_location


###########################################################################################################


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

# DGs = DG_Space(path_dict)
# print DGs.DGs
# IP = feekback_arc_ILP(path_dict)
EXP = Experiments()
EXP.edge_and_DG()

