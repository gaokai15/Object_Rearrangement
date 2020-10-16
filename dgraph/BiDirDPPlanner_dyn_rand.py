from __future__ import division, print_function

import copy
import time
import numpy as np
from itertools import product
from random import sample, choice
from collections import OrderedDict
from operator import itemgetter, attrgetter

from dspace import genBuffers, Circle
from util import checkBitStatusAtPos
from ILPSolver import feedback_arc_ILP_buffers
from DG_Space import DFS_Rec_for_Monotone_General, linked_list_conversion


class BiDirDPPlanner(object):
    ### Input:
    ### (1) initial_arrangement (a list of pose_ids, each of which indicating the initial pose for an object)
    ### (2) final_arrangement (a list of pose_ids, each of which indicating the final pose for an object)
    ### (3) space (the dspace object which contains the obstacles, disks, region graph, and
    ### (i) workspace, (ii) object centers/slots, (iii) buffer centers/slots
    ### Output:
    ### the whole plan
    def __init__(self, init_arr, final_arr, space):
        self.space = space
        self.initial_arrangement = init_arr
        self.final_arrangement = final_arr
        self.numObjs = len(self.initial_arrangement)
        ### this number decides how many leafs we will love to add each time (braching factor)
        self.k = self.numObjs  # min(1, int(self.numObjs / 2))

        ### initialize dependency_dict and path_dict as empty dict
        ### since now we are going to increment these two dicts online, instead of offline
        self.dependency_dict = {}
        self.path_dict = {}
        # self.space.regionGraph()
        self.region_dict, self.linked_list = linked_list_conversion(self.space.RGAdj)
        self.object_locations = self.space.pose2reg

        self.treeL = {}
        self.treeR = {}
        self.trees = {}
        self.trees["Left"] = self.treeL
        self.trees["Right"] = self.treeR
        self.arrLeftRegistr = []
        self.arrRightRegistr = []
        self.idLeftRegistr = []
        self.idRightRegistr = []
        ### add the initial_arrangement and final_arrangement as the root node to two trees, respectively
        self.treeL["L0"] = ArrNode(self.initial_arrangement, "L0", None, None, None, 0, None)
        self.treeR["R0"] = ArrNode(self.final_arrangement, "R0", None, None, None, 0, None)
        self.arrLeftRegistr.append(self.initial_arrangement)
        self.arrRightRegistr.append(self.final_arrangement)
        self.idLeftRegistr.append("L0")
        self.idRightRegistr.append("R0")
        self.leftKey = "L0"
        self.rightKey = "R0"
        self.bridge = [None, None, None, None, None]
        ### [leftKey, rightKey, object_transition, objectMoved, path_option]
        self.leftLeaves = ["L0"]  ### keep track of leaves in the left tree
        self.rightLeaves = ["R0"]  ### keep track of leaves in the right tree

        ################## results ################
        self.isConnected = False
        self.best_solution_cost = np.inf
        ### the whole_path is a list of items and each item has the following format
        ### [("node1_id", node2_id), {2:path2, 1:path1, ...}]
        self.totalActions = 0  ### record the total number of actions
        self.numLeftBranches = 0  ### record the number of left branches in the solution
        self.numRightBranches = 0  ### record the number of right branches in the solution
        self.numNodesInLeftTree = 0  ### record the total number of nodes in the left tree
        self.numNodesInRightTree = 0  ### record the total number of nodes in the right tree

        ### start ruuning
        self.left_idx = 1
        self.right_idx = 1
        ### initial connection attempt

        self.growSubTree(self.treeL["L0"], self.treeR["R0"], "Left")
        if (self.isConnected != True):
            self.growSubTree(self.treeR["R0"], self.treeL["L0"], "Right")

        self.totalTime_allowed = 30 * self.numObjs  ### allow 30s per object for the total search
        start_time = time.clock()

        while (self.isConnected != True and time.clock() - start_time < self.totalTime_allowed):
            ### The problem is not monotone
            newChild_nodeID = self.mutateLeftChild()
            if newChild_nodeID != None:
                self.growSubTree(self.treeL[newChild_nodeID], self.treeR["R0"], "Left")
            if (self.isConnected != True):
                newChild_nodeID = self.mutateRightChild()
                if newChild_nodeID != None:
                    self.growSubTree(self.treeR[newChild_nodeID], self.treeL["L0"], "Right")

        # if self.isConnected:
        #     self.getTheStat()

        # if self.isConnected == False:
        #     print("failed to find a solution within " + str(self.totalTime_allowed) + " seconds...")

    def computeObjectsToMove(self, init_arrangement, final_arrangement):
        ### input: init_arrangement (a list of pose_idx)
        ###        final arrangement (a list of pose_idx)
        ### Output: objects_to_move (a list of obj_idx)
        objects_to_move = []
        for obj_idx in range(len(init_arrangement)):
            if init_arrangement[obj_idx] != final_arrangement[obj_idx]:
                objects_to_move.append(obj_idx)

        return objects_to_move

    def rankObjects(self, DG):
        ### get the objects inner degree and outer degree from DG
        objects_degree_dict = []  ### (obj_idx, inner_degree, outer_degree)
        numObjs = DG.shape[0]
        for obj_idx in range(numObjs):
            objects_degree_dict.append((obj_idx, sum(DG[:, obj_idx]), sum(DG[obj_idx, :])))

        # print("objects_degree_dict: ")
        # print(objects_degree_dict)

        objects_degree_dict = sorted(objects_degree_dict, key=itemgetter(2))
        objects_degree_dict = sorted(objects_degree_dict, key=itemgetter(1), reverse=True)
        # print("objects_degree_dict after: ")
        # print(objects_degree_dict)

        object_ranking = []
        for obj_info in objects_degree_dict[0:self.k + 1]:
            object_ranking.append(obj_info[0])

        # print("object_ranking: " + str(object_ranking))

        return object_ranking

    def object_dependency_opts_generate(self, query_arrangement, goal_arrangement, object_to_move, dependency_dict):
        num_objs = len(query_arrangement)
        object_dependency_opts = {}
        for obj_idx in range(num_objs):
            if obj_idx not in object_dependency_opts.keys():
                object_dependency_opts[obj_idx] = []
            ### look into the path option for the current object's current pose and destination pose
            # pose_key1 = min(query_arrangement[obj_idx], goal_arrangement[obj_idx])
            # pose_key2 = max(query_arrangement[obj_idx], goal_arrangement[obj_idx])
            pose_key1, pose_key2 = sorted((query_arrangement[obj_idx], goal_arrangement[obj_idx]))
            # if pose_key1 == pose_key2:
            #     path_set = set()
            #     if (constr in query_arrangement) and (query_arrangement.index(constr) != obj_idx):
            #         path_set.add((query_arrangement.index(constr), 0))
            #     if (constr in goal_arrangement) and (goal_arrangement.index(constr) != obj_idx):
            #         path_set.add((goal_arrangement.index(constr), 1))
            #     object_dependency_opts[obj_idx].append(path_set)
            # else:
            for path in dependency_dict[(pose_key1, pose_key2)]:
                path_set = set()
                for constr in path:
                    ### check the constraint based on query_arrangement and goal_arrangement
                    ### if the constraint is caused by current pose of an object
                    if (constr in query_arrangement) and (query_arrangement.index(constr) != obj_idx):
                        path_set.add((query_arrangement.index(constr), 0))
                    ### if the constraint is caused by final pose of an object
                    if (constr in goal_arrangement) and (goal_arrangement.index(constr) != obj_idx):
                        path_set.add((goal_arrangement.index(constr), 1))
                    ### Otherwise, it is a buffer and do nothing
                object_dependency_opts[obj_idx].append(path_set)

        return object_dependency_opts

    def choose_object(self, mutated_arrangement, goal_arrangement, dependency_dict):
        return range(self.numObjs)
        # print(dependency_dict)
        object_to_move_leaf2goal = self.computeObjectsToMove(mutated_arrangement, goal_arrangement)
        object_dependency_opts = self.object_dependency_opts_generate(
            mutated_arrangement,
            goal_arrangement,
            object_to_move_leaf2goal,
            dependency_dict,
        )
        IP_arc_buffers = feedback_arc_ILP_buffers(object_dependency_opts)
        arc_setSize, arcs, path_selection, object_ordering, DG, dependencyEdge_paths = IP_arc_buffers.optimum
        object_ranking = self.rankObjects(DG)
        # object_ranking = list(reversed(self.rankObjects(DG)))
        print("RANK: ", object_ranking)
        # return weighted ranking
        return sum([[x] * (len(object_ranking) - object_ranking.index(x)) for x in object_ranking], [])

    def choose_pose(self, obj_idx, mutated_arrangement):
        # PROB = 0.25
        # static_buffs = filter(lambda x: x[0] == 'B' and not x.find(';') < 0, self.space.poseMap.keys())
        bufs_for_obj = filter(lambda x: x[0] == 'B' and int(x.split(';')[-1][1:]) == obj_idx, self.space.poseMap.keys())
        # prob_adj = int(PROB * (len(static_buffs) + len(bufs_for_obj))) + 1

        # print(static_buffs)
        # print(bufs_for_obj)
        # print(prob_adj)
        # pose_idx = choice(bufs_for_obj + static_buffs + [None] * prob_adj)
        # print(pose_idx)

        # if pose_idx is None:
        ind = len(bufs_for_obj)
        # didgen = genBuffers(
        #     1,
        #     self.space,
        #     filter(lambda x: x != mutated_arrangement[obj_idx], mutated_arrangement),
        #     method='object_feasible',
        #     param1=mutated_arrangement[obj_idx],
        #     count=ind,
        #     suffix=';O' + str(obj_idx),
        # )
        didgen = genBuffers(
            1,
            self.space,
            filter(lambda x: x != mutated_arrangement[obj_idx], mutated_arrangement),
            method='random',
            # method='boundary_random',
            param1=50,
            count=ind,
            suffix=';O' + str(obj_idx),
        )
        pose_idx = 'B' + str(ind) + ';O' + str(obj_idx)

        # check if created?
        # bufs_for_obj = filter(lambda x: x[0] == 'B' and int(x.split(';')[-1][1:]) == obj_idx, self.space.poseMap.keys())
        # ind = len(bufs_for_obj)

        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))

        # self.space.regionGraph(lambda x: x[0] in mutated_arrangement + [pose_idx])
        # self.space.regionGraph()
        # self.region_dict, self.linked_list = linked_list_conversion(self.space.RGAdj)
        # self.object_locations = self.space.pose2reg

        if didgen:
            self.space.regionGraph(lambda x: x[0] in mutated_arrangement + [pose_idx])
            self.dependency_dict = {}
            self.path_dict = {}
            self.region_dict, self.linked_list = linked_list_conversion(self.space.RGAdj)
            self.object_locations = self.space.pose2reg
            return pose_idx
        else:
            return None

    def mutateRightChild(self):
        ### first choose a node to mutate
        # print("Right mutation")
        mutate_id = "R" + str(choice(range(len(self.treeR))))
        mutated_arrangement = self.treeR[mutate_id].arrangement
        deps_thusfar = self.treeR[mutate_id].dependency_dict
        ### choose an object to move
        objs = self.choose_object(mutated_arrangement, self.initial_arrangement, deps_thusfar)
        print(objs)
        obj_idx = choice(objs)  # choose from weighted list
        # obj_idx = choice(range(self.numObjs))
        ### choose a slot to put the object
        pose_idx = self.choose_pose(obj_idx, mutated_arrangement)

        # failed to generate buffer
        if pose_idx is None:
            print("failed to generate buffer!")
            return None

        ### get new arrangement
        new_arrangement = copy.deepcopy(mutated_arrangement)
        new_arrangement[obj_idx] = pose_idx
        # print("new_arrangement: " + str(new_arrangement))
        ### Let's check if it is already in the current tree
        if new_arrangement in self.arrRightRegistr:
            # print("The mutation makes a duplicate")
            return None
        ### Otherwise it is a new arrangement, check if it can be connected to the mutated_arrangement
        start_poses = {}
        goal_poses = {}
        for i in range(len(mutated_arrangement)):
            start_poses[i] = mutated_arrangement[i]
        for i in range(len(new_arrangement)):
            goal_poses[i] = new_arrangement[i]
        subTree = DFS_Rec_for_Monotone_General(
            start_poses,
            goal_poses,
            self.dependency_dict,
            self.path_dict,
            self.object_locations,
            self.linked_list,
            self.region_dict,
        )
        # ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        # else:

        ### we reach here since it is not a duplicate and it can be connected
        ### welcome this new arrangement
        # print("the new arrangement after mutation has been accepted")
        temp_transition = [new_arrangement[obj_idx], mutated_arrangement[obj_idx]]
        temp_object_idx = obj_idx
        temp_path_option = None  # subTree.path_option[subTree.parent.keys()[0]]
        temp_parent_cost = self.treeR[mutate_id].cost_to_come
        self.treeR["R" + str(self.right_idx)] = ArrNode(
            new_arrangement, "R" + str(self.right_idx), temp_transition, temp_object_idx, temp_path_option,
            temp_parent_cost + 1, mutate_id
        )
        self.arrRightRegistr.append(new_arrangement)
        self.idRightRegistr.append("R" + str(self.right_idx))
        self.right_idx += 1
        return self.idRightRegistr[self.arrRightRegistr.index(new_arrangement)]

    def mutateLeftChild(self):
        ### first choose a node to mutate
        # print("Left mutation")
        mutate_id = "L" + str(choice(range(len(self.treeL))))
        mutated_arrangement = self.treeL[mutate_id].arrangement
        deps_thusfar = self.treeL[mutate_id].dependency_dict
        ### choose an object to move
        objs = self.choose_object(mutated_arrangement, self.final_arrangement, deps_thusfar)
        print(objs)
        obj_idx = choice(objs)
        # obj_idx = choice(range(self.numObjs))
        ### choose a slot to put the object
        pose_idx = self.choose_pose(obj_idx, mutated_arrangement)

        # failed to generate buffer
        if pose_idx is None:
            print("failed to generate buffer!")
            return None

        ### get new arrangement
        new_arrangement = copy.deepcopy(mutated_arrangement)
        new_arrangement[obj_idx] = pose_idx
        # print("new_arrangement: " + str(new_arrangement))
        ### Let's check if it is already in the current tree
        if new_arrangement in self.arrLeftRegistr:
            # print("The mutation makes a duplicate")
            return None
        ### Otherwise it is a new arrangement, check if it can be connected to the mutated_arrangement
        start_poses = {}
        goal_poses = {}
        for i in range(len(mutated_arrangement)):
            start_poses[i] = mutated_arrangement[i]
        for i in range(len(new_arrangement)):
            goal_poses[i] = new_arrangement[i]
        subTree = DFS_Rec_for_Monotone_General(
            start_poses,
            goal_poses,
            self.dependency_dict,
            self.path_dict,
            self.object_locations,
            self.linked_list,
            self.region_dict,
        )
        # ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        # else:

        ### we reach here since it is a duplicate and it can be connected
        ### welcome this new arrangement
        # print("the new arrangement after mutation has been accepted")
        temp_transition = [mutated_arrangement[obj_idx], new_arrangement[obj_idx]]
        temp_object_idx = obj_idx
        temp_path_option = None  # subTree.path_option[subTree.parent.keys()[0]]
        temp_parent_cost = self.treeL[mutate_id].cost_to_come
        self.treeL["L" + str(self.left_idx)] = ArrNode(
            new_arrangement, "L" + str(self.left_idx), temp_transition, temp_object_idx, temp_path_option,
            temp_parent_cost + 1, mutate_id
        )
        self.arrLeftRegistr.append(new_arrangement)
        self.idLeftRegistr.append("L" + str(self.left_idx))
        self.left_idx += 1
        return self.idLeftRegistr[self.arrLeftRegistr.index(new_arrangement)]

    def growSubTree(self, initNode, goalNode, treeSide):
        ### construct start_poses and goal_poses
        start_poses = {}
        goal_poses = {}
        for i in range(len(initNode.arrangement)):
            start_poses[i] = initNode.arrangement[i]
        for i in range(len(goalNode.arrangement)):
            goal_poses[i] = goalNode.arrangement[i]

        all_poses = set(initNode.arrangement + goalNode.arrangement)
        # print(start_poses, goal_poses)
        # print(all_poses)
        # self.space.regionGraph()
        self.space.regionGraph(lambda x: x[0] in all_poses)
        self.dependency_dict = {}
        self.path_dict = {}
        self.region_dict, self.linked_list = linked_list_conversion(self.space.RGAdj)
        self.object_locations = self.space.pose2reg
        # self.getStraightPaths(all_poses)

        subTree = DFS_Rec_for_Monotone_General(
            start_poses,
            goal_poses,
            self.dependency_dict,
            self.path_dict,
            self.object_locations,
            self.linked_list,
            self.region_dict,
        )

        # initNode.updateDeps(subTree.dependency_dict)
        # print(initNode.dependency_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict

        if treeSide == "Left":
            self.engraftingLeftTree(subTree, initNode, goalNode)
        else:
            self.engraftingRightTree(subTree, initNode, goalNode)

    def engraftingRightTree(self, subTree, rootNode, goalNode):
        # print("Right tree")
        # print(subTree.parent)

        if len(subTree.parent) == 0:
            ### The tree does not exist
            return

        ### first construct a child dict
        child_dict = {}
        for child_id, parent_id in subTree.parent.items():
            if parent_id not in child_dict.keys():
                child_dict[parent_id] = []
            child_dict[parent_id].append(child_id)

        ### use a BFS to add the subTree to the entire tree structure
        queue = [0]
        while (len(queue) != 0):
            parent_id = queue.pop()
            parent_arrangement = self.encodeArrangement(parent_id, rootNode.arrangement, goalNode.arrangement)
            parent_nodeID = self.idRightRegistr[self.arrRightRegistr.index(parent_arrangement)]
            ### get all the children of this parent node
            if parent_id not in child_dict.keys():
                children_ids = []
            else:
                children_ids = child_dict[parent_id]
            for child_id in children_ids:
                child_arrangement = self.encodeArrangement(child_id, rootNode.arrangement, goalNode.arrangement)
                temp_object_idx = self.getTheObjectMoved(child_id, parent_id)
                temp_path_option = subTree.path_option[child_id]
                temp_transition = [child_arrangement[temp_object_idx], parent_arrangement[temp_object_idx]]
                ### check if this child arrangement has already in the tree
                if child_arrangement is self.arrRightRegistr:
                    ### we don't add duplicate nodes BUT we may rewire it to a better parent
                    child_nodeID = self.idRightRegistr[self.arrRightRegistr.index(child_arrangement)]
                    if self.treeR[parent_nodeID].cost_to_come + 1 < self.treeR[child_nodeID].cost_to_come:
                        ### It indicates that the current parent is a better parent since it costs less
                        ### update the corresponding infos for the child node
                        self.treeR[child_nodeID].updateParent(parent_nodeID)
                        self.treeR[child_nodeID].updateObjectTransition(temp_transition)
                        self.treeR[child_nodeID].updateObjectMoved(temp_object_idx)
                        self.treeR[child_nodeID].updatePathOption(temp_path_option)
                        self.treeR[child_nodeID].updateCostToCome(self.treeR[parent_nodeID].cost_to_come + 1)
                        # self.treeR[child_nodeID].updateDeps(subTree.dependency_dict)

                elif child_arrangement in self.arrLeftRegistr:
                    ### this is a sign that two trees are connected
                    ### check if it is really a bridge
                    if parent_arrangement not in self.arrLeftRegistr:
                        # print("a bridge is found")
                        self.isConnected = True
                        ### check if it leads to a better solution
                        temp_leftKey = self.idLeftRegistr[self.arrLeftRegistr.index(child_arrangement)]
                        temp_rightKey = self.idRightRegistr[self.arrRightRegistr.index(parent_arrangement)]
                        temp_cost = self.treeL[temp_leftKey].cost_to_come + self.treeR[temp_rightKey].cost_to_come + 1
                        if temp_cost < self.best_solution_cost:
                            ### This is a better solution
                            self.best_solution_cost = temp_cost
                            ### let's update the bridge
                            self.leftKey = temp_leftKey
                            self.rightKey = temp_rightKey
                            self.bridge_transition = temp_transition
                            self.bridge_objectMoved = temp_object_idx
                            self.bridge_path_option = temp_path_option
                            self.bridge = [
                                self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved,
                                self.bridge_path_option
                            ]

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeR[parent_nodeID].cost_to_come + 1
                    self.treeR["R" + str(self.right_idx)] = ArrNode(
                        child_arrangement, "R" + str(self.right_idx), temp_transition, temp_object_idx,
                        temp_path_option, temp_cost_to_come, parent_nodeID
                    )
                    # self.treeR["R" + str(self.right_idx)].updateDeps(subTree.dependency_dict)
                    self.arrRightRegistr.append(child_arrangement)
                    self.idRightRegistr.append("R" + str(self.right_idx))
                    self.right_idx += 1
                    ### add this child node into queue
                    queue.insert(0, child_id)

        # for i in self.treeR.keys():
        #     node = self.treeR[i]
        #     print(str(i) + ": " + str(node.arrangement) + ", " + str(node.object_transition) + ", " + \
        #         str(node.objectMoved) + ", " + str(node.cost_to_come) + ", " + str(node.parent_id))

    def engraftingLeftTree(self, subTree, rootNode, goalNode):
        # print("Left tree")
        # print(subTree.parent)

        if len(subTree.parent) == 0:
            ### The tree does not exist
            return

        ### first construct a child dict
        child_dict = {}
        for child_id, parent_id in subTree.parent.items():
            if parent_id not in child_dict.keys():
                child_dict[parent_id] = []
            child_dict[parent_id].append(child_id)

        ### use a BFS to add the subTree to the entire tree structure
        queue = [0]
        while (len(queue) != 0):
            parent_id = queue.pop()
            parent_arrangement = self.encodeArrangement(parent_id, rootNode.arrangement, goalNode.arrangement)
            parent_nodeID = self.idLeftRegistr[self.arrLeftRegistr.index(parent_arrangement)]
            ### get all the children of this parent node
            if parent_id not in child_dict.keys():
                children_ids = []
            else:
                children_ids = child_dict[parent_id]
            for child_id in children_ids:
                child_arrangement = self.encodeArrangement(child_id, rootNode.arrangement, goalNode.arrangement)
                temp_object_idx = self.getTheObjectMoved(child_id, parent_id)
                temp_path_option = subTree.path_option[child_id]
                temp_transition = [parent_arrangement[temp_object_idx], child_arrangement[temp_object_idx]]
                ### check if this child arrangement has already in the tree
                if child_arrangement is self.arrLeftRegistr:
                    ### we don't add duplicate nodes BUT we may rewire it to a better parent
                    child_nodeID = self.idLeftRegistr[self.arrLeftRegistr.index(child_arrangement)]
                    if self.treeL[parent_nodeID].cost_to_come + 1 < self.treeL[child_nodeID].cost_to_come:
                        ### It indicates that the current parent is a better parent since it costs less
                        ### update the corresponding infos for the child node
                        self.treeL[child_nodeID].updateParent(parent_nodeID)
                        self.treeL[child_nodeID].updateObjectTransition(temp_transition)
                        self.treeL[child_nodeID].updateObjectMoved(temp_object_idx)
                        self.treeL[child_nodeID].updatePathOption(temp_path_option)
                        self.treeL[child_nodeID].updateCostToCome(self.treeL[parent_nodeID].cost_to_come + 1)
                        # self.treeL[child_nodeID].updateDeps(subTree.dependency_dict)

                elif child_arrangement in self.arrRightRegistr:
                    ### this is a sign that two trees are connected
                    ### check if it is really a bridge
                    if parent_arrangement not in self.arrRightRegistr:
                        # print("a bridge is found")
                        self.isConnected = True
                        ### check if it leads to a better solution
                        temp_leftKey = self.idLeftRegistr[self.arrLeftRegistr.index(parent_arrangement)]
                        temp_rightKey = self.idRightRegistr[self.arrRightRegistr.index(child_arrangement)]
                        temp_cost = self.treeL[temp_leftKey].cost_to_come + self.treeR[temp_rightKey].cost_to_come + 1
                        if temp_cost < self.best_solution_cost:
                            ### This is a better solution
                            self.best_solution_cost = temp_cost
                            ### let's update the bridge
                            self.leftKey = temp_leftKey
                            self.rightKey = temp_rightKey
                            self.bridge_transition = temp_transition
                            self.bridge_objectMoved = temp_object_idx
                            self.bridge_path_option = temp_path_option
                            self.bridge = [
                                self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved,
                                self.bridge_path_option
                            ]

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeL[parent_nodeID].cost_to_come + 1
                    self.treeL["L" + str(self.left_idx)] = ArrNode(
                        child_arrangement, "L" + str(self.left_idx), temp_transition, temp_object_idx, temp_path_option,
                        temp_cost_to_come, parent_nodeID
                    )
                    # self.treeL["L" + str(self.left_idx)].updateDeps(subTree.dependency_dict)
                    self.arrLeftRegistr.append(child_arrangement)
                    self.idLeftRegistr.append("L" + str(self.left_idx))
                    self.left_idx += 1
                    ### add this child node into the queue
                    queue.insert(0, child_id)

        # for i in self.treeL.keys():
        #     node = self.treeL[i]
        #     print(str(i) + ": " + str(node.arrangement) + ", " + str(node.object_transition) + ", " + \
        #         str(node.objectMoved) + ", " + str(node.cost_to_come) + ", " + str(node.parent_id))

    def getTheObjectMoved(self, child_id, parent_id):
        for i in range(self.numObjs):
            bit_stat1 = checkBitStatusAtPos(child_id, i)
            bit_stat2 = checkBitStatusAtPos(parent_id, i)
            if (bit_stat1 != bit_stat2):
                ### since we know currently it will be just one object
                return i

        return None

    def encodeArrangement(self, new_node_id, root_arrangement, goal_arrangement):
        ### This function, based on number of objects in the current problem
        ### convert the node_id (int) into arrangement (a list of pose idx)
        new_arrangement = []
        for i in range(self.numObjs):
            isThatObjectInGoal = checkBitStatusAtPos(new_node_id, i)
            if isThatObjectInGoal:
                ### add the goal pose index
                new_arrangement.append(goal_arrangement[i])
            else:
                ### add the initial pose index
                new_arrangement.append(root_arrangement[i])

        return new_arrangement

    def getStraightPaths(self, poses):
        ### Before we perform search and increment the dependency and path dict
        ### Let's use the straight path as the first and backup path
        ### for each pair of pose
        # nPoses = len(self.space.poseMap)
        # for i in range(nPoses):
        #     for j in range(i, nPoses):
        for i, j in product(poses, repeat=2):
            key_pair = (i, j)
            # print("key_pair: " + str(key_pair))

            path = []
            dep_set = set()

            if key_pair not in self.dependency_dict.keys():
                self.dependency_dict[key_pair] = []
                self.path_dict[key_pair] = []

            if i == j:
                path.append(self.space.poseMap[i].center)
                self.path_dict[key_pair].append(path)
                dep_set.add(i)
                self.dependency_dict[key_pair].append(dep_set)
                continue

            # start_pt = self.points[i]
            # goal_pt = self.points[j]
            start_pt = self.space.poseMap[i].center
            goal_pt = self.space.poseMap[j].center
            nsegs = 20
            keypts_check = 5
            for kk in range(nsegs + 1):
                temp_ptx = start_pt[0] + (goal_pt[0] - start_pt[0]) / nsegs * kk
                temp_pty = start_pt[1] + (goal_pt[1] - start_pt[1]) / nsegs * kk
                temp_pt = (temp_ptx, temp_pty)
                ### check with every pose except its own start and goal pose
                for ii, o in self.space.poseMap.items():
                    if not Circle(o.center[0], o.center[1], o.radius + self.space.robot.radius).contains(temp_pt):
                        dep_set.add(ii)
                if kk % 5 == 0:
                    path.append(temp_pt)
            self.dependency_dict[key_pair].append(dep_set)
            self.path_dict[key_pair].append(path)

        # print("dependency_dict: ")
        # for key_pair, dependency_set in self.dependency_dict.items():
        #     print(str(key_pair) + ": " + str(dependency_set))

        # print("path_dict: ")
        # for key_pair, path in self.path_dict.items():
        #     print(str(key_pair) + ": " + str(path))

    def getTheStat(self):
        # print("Let's get stats!!")
        ### This function is used to get the statistics if the path is found
        self.numNodesInLeftTree = len(self.treeL)
        self.numNodesInRightTree = len(self.treeR)
        self.numLeftBranches = self.numNodesInLeftTree - 1
        self.numRightBranches = self.numNodesInRightTree - 1
        self.simplePath = []
        ### from leftKey, back track to left root via parent search
        curr_waypoint_id = self.leftKey
        self.simplePath.insert(0, curr_waypoint_id)
        while curr_waypoint_id != "L0":
            curr_waypoint_id = self.treeL[curr_waypoint_id].parent_id
            self.simplePath.insert(0, curr_waypoint_id)
        ### from rightKey, back track to right root via parent search
        curr_waypoint_id = self.rightKey
        self.simplePath.append(curr_waypoint_id)
        while curr_waypoint_id != "R0":
            curr_waypoint_id = self.treeR[curr_waypoint_id].parent_id
            self.simplePath.append(curr_waypoint_id)

        # print("path: " + str(self.simplePath))
        self.solution = []
        self.arrangements = []
        for nid in self.simplePath:
            if nid in self.treeL:
                self.arrangements.append(self.treeL[nid].arrangement)
            elif nid in self.treeR:
                self.arrangements.append(self.treeR[nid].arrangement)
            if nid == self.bridge[1]:
                self.solution.append((self.bridge[3], self.bridge[2]))
            if nid[1:] == '0':
                continue
            if nid in self.treeL:
                self.solution.append((self.treeL[nid].objectMoved, self.treeL[nid].object_transition))
            elif nid in self.treeR:
                self.solution.append((self.treeR[nid].objectMoved, self.treeR[nid].object_transition))
        # print("\nsolution path: " + str(self.solution))
        # print("bridge?: " + str(self.bridge))
        self.totalActions = len(self.simplePath) - 1
        # print("total action: " + str(self.totalActions))
        # print("solution cost: " + str(self.best_solution_cost))
        return self.solution


class ArrNode(object):
    def __init__(self, arrangement, node_id, object_transition, objectMoved, path_option, cost_to_come, parent_id):
        self.arrangement = arrangement
        self.node_id = node_id
        ### object_transition indicates the pose for certain objects before and after the transition
        ### For example, object 4 moves from pose 1 to pose 3, then object_transition = [1, 3]
        self.object_transition = object_transition
        self.objectMoved = objectMoved
        self.path_option = path_option
        self.cost_to_come = cost_to_come
        self.parent_id = parent_id
        self.dependency_dict = {}

    def updateObjectTransition(self, object_transition):
        self.object_transition = object_transition

    def updateObjectMoved(self, objectMoved):
        self.objectMoved = objectMoved

    def updatePathOption(self, path_option):
        self.path_option = path_option

    def updateCostToCome(self, cost_to_come):
        self.cost_to_come = cost_to_come

    def updateParent(self, parent_id):
        self.parent_id = parent_id

    # def updateDeps(self, depdict):
    #     self.dependency_dict = copy.deepcopy(depdict)

    def getParentArr(self):
        parent_arr = copy.deepcopy(self.arrangement)
        if self.node_id[0] == 'L':
            parent_arr[self.objectMoved] = self.object_transition[0]  ### move to a pose before transition
        else:
            parent_arr[self.objectMoved] = self.object_transition[1]  ### move to a pose after transition

        return parent_arr
