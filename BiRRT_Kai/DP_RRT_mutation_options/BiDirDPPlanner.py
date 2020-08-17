from __future__ import division


from DPLocalSolver import DFS_Rec_for_Monotone_General
from util import *
import copy
import IPython
import time
import random
import numpy as np
import math
from random import sample
from collections import OrderedDict
import sys
import os

# Disable
def blockPrint():
    sys.stdout = open(os.devnull, 'w')

# Restore
def enablePrint():
    sys.stdout = sys.__stdout__

class BiDirDPPlanner(object):
    ### Input:
    ### (1) initial_arrangement (a list of pose_ids, each of which indicating the initial pose for an object)
    ### (2) final_arrangement (a list of pose_ids, each of which indicating the final pose for an object)
    ### instance 
    ### (i) workspace, (ii) object centers/slots, (iii) buffer centers/slots
    ### visualTool: a visualization tool as a debugging purpose
    ### Output:
    ### the whole plan 
    def __init__(self, init_arr, final_arr, instance, Object_locations, region_dict, linked_list, visualTool):
        self.initial_arrangement = init_arr
        self.final_arrangement = final_arr
        self.points = instance.points + instance.buffer_points
        self.objects = instance.objects + instance.buffers
        self.numObjs = len(self.initial_arrangement)
        self.nPoses = len(self.points)
        self.allPoses = range(self.nPoses)
        # self.magicNumber = self.getMagicNumber(self.numObjs)
        # print("Magic number: " + str(self.magicNumber))

        ### initialize dependency_dict and path_dict as empty dict
        ### since now we are going to increment these two dicts online, instead of offline
        self.dependency_dict = {}
        self.path_dict = {}
        self.Object_locations = copy.deepcopy(Object_locations)
        self.region_dict = copy.deepcopy(region_dict)
        self.linked_list = copy.deepcopy(linked_list)

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
        self.bridge = [None, None, None, None, None] ### [leftKey, rightKey, object_transition, objectMoved, path_option]
        self.leftLeaves = ["L0"] ### keep track of leaves in the left tree
        self.rightLeaves = ["R0"] ### keep track of leaves in the right tree

        ################# testing #################
        self.num_mutation = 0
        self.mutation_time_list = []
        self.avg_time = 0.0
        self.total_time = 0.0

        ################## results ################
        self.isConnected = False
        self.best_solution_cost = np.inf
        ### the whole_path is a list of items and each item has the following format
        ### [("node1_id", node2_id), {2:path2, 1:path1, ...}]
        self.totalActions = 0 ### record the total number of actions
        self.numLeftBranches = 0 ### record the number of left branches in the solution
        self.numRightBranches = 0 ### record the number of right branches in the solution
        self.numNodesInLeftTree = 0 ### record the total number of nodes in the left tree
        self.numNodesInRightTree = 0 ### record the total number of nodes in the right tree

        ### start ruuning
        self.left_idx = 1
        self.right_idx = 1
        ### initial connection attempt

        self.growSubTree(self.treeL["L0"], self.treeR["R0"], "Left")
        if (self.isConnected != True):
            self.growSubTree(self.treeR["R0"], self.treeL["L0"], "Right")

        totalTime_allowed = 900.0 ### allow 500s for the total search tree construction
        start_time = time.clock()        

        while (self.isConnected != True and time.clock() - start_time < totalTime_allowed):
            ### The problem is not monotone
            start = time.time()
            newChild_nodeID = self.mutateLeftChild()
            if newChild_nodeID != None:
                self.growSubTree(self.treeL[newChild_nodeID], self.treeR["R0"], "Left")
            self.mutation_time_list.append(time.time()-start)
            self.num_mutation += 1


            start = time.time()
            if (self.isConnected != True):
                newChild_nodeID = self.mutateRightChild()
                if newChild_nodeID != None:
                    self.growSubTree(self.treeR[newChild_nodeID], self.treeL["L0"], "Right")
            self.mutation_time_list.append(time.time()-start)
            self.num_mutation += 1


        if self.isConnected:
            self.getTheStat()

        if self.isConnected == False:
            print("fail the find a solution within " + str(totalTime_allowed) + " seconds...")

    def mutateRightChild(self):
        ### first choose a node to mutate
        # print("Right mutation")
        mutate_id = "R" + str(random.choice(range(len(self.treeR))))
        mutated_arrangement = self.treeR[mutate_id].arrangement
        ### choose an object to move
        obj_idx = random.choice(range(self.numObjs))
        ### choose a slot to put the object
        pose_idx = random.choice(self.allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))

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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [new_arrangement[obj_idx], mutated_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeR[mutate_id].cost_to_come
            self.treeR["R"+str(self.right_idx)] = ArrNode(
                        new_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.arrRightRegistr.append(new_arrangement)
            self.idRightRegistr.append("R"+str(self.right_idx))
            self.right_idx += 1
            return self.idRightRegistr[self.arrRightRegistr.index(new_arrangement)]



    def mutateLeftChild(self):
        ### first choose a node to mutate
        # print("Left mutation")
        mutate_id = "L" + str(random.choice(range(len(self.treeL))))
        mutated_arrangement = self.treeL[mutate_id].arrangement
        ### choose an object to move
        obj_idx = random.choice(range(self.numObjs))
        ### choose a slot to put the object
        pose_idx = random.choice(self.allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))

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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [mutated_arrangement[obj_idx], new_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeL[mutate_id].cost_to_come
            self.treeL["L"+str(self.left_idx)] = ArrNode(
                        new_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.arrLeftRegistr.append(new_arrangement)
            self.idLeftRegistr.append("L"+str(self.left_idx))
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

        subTree = DFS_Rec_for_Monotone_General(
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
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
        while(len(queue) != 0):
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

                elif child_arrangement in self.arrLeftRegistr:
                    ### this is a sign that two trees are connected
                    ### check if it is really a bridge
                    if parent_arrangement not in self.arrLeftRegistr:
                        print("a bridge is found")
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
                            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeR[parent_nodeID].cost_to_come + 1
                    self.treeR["R"+str(self.right_idx)] = ArrNode(
                        child_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_cost_to_come, parent_nodeID)
                    self.arrRightRegistr.append(child_arrangement)
                    self.idRightRegistr.append("R"+str(self.right_idx))
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
        while(len(queue) != 0):
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

                elif child_arrangement in self.arrRightRegistr:
                    ### this is a sign that two trees are connected
                    ### check if it is really a bridge
                    if parent_arrangement not in self.arrRightRegistr:
                        print("a bridge is found")
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
                            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeL[parent_nodeID].cost_to_come + 1
                    self.treeL["L"+str(self.left_idx)] = ArrNode(
                        child_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_cost_to_come, parent_nodeID)
                    self.arrLeftRegistr.append(child_arrangement)
                    self.idLeftRegistr.append("L"+str(self.left_idx))
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

        print("path: " + str(self.simplePath))
        self.totalActions = len(self.simplePath) - 1
        print("total action: " + str(self.totalActions))
        print("solution cost: " + str(self.best_solution_cost)) 


    def getMagicNumber(self, numObjs):
        temp_str = ''
        for i in range(numObjs):
            temp_str += '1'

        return int(temp_str, 2)

class BiDirDPPlanner_A_star_furthest(object):
    ### Input:
    ### (1) initial_arrangement (a list of pose_ids, each of which indicating the initial pose for an object)
    ### (2) final_arrangement (a list of pose_ids, each of which indicating the final pose for an object)
    ### instance 
    ### (i) workspace, (ii) object centers/slots, (iii) buffer centers/slots
    ### visualTool: a visualization tool as a debugging purpose
    ### Output:
    ### the whole plan 
    def __init__(self, init_arr, final_arr, instance, Object_locations, region_dict, linked_list, points, RAD, visualTool):
        self.initial_arrangement = init_arr
        self.final_arrangement = final_arr
        self.points = instance.points + instance.buffer_points
        self.objects = instance.objects + instance.buffers
        self.numObjs = len(self.initial_arrangement)
        self.nPoses = len(self.points)
        self.allPoses = range(self.nPoses)
        # self.magicNumber = self.getMagicNumber(self.numObjs)
        # print("Magic number: " + str(self.magicNumber))

        ### initialize dependency_dict and path_dict as empty dict
        ### since now we are going to increment these two dicts online, instead of offline
        self.dependency_dict = {}
        self.path_dict = {}
        self.Object_locations = copy.deepcopy(Object_locations)
        self.region_dict = copy.deepcopy(region_dict)
        self.linked_list = copy.deepcopy(linked_list)

        self.treeL = {}
        self.treeR = {}
        self.trees = {}
        self.trees["Left"] = self.treeL
        self.trees["Right"] = self.treeR
        self.arrLeftRegistr = []
        self.arrRightRegistr = []
        self.idLeftRegistr = []
        self.idRightRegistr = []

        self.points = points
        self.RAD = RAD

        # candidates
        self.treeL_candidates = []
        self.treeR_candidates = []
        ### add the initial_arrangement and final_arrangement as the root node to two trees, respectively
        self.treeL["L0"] = ArrNode(self.initial_arrangement, "L0", None, None, None, 0, None)
        self.treeR["R0"] = ArrNode(self.final_arrangement, "R0", None, None, None, 0, None)
        self.update_candidates(self.treeL["L0"], self.treeR["R0"], "Left")
        self.update_candidates(self.treeR["R0"], self.treeL["L0"], "Right")
        self.arrLeftRegistr.append(self.initial_arrangement)
        self.arrRightRegistr.append(self.final_arrangement)
        self.idLeftRegistr.append("L0")
        self.idRightRegistr.append("R0")
        self.leftKey = "L0"
        self.rightKey = "R0"
        self.bridge = [None, None, None, None, None] ### [leftKey, rightKey, object_transition, objectMoved, path_option]
        self.leftLeaves = ["L0"] ### keep track of leaves in the left tree
        self.rightLeaves = ["R0"] ### keep track of leaves in the right tree

        ################# testing #################
        self.num_mutation = 0
        self.mutation_time_list = []
        self.avg_time = 0.0
        self.total_time = 0.0

        ################## results ################
        self.isConnected = False
        self.best_solution_cost = np.inf
        ### the whole_path is a list of items and each item has the following format
        ### [("node1_id", node2_id), {2:path2, 1:path1, ...}]
        self.totalActions = 0 ### record the total number of actions
        self.numLeftBranches = 0 ### record the number of left branches in the solution
        self.numRightBranches = 0 ### record the number of right branches in the solution
        self.numNodesInLeftTree = 0 ### record the total number of nodes in the left tree
        self.numNodesInRightTree = 0 ### record the total number of nodes in the right tree

        ### start ruuning
        self.left_idx = 1
        self.right_idx = 1
        ### initial connection attempt

        start = time.time()
        self.growSubTree(self.treeL["L0"], self.treeR["R0"], "Left")
        enablePrint()
        print "time:", time.time()-start
        blockPrint()
        if (self.isConnected != True):
            self.growSubTree(self.treeR["R0"], self.treeL["L0"], "Right")

        totalTime_allowed = 900.0 ### allow 500s for the total search tree construction
        start_time = time.clock()        

        w = 0.5 #### weight
        while (self.isConnected != True and time.clock() - start_time < totalTime_allowed):
            ### The problem is not monotone
            min_cost = float('inf')
            left_best_candidate = None
            right_best_candidate = None
            for left_cand in self.treeL_candidates:
                for right_cand in self.treeR_candidates:
                    diff, _ = self.potentialObstacles(self.treeL[left_cand[0]], self.treeR[right_cand[0]])
                    cost = self.treeL[left_cand[0]].cost_to_come + w * diff + self.treeR[right_cand[0]].cost_to_come
                    if min_cost > cost:
                        min_cost = cost
                        left_best_candidate = left_cand
                        right_best_candidate = right_cand
            if left_best_candidate == None:
                left_best_candidate = ("L" + str(random.choice(range(len(self.treeL)))), 0)
            elif (len(self.treeL_candidates)>=len(self.treeR_candidates)):
                self.treeL_candidates.remove(left_best_candidate)
            if right_best_candidate == None:
                right_best_candidate = ("R" + str(random.choice(range(len(self.treeR)))), 0)
            elif (len(self.treeL_candidates)<len(self.treeR_candidates)):
                self.treeR_candidates.remove(right_best_candidate)

            start = time.time()
            _, index = self.potentialObstacles(self.treeL[left_best_candidate[0]], self.treeR[right_best_candidate[0]])
            newChild_nodeID = self.improvedMutateLeftChild( index, left_best_candidate)
            if newChild_nodeID != None:
                self.growSubTree(self.treeL[newChild_nodeID], self.treeR[right_best_candidate[0]], "Left")
            self.mutation_time_list.append(time.time()-start)
            self.num_mutation += 1

            start = time.time()
            _, index = self.potentialObstacles(self.treeR[right_best_candidate[0]], self.treeL[left_best_candidate[0]])
            newChild_nodeID = self.improvedMutateRightChild( index, right_best_candidate)
            if newChild_nodeID != None:
                self.growSubTree(self.treeR[newChild_nodeID], self.treeL[left_best_candidate[0]], "Right")
            self.mutation_time_list.append(time.time()-start)
            self.num_mutation += 1
            
            # start = time.time()
            # newChild_nodeID = self.mutateLeftChild()
            # if newChild_nodeID != None:
            #     self.growSubTree(self.treeL[newChild_nodeID], self.treeR["R0"], "Left")
            # self.mutation_time_list.append(time.time()-start)
            # self.num_mutation += 1


            # start = time.time()
            # if (self.isConnected != True):
            #     newChild_nodeID = self.mutateRightChild()
            #     if newChild_nodeID != None:
            #         self.growSubTree(self.treeR[newChild_nodeID], self.treeL["L0"], "Right")
            # self.mutation_time_list.append(time.time()-start)
            # self.num_mutation += 1


        if self.isConnected:
            self.getTheStat()

        if self.isConnected == False:
            print("fail the find a solution within " + str(totalTime_allowed) + " seconds...")

    def update_candidates(self, new_node, root_node, treeside):
        num_candidate = 10

        if treeside == "Left":
            diff, _ = self.potentialObstacles(new_node, root_node)
            total_cost = diff + new_node.cost_to_come
            insert_index = -1
            for i in range(len(self.treeL_candidates)):
                if self.treeL_candidates[i][1] < total_cost:
                    continue
                else:
                    insert_index = i
                    break
            if insert_index == -1:
                if len(self.treeL_candidates) >= num_candidate:
                    return
                self.treeL_candidates.append((new_node.node_id, total_cost))
                return
            self.treeL_candidates.insert(insert_index, (new_node.node_id, total_cost))
            if len(self.treeL_candidates)>num_candidate:
                self.treeL_candidates.pop(-1)
                return

        if treeside == "Right":
            diff, _ = self.potentialObstacles(new_node, root_node)
            total_cost = diff + new_node.cost_to_come
            insert_index = -1
            for i in range(len(self.treeR_candidates)):
                if self.treeR_candidates[i][1] < total_cost:
                    continue
                else:
                    insert_index = i
                    break
            if insert_index == -1:
                if len(self.treeR_candidates) >= num_candidate:
                    return
                self.treeR_candidates.append((new_node.node_id, total_cost))
                return
            self.treeR_candidates.insert(insert_index, (new_node.node_id, total_cost))
            if len(self.treeR_candidates)>num_candidate:
                self.treeR_candidates.pop(-1)
                    
                


    def potentialObstacles(self, initNode, goalNode):
        diff = 0
        # potential collision times as obstacles to other objects
        collision_times = {}
        for i in range(len(initNode.arrangement)):
            collision_times[i] = 0
        
        for i in range(len(initNode.arrangement)):
            if initNode.arrangement[i] == goalNode.arrangement[i]:
                continue
            init_point = self.points[initNode.arrangement[i]]
            goal_point = self.points[goalNode.arrangement[i]]
            for j in range(len(initNode.arrangement)):
                if i == j:
                    continue
                obstacle = self.points[initNode.arrangement[j]]
                if self.collision_check(init_point, goal_point, obstacle):
                    diff += 0.5
                    collision_times[j] += 0.5
                obstacle = self.points[goalNode.arrangement[j]]
                if self.collision_check(init_point, goal_point, obstacle):
                    diff += 0.5
                    collision_times[j] += 0.5
        max_collision = 0
        most_collision_index = 0
        for key in collision_times.keys():
            if collision_times[key] > max_collision:
                max_collision = collision_times[key]
                most_collision_index = key
        return diff, most_collision_index
        
    def collision_check(self, init, goal, obst):
        dx = goal[0] - init[0]
        dy = goal[1] - init[1]
        # init->goal dyx-dxy+c1 = 0
        c1 = dx*init[1] - dy*init[0]
        # obstacle: dxx+dyy+c2 = 0
        c2 = -dx*init[0] -dy*init[1]

        # foot
        fx = (c1*dx-c2*dy)/(dx**2+dy**2)
        fy = (c2*dx-c1*dy)/(dx**2+dy**2)

        # alpha
        if fx == init[0]:
            alpha = (fy-init[1])/(goal[1]-init[1])
        else:
            alpha = (fx-init[0])/(goal[0]-init[0])
    
        if alpha>1.0:
            dist = math.sqrt((obst[0]-goal[0])**2 + (obst[1]-goal[1])**2)
        elif alpha <0.0:
            dist = math.sqrt((obst[0]-init[0])**2 + (obst[1]-init[1])**2)
        else:
            dist = math.sqrt((obst[0]-fx)**2 + (obst[1]-fy)**2)

        if dist < 2* self.RAD:
            return True
        else:
            return False



    def mutateRightChild(self):
        ### first choose a node to mutate
        # print("Right mutation")
        mutate_id = "R" + str(random.choice(range(len(self.treeR))))
        mutated_arrangement = self.treeR[mutate_id].arrangement
        ### choose an object to move
        obj_idx = random.choice(range(self.numObjs))
        ### choose a slot to put the object
        pose_idx = random.choice(self.allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))

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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [new_arrangement[obj_idx], mutated_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeR[mutate_id].cost_to_come
            self.treeR["R"+str(self.right_idx)] = ArrNode(
                        new_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.update_candidates(self.treeR["R"+str(self.right_idx)], self.treeL["L0"], "Right")
            self.arrRightRegistr.append(new_arrangement)
            self.idRightRegistr.append("R"+str(self.right_idx))
            self.right_idx += 1
            return self.idRightRegistr[self.arrRightRegistr.index(new_arrangement)]

    def mutateLeftChild(self):
        ### first choose a node to mutate
        # print("Left mutation")
        mutate_id = "L" + str(random.choice(range(len(self.treeL))))
        mutated_arrangement = self.treeL[mutate_id].arrangement
        ### choose an object to move
        obj_idx = random.choice(range(self.numObjs))
        ### choose a slot to put the object
        pose_idx = random.choice(self.allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))

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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [mutated_arrangement[obj_idx], new_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeL[mutate_id].cost_to_come
            self.treeL["L"+str(self.left_idx)] = ArrNode(
                        new_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.update_candidates(self.treeL["L"+str(self.left_idx)], self.treeR["R0"], "Left")
            self.arrLeftRegistr.append(new_arrangement)
            self.idLeftRegistr.append("L"+str(self.left_idx))
            self.left_idx += 1
            return self.idLeftRegistr[self.arrLeftRegistr.index(new_arrangement)]

    def improvedMutateRightChild(self, index, right_best_candidate):
        ### first choose a node to mutate
        # print("Left mutation")
        mutate_id = right_best_candidate[0]
        mutated_arrangement = self.treeR[mutate_id].arrangement
        ### choose an object to move
        obj_idx = index
        ### choose a slot to put the object
        pose_idx = self.furthest_pose(self.treeR[mutate_id].arrangement, obj_idx)

        if pose_idx == -1:
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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [mutated_arrangement[obj_idx], new_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeR[mutate_id].cost_to_come
            self.treeR["R"+str(self.right_idx)] = ArrNode(
                        new_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.update_candidates(self.treeR["R"+str(self.right_idx)], self.treeL["L0"], "Right")
            self.arrRightRegistr.append(new_arrangement)
            self.idRightRegistr.append("R"+str(self.right_idx))
            self.right_idx += 1

            ### Check whether the new arrangement is at the other side
            if new_arrangement in self.arrLeftRegistr:
                ### this is a sign that two trees are connected
                ### check if it is really a bridge
                print("a bridge is found")
                self.isConnected = True
                ### check if it leads to a better solution
                temp_rightKey = self.idRightRegistr[self.arrRightRegistr.index(self.treeR[mutate_id].arrangement)]
                temp_leftKey = self.idLeftRegistr[self.arrLeftRegistr.index(new_arrangement)]
                temp_cost = self.treeR[temp_rightKey].cost_to_come + self.treeL[temp_leftKey].cost_to_come + 1
                if temp_cost < self.best_solution_cost:
                    ### This is a better solution
                    self.best_solution_cost = temp_cost
                    ### let's update the bridge
                    self.leftKey = temp_leftKey
                    self.rightKey = temp_rightKey
                    self.bridge_transition = temp_transition
                    self.bridge_objectMoved = temp_object_idx
                    self.bridge_path_option = temp_path_option
                    self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]
                return None
                
            return self.idRightRegistr[self.arrRightRegistr.index(new_arrangement)]

    def improvedMutateLeftChild(self, index, left_best_candidate):
        ### first choose a node to mutate
        # print("Left mutation")
        mutate_id = left_best_candidate[0]
        mutated_arrangement = self.treeL[mutate_id].arrangement
        ### choose an object to move
        obj_idx = index
        ### choose a slot to put the object
        pose_idx = self.furthest_pose(self.treeL[mutate_id].arrangement, obj_idx)

        if pose_idx == -1:
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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [mutated_arrangement[obj_idx], new_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeL[mutate_id].cost_to_come
            self.treeL["L"+str(self.left_idx)] = ArrNode(
                        new_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.update_candidates(self.treeL["L"+str(self.left_idx)], self.treeR["R0"], "Left")
            self.arrLeftRegistr.append(new_arrangement)
            self.idLeftRegistr.append("L"+str(self.left_idx))
            self.left_idx += 1

            ### Check whether the new arrangement is at the other side
            if new_arrangement in self.arrRightRegistr:
                ### this is a sign that two trees are connected
                ### check if it is really a bridge
                print("a bridge is found")
                self.isConnected = True
                ### check if it leads to a better solution
                temp_leftKey = self.idLeftRegistr[self.arrLeftRegistr.index(self.treeL[mutate_id].arrangement)]
                temp_rightKey = self.idRightRegistr[self.arrRightRegistr.index(new_arrangement)]
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
                    self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]
                return None

            return self.idLeftRegistr[self.arrLeftRegistr.index(new_arrangement)]


    def furthest_pose(self, arrangement, obj_idx):
        occupied_poses = []
        for i in range(len(arrangement)):
            if i == obj_idx:
                continue
            occupied_poses.append(arrangement[i])
        
        Available_Regions = []
        for region in self.region_dict.keys():
            OCCUPIED = False
            for pose in region:
                if pose in occupied_poses:
                    OCCUPIED = True
                    break
            if not OCCUPIED:
                Available_Regions.append(self.region_dict[region])

        available_goals_dict = {}
        for i in self.allPoses:
            if (self.region_dict[self.Object_locations[i]] in Available_Regions):
                available_goals_dict[self.region_dict[self.Object_locations[i]]] = i
        
        furthest_available_pose = -1
        parents = {}
        explored = {}
        for key in self.region_dict.values():
            explored[key] = 0
        queue = [self.region_dict[self.Object_locations[arrangement[obj_idx]]]]
        explored[self.region_dict[self.Object_locations[arrangement[obj_idx]]]] = 1
        while (len(queue) >0):
            # stack(-1) for DFS and queue(0) for BFS
            old_node = queue.pop(-1)
            if old_node in self.linked_list:
                for region in self.linked_list[old_node]:
                    if explored[region]:
                        continue
                    if region not in Available_Regions:
                        continue
                    parents[region] = old_node
                    if region in available_goals_dict.keys():
                        furthest_available_pose = available_goals_dict[region]
                    queue.append(region)
                    explored[region] = 1
        return furthest_available_pose


    def growSubTree(self, initNode, goalNode, treeSide):
        ### construct start_poses and goal_poses
        start_poses = {}
        goal_poses = {}
        for i in range(len(initNode.arrangement)):
            start_poses[i] = initNode.arrangement[i]
        for i in range(len(goalNode.arrangement)):
            goal_poses[i] = goalNode.arrangement[i]

        subTree = DFS_Rec_for_Monotone_General(
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
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
        while(len(queue) != 0):
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
                        self.update_candidates(self.treeR[child_nodeID], self.treeL["L0"], "Right")

                elif child_arrangement in self.arrLeftRegistr:
                    ### this is a sign that two trees are connected
                    ### check if it is really a bridge
                    if parent_arrangement not in self.arrLeftRegistr:
                        print("a bridge is found")
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
                            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeR[parent_nodeID].cost_to_come + 1
                    self.treeR["R"+str(self.right_idx)] = ArrNode(
                        child_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_cost_to_come, parent_nodeID)
                    self.update_candidates(self.treeR["R"+str(self.right_idx)], self.treeL["L0"], "Right")
                    self.arrRightRegistr.append(child_arrangement)
                    self.idRightRegistr.append("R"+str(self.right_idx))
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
        while(len(queue) != 0):
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
                        self.update_candidates(self.treeL[child_nodeID], self.treeR["R0"], "Left")
                        print " "

                elif child_arrangement in self.arrRightRegistr:
                    ### this is a sign that two trees are connected
                    ### check if it is really a bridge
                    if parent_arrangement not in self.arrRightRegistr:
                        print("a bridge is found")
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
                            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeL[parent_nodeID].cost_to_come + 1
                    self.treeL["L"+str(self.left_idx)] = ArrNode(
                        child_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_cost_to_come, parent_nodeID)
                    self.update_candidates(self.treeL["L"+str(self.left_idx)], self.treeR["R0"], "Left")
                    self.arrLeftRegistr.append(child_arrangement)
                    self.idLeftRegistr.append("L"+str(self.left_idx))
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

        print("path: " + str(self.simplePath))
        self.totalActions = len(self.simplePath) - 1
        print("total action: " + str(self.totalActions))
        print("solution cost: " + str(self.best_solution_cost)) 


    def getMagicNumber(self, numObjs):
        temp_str = ''
        for i in range(numObjs):
            temp_str += '1'

        return int(temp_str, 2)

class BiDirDPPlanner_A_star_nearest(object):
    ### Input:
    ### (1) initial_arrangement (a list of pose_ids, each of which indicating the initial pose for an object)
    ### (2) final_arrangement (a list of pose_ids, each of which indicating the final pose for an object)
    ### instance 
    ### (i) workspace, (ii) object centers/slots, (iii) buffer centers/slots
    ### visualTool: a visualization tool as a debugging purpose
    ### Output:
    ### the whole plan 
    def __init__(self, init_arr, final_arr, instance, Object_locations, region_dict, linked_list, points, RAD, visualTool):
        self.initial_arrangement = init_arr
        self.final_arrangement = final_arr
        self.points = instance.points + instance.buffer_points
        self.objects = instance.objects + instance.buffers
        self.numObjs = len(self.initial_arrangement)
        self.nPoses = len(self.points)
        self.allPoses = range(self.nPoses)
        # self.magicNumber = self.getMagicNumber(self.numObjs)
        # print("Magic number: " + str(self.magicNumber))

        ### initialize dependency_dict and path_dict as empty dict
        ### since now we are going to increment these two dicts online, instead of offline
        self.dependency_dict = {}
        self.path_dict = {}
        self.Object_locations = copy.deepcopy(Object_locations)
        self.region_dict = copy.deepcopy(region_dict)
        self.linked_list = copy.deepcopy(linked_list)

        self.treeL = {}
        self.treeR = {}
        self.trees = {}
        self.trees["Left"] = self.treeL
        self.trees["Right"] = self.treeR
        self.arrLeftRegistr = []
        self.arrRightRegistr = []
        self.idLeftRegistr = []
        self.idRightRegistr = []

        self.points = points
        self.RAD = RAD

        # candidates
        self.treeL_candidates = []
        self.treeR_candidates = []
        ### add the initial_arrangement and final_arrangement as the root node to two trees, respectively
        self.treeL["L0"] = ArrNode(self.initial_arrangement, "L0", None, None, None, 0, None)
        self.treeR["R0"] = ArrNode(self.final_arrangement, "R0", None, None, None, 0, None)
        self.update_candidates(self.treeL["L0"], self.treeR["R0"], "Left")
        self.update_candidates(self.treeR["R0"], self.treeL["L0"], "Right")
        self.arrLeftRegistr.append(self.initial_arrangement)
        self.arrRightRegistr.append(self.final_arrangement)
        self.idLeftRegistr.append("L0")
        self.idRightRegistr.append("R0")
        self.leftKey = "L0"
        self.rightKey = "R0"
        self.bridge = [None, None, None, None, None] ### [leftKey, rightKey, object_transition, objectMoved, path_option]
        self.leftLeaves = ["L0"] ### keep track of leaves in the left tree
        self.rightLeaves = ["R0"] ### keep track of leaves in the right tree

        ################# testing #################
        self.num_mutation = 0
        self.mutation_time_list = []
        self.avg_time = 0.0
        self.total_time = 0.0

        ################## results ################
        self.isConnected = False
        self.best_solution_cost = np.inf
        ### the whole_path is a list of items and each item has the following format
        ### [("node1_id", node2_id), {2:path2, 1:path1, ...}]
        self.totalActions = 0 ### record the total number of actions
        self.numLeftBranches = 0 ### record the number of left branches in the solution
        self.numRightBranches = 0 ### record the number of right branches in the solution
        self.numNodesInLeftTree = 0 ### record the total number of nodes in the left tree
        self.numNodesInRightTree = 0 ### record the total number of nodes in the right tree

        ### start ruuning
        self.left_idx = 1
        self.right_idx = 1
        ### initial connection attempt

        self.growSubTree(self.treeL["L0"], self.treeR["R0"], "Left")
        if (self.isConnected != True):
            self.growSubTree(self.treeR["R0"], self.treeL["L0"], "Right")

        totalTime_allowed = 900.0 ### allow 500s for the total search tree construction
        start_time = time.clock()        

        w = 0.5 #### weight
        while (self.isConnected != True and time.clock() - start_time < totalTime_allowed):
            ### The problem is not monotone
            min_cost = float('inf')
            left_best_candidate = None
            right_best_candidate = None
            for left_cand in self.treeL_candidates:
                for right_cand in self.treeR_candidates:
                    diff, _ = self.potentialObstacles(self.treeL[left_cand[0]], self.treeR[right_cand[0]])
                    cost = self.treeL[left_cand[0]].cost_to_come + w*diff + self.treeR[right_cand[0]].cost_to_come
                    if min_cost > cost:
                        min_cost = cost
                        left_best_candidate = left_cand
                        right_best_candidate = right_cand
            if left_best_candidate == None:
                left_best_candidate = ("L" + str(random.choice(range(len(self.treeL)))), 0)
            elif (len(self.treeL_candidates) >= len(self.treeR_candidates)):
                self.treeL_candidates.remove(left_best_candidate)
            if right_best_candidate == None:
                right_best_candidate = ("R" + str(random.choice(range(len(self.treeR)))), 0)
            elif (len(self.treeL_candidates) < len(self.treeR_candidates)):
                self.treeR_candidates.remove(right_best_candidate)

            start = time.time()
            _, index = self.potentialObstacles(self.treeL[left_best_candidate[0]], self.treeR[right_best_candidate[0]])
            newChild_nodeID = self.improvedMutateLeftChild( index, left_best_candidate)
            if newChild_nodeID != None:
                self.growSubTree(self.treeL[newChild_nodeID], self.treeR[right_best_candidate[0]], "Left")
            self.mutation_time_list.append(time.time()-start)
            self.num_mutation += 1

            start = time.time()
            _, index = self.potentialObstacles(self.treeR[right_best_candidate[0]], self.treeL[left_best_candidate[0]])
            newChild_nodeID = self.improvedMutateRightChild( index, right_best_candidate)
            if newChild_nodeID != None:
                self.growSubTree(self.treeR[newChild_nodeID], self.treeL[left_best_candidate[0]], "Right")
            self.mutation_time_list.append(time.time()-start)
            self.num_mutation += 1
            
            # start = time.time()
            # newChild_nodeID = self.mutateLeftChild()
            # if newChild_nodeID != None:
            #     self.growSubTree(self.treeL[newChild_nodeID], self.treeR["R0"], "Left")
            # self.mutation_time_list.append(time.time()-start)
            # self.num_mutation += 1


            # start = time.time()
            # if (self.isConnected != True):
            #     newChild_nodeID = self.mutateRightChild()
            #     if newChild_nodeID != None:
            #         self.growSubTree(self.treeR[newChild_nodeID], self.treeL["L0"], "Right")
            # self.mutation_time_list.append(time.time()-start)
            # self.num_mutation += 1


        if self.isConnected:
            self.getTheStat()

        if self.isConnected == False:
            print("fail the find a solution within " + str(totalTime_allowed) + " seconds...")

    def update_candidates(self, new_node, root_node, treeside):
        num_candidate = 10

        if treeside == "Left":
            diff, _ = self.potentialObstacles(new_node, root_node)
            total_cost = diff + new_node.cost_to_come
            insert_index = -1
            for i in range(len(self.treeL_candidates)):
                if self.treeL_candidates[i][1] < total_cost:
                    continue
                else:
                    insert_index = i
                    break
            if insert_index == -1:
                if len(self.treeL_candidates) >= num_candidate:
                    return
                self.treeL_candidates.append((new_node.node_id, total_cost))
            self.treeL_candidates.insert(insert_index, (new_node.node_id, total_cost))
            if len(self.treeL_candidates)>num_candidate:
                self.treeL_candidates.pop(-1)

        if treeside == "Right":
            diff, _ = self.potentialObstacles(new_node, root_node)
            total_cost = diff + new_node.cost_to_come
            insert_index = -1
            for i in range(len(self.treeR_candidates)):
                if self.treeR_candidates[i][1] < total_cost:
                    continue
                else:
                    insert_index = i
                    break
            if insert_index == -1:
                if len(self.treeR_candidates) >= num_candidate:
                    return
                self.treeR_candidates.append((new_node.node_id, total_cost))
            self.treeR_candidates.insert(insert_index, (new_node.node_id, total_cost))
            if len(self.treeR_candidates)>num_candidate:
                self.treeR_candidates.pop(-1)
                    
                


    def potentialObstacles(self, initNode, goalNode):
        diff = 0
        # potential collision times as obstacles to other objects
        collision_times = {}
        for i in range(len(initNode.arrangement)):
            collision_times[i] = 0
        
        for i in range(len(initNode.arrangement)):
            if initNode.arrangement[i] == goalNode.arrangement[i]:
                continue
            init_point = self.points[initNode.arrangement[i]]
            goal_point = self.points[goalNode.arrangement[i]]
            for j in range(len(initNode.arrangement)):
                if i == j:
                    continue
                obstacle = self.points[initNode.arrangement[j]]
                if self.collision_check(init_point, goal_point, obstacle):
                    diff += 0.5
                    collision_times[j] += 0.5
                obstacle = self.points[goalNode.arrangement[j]]
                if self.collision_check(init_point, goal_point, obstacle):
                    diff += 0.5
                    collision_times[j] += 0.5
        max_collision = 0
        most_collision_index = 0
        for key in collision_times.keys():
            if collision_times[key] > max_collision:
                max_collision = collision_times[key]
                most_collision_index = key
        return diff, most_collision_index
        
    def collision_check(self, init, goal, obst):
        dx = goal[0] - init[0]
        dy = goal[1] - init[1]
        # init->goal dyx-dxy+c1 = 0
        c1 = dx*init[1] - dy*init[0]
        # obstacle: dxx+dyy+c2 = 0
        c2 = -dx*init[0] -dy*init[1]

        # foot
        fx = (c1*dx-c2*dy)/(dx**2+dy**2)
        fy = (c2*dx-c1*dy)/(dx**2+dy**2)

        # alpha
        if fx == init[0]:
            alpha = (fy-init[1])/(goal[1]-init[1])
        else:
            alpha = (fx-init[0])/(goal[0]-init[0])
    
        if alpha>1.0:
            dist = math.sqrt((obst[0]-goal[0])**2 + (obst[1]-goal[1])**2)
        elif alpha <0.0:
            dist = math.sqrt((obst[0]-init[0])**2 + (obst[1]-init[1])**2)
        else:
            dist = math.sqrt((obst[0]-fx)**2 + (obst[1]-fy)**2)

        if dist < 2* self.RAD:
            return True
        else:
            return False



    def mutateRightChild(self):
        ### first choose a node to mutate
        # print("Right mutation")
        mutate_id = "R" + str(random.choice(range(len(self.treeR))))
        mutated_arrangement = self.treeR[mutate_id].arrangement
        ### choose an object to move
        obj_idx = random.choice(range(self.numObjs))
        ### choose a slot to put the object
        pose_idx = random.choice(self.allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))

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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [new_arrangement[obj_idx], mutated_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeR[mutate_id].cost_to_come
            self.treeR["R"+str(self.right_idx)] = ArrNode(
                        new_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.update_candidates(self.treeR["R"+str(self.right_idx)], self.treeL["L0"], "Right")
            self.arrRightRegistr.append(new_arrangement)
            self.idRightRegistr.append("R"+str(self.right_idx))
            self.right_idx += 1
            return self.idRightRegistr[self.arrRightRegistr.index(new_arrangement)]

    def mutateLeftChild(self):
        ### first choose a node to mutate
        # print("Left mutation")
        mutate_id = "L" + str(random.choice(range(len(self.treeL))))
        mutated_arrangement = self.treeL[mutate_id].arrangement
        ### choose an object to move
        obj_idx = random.choice(range(self.numObjs))
        ### choose a slot to put the object
        pose_idx = random.choice(self.allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))

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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [mutated_arrangement[obj_idx], new_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeL[mutate_id].cost_to_come
            self.treeL["L"+str(self.left_idx)] = ArrNode(
                        new_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.update_candidates(self.treeL["L"+str(self.left_idx)], self.treeR["R0"], "Left")
            self.arrLeftRegistr.append(new_arrangement)
            self.idLeftRegistr.append("L"+str(self.left_idx))
            self.left_idx += 1
            return self.idLeftRegistr[self.arrLeftRegistr.index(new_arrangement)]

    def improvedMutateRightChild(self, index, left_best_candidate):
        ### first choose a node to mutate
        # print("Left mutation")
        mutate_id = left_best_candidate[0]
        mutated_arrangement = self.treeR[mutate_id].arrangement
        ### choose an object to move
        obj_idx = index
        ### choose a slot to put the object
        pose_idx = self.nearest_pose(self.treeR[mutate_id].arrangement, obj_idx)

        if pose_idx == -1:
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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [mutated_arrangement[obj_idx], new_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeR[mutate_id].cost_to_come
            self.treeR["R"+str(self.right_idx)] = ArrNode(
                        new_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.update_candidates(self.treeR["R"+str(self.right_idx)], self.treeL["L0"], "Right")
            self.arrRightRegistr.append(new_arrangement)
            self.idRightRegistr.append("R"+str(self.right_idx))
            self.right_idx += 1

            ### Check whether the new arrangement is at the other side
            if new_arrangement in self.arrLeftRegistr:
                ### this is a sign that two trees are connected
                ### check if it is really a bridge
                print("a bridge is found")
                self.isConnected = True
                ### check if it leads to a better solution
                temp_rightKey = self.idRightRegistr[self.arrRightRegistr.index(self.treeR[mutate_id].arrangement)]
                temp_leftKey = self.idLeftRegistr[self.arrLeftRegistr.index(new_arrangement)]
                temp_cost = self.treeL[temp_rightKey].cost_to_come + self.treeR[temp_leftKey].cost_to_come + 1
                if temp_cost < self.best_solution_cost:
                    ### This is a better solution
                    self.best_solution_cost = temp_cost
                    ### let's update the bridge
                    self.leftKey = temp_leftKey
                    self.rightKey = temp_rightKey
                    self.bridge_transition = temp_transition
                    self.bridge_objectMoved = temp_object_idx
                    self.bridge_path_option = temp_path_option
                    self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]
                return None
                
            return self.idRightRegistr[self.arrRightRegistr.index(new_arrangement)]

    def improvedMutateLeftChild(self, index, left_best_candidate):
        ### first choose a node to mutate
        # print("Left mutation")
        mutate_id = left_best_candidate[0]
        mutated_arrangement = self.treeL[mutate_id].arrangement
        ### choose an object to move
        obj_idx = index
        ### choose a slot to put the object
        pose_idx = self.nearest_pose(self.treeL[mutate_id].arrangement, obj_idx)

        if pose_idx == -1:
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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [mutated_arrangement[obj_idx], new_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeL[mutate_id].cost_to_come
            self.treeL["L"+str(self.left_idx)] = ArrNode(
                        new_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.update_candidates(self.treeL["L"+str(self.left_idx)], self.treeR["R0"], "Left")
            self.arrLeftRegistr.append(new_arrangement)
            self.idLeftRegistr.append("L"+str(self.left_idx))
            self.left_idx += 1

            ### Check whether the new arrangement is at the other side
            if new_arrangement in self.arrRightRegistr:
                ### this is a sign that two trees are connected
                ### check if it is really a bridge
                print("a bridge is found")
                self.isConnected = True
                ### check if it leads to a better solution
                temp_leftKey = self.idLeftRegistr[self.arrLeftRegistr.index(self.treeL[mutate_id].arrangement)]
                temp_rightKey = self.idRightRegistr[self.arrRightRegistr.index(new_arrangement)]
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
                    self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]
                return None

            return self.idLeftRegistr[self.arrLeftRegistr.index(new_arrangement)]


    def nearest_pose(self, arrangement, obj_idx):
        occupied_poses = []
        for i in range(len(arrangement)):
            if i == obj_idx:
                continue
            occupied_poses.append(arrangement[i])
        
        Available_Regions = []
        for region in self.region_dict.keys():
            OCCUPIED = False
            for pose in region:
                if pose in occupied_poses:
                    OCCUPIED = True
                    break
            if not OCCUPIED:
                Available_Regions.append(self.region_dict[region])

        available_goals_dict = {}
        for i in self.allPoses:
            if (self.region_dict[self.Object_locations[i]] in Available_Regions):
                available_goals_dict[self.region_dict[self.Object_locations[i]]] = i
        
        parents = {}
        explored = {}
        for key in self.region_dict.values():
            explored[key] = 0
        queue = [self.region_dict[self.Object_locations[arrangement[obj_idx]]]]
        explored[self.region_dict[self.Object_locations[arrangement[obj_idx]]]] = 1
        while (len(queue) >0):
            # stack(-1) for DFS and queue(0) for BFS
            old_node = queue.pop(-1)
            if old_node in self.linked_list:
                for region in self.linked_list[old_node]:
                    if explored[region]:
                        continue
                    if region not in Available_Regions:
                        continue
                    parents[region] = old_node
                    if region in available_goals_dict.keys():
                        return available_goals_dict[region]
                    queue.append(region)
                    explored[region] = 1
        return -1


    def growSubTree(self, initNode, goalNode, treeSide):
        ### construct start_poses and goal_poses
        start_poses = {}
        goal_poses = {}
        for i in range(len(initNode.arrangement)):
            start_poses[i] = initNode.arrangement[i]
        for i in range(len(goalNode.arrangement)):
            goal_poses[i] = goalNode.arrangement[i]

        subTree = DFS_Rec_for_Monotone_General(
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
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
        while(len(queue) != 0):
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
                        self.update_candidates(self.treeR[child_nodeID], self.treeL["L0"], "Right")

                elif child_arrangement in self.arrLeftRegistr:
                    ### this is a sign that two trees are connected
                    ### check if it is really a bridge
                    if parent_arrangement not in self.arrLeftRegistr:
                        print("a bridge is found")
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
                            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeR[parent_nodeID].cost_to_come + 1
                    self.treeR["R"+str(self.right_idx)] = ArrNode(
                        child_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_cost_to_come, parent_nodeID)
                    self.update_candidates(self.treeR["R"+str(self.right_idx)], self.treeL["L0"], "Right")
                    self.arrRightRegistr.append(child_arrangement)
                    self.idRightRegistr.append("R"+str(self.right_idx))
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
        while(len(queue) != 0):
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
                        self.update_candidates(self.treeL[child_nodeID], self.treeR["R0"], "Left")

                elif child_arrangement in self.arrRightRegistr:
                    ### this is a sign that two trees are connected
                    ### check if it is really a bridge
                    if parent_arrangement not in self.arrRightRegistr:
                        print("a bridge is found")
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
                            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeL[parent_nodeID].cost_to_come + 1
                    self.treeL["L"+str(self.left_idx)] = ArrNode(
                        child_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_cost_to_come, parent_nodeID)
                    self.update_candidates(self.treeL["L"+str(self.left_idx)], self.treeR["R0"], "Left")
                    self.arrLeftRegistr.append(child_arrangement)
                    self.idLeftRegistr.append("L"+str(self.left_idx))
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

        print("path: " + str(self.simplePath))
        self.totalActions = len(self.simplePath) - 1
        print("total action: " + str(self.totalActions))
        print("solution cost: " + str(self.best_solution_cost)) 


    def getMagicNumber(self, numObjs):
        temp_str = ''
        for i in range(numObjs):
            temp_str += '1'

        return int(temp_str, 2)

class BiDirDPPlanner_suboptimal_furthest(object):
    ### Input:
    ### (1) initial_arrangement (a list of pose_ids, each of which indicating the initial pose for an object)
    ### (2) final_arrangement (a list of pose_ids, each of which indicating the final pose for an object)
    ### instance 
    ### (i) workspace, (ii) object centers/slots, (iii) buffer centers/slots
    ### visualTool: a visualization tool as a debugging purpose
    ### Output:
    ### the whole plan 
    def __init__(self, init_arr, final_arr, instance, Object_locations, region_dict, linked_list, points, RAD, visualTool):
        self.initial_arrangement = init_arr
        self.final_arrangement = final_arr
        self.points = instance.points + instance.buffer_points
        self.objects = instance.objects + instance.buffers
        self.numObjs = len(self.initial_arrangement)
        self.nPoses = len(self.points)
        self.allPoses = range(self.nPoses)
        # self.magicNumber = self.getMagicNumber(self.numObjs)
        # print("Magic number: " + str(self.magicNumber))

        ### initialize dependency_dict and path_dict as empty dict
        ### since now we are going to increment these two dicts online, instead of offline
        self.dependency_dict = {}
        self.path_dict = {}
        self.Object_locations = copy.deepcopy(Object_locations)
        self.region_dict = copy.deepcopy(region_dict)
        self.linked_list = copy.deepcopy(linked_list)

        self.treeL = {}
        self.treeR = {}
        self.trees = {}
        self.trees["Left"] = self.treeL
        self.trees["Right"] = self.treeR
        self.arrLeftRegistr = []
        self.arrRightRegistr = []
        self.idLeftRegistr = []
        self.idRightRegistr = []

        self.points = points
        self.RAD = RAD

        # candidates
        self.treeL_candidates = []
        self.treeR_candidates = []
        ### add the initial_arrangement and final_arrangement as the root node to two trees, respectively
        self.treeL["L0"] = ArrNode(self.initial_arrangement, "L0", None, None, None, 0, None)
        self.treeR["R0"] = ArrNode(self.final_arrangement, "R0", None, None, None, 0, None)
        self.update_candidates(self.treeL["L0"], self.treeR["R0"], "Left")
        self.update_candidates(self.treeR["R0"], self.treeL["L0"], "Right")
        self.arrLeftRegistr.append(self.initial_arrangement)
        self.arrRightRegistr.append(self.final_arrangement)
        self.idLeftRegistr.append("L0")
        self.idRightRegistr.append("R0")
        self.leftKey = "L0"
        self.rightKey = "R0"
        self.bridge = [None, None, None, None, None] ### [leftKey, rightKey, object_transition, objectMoved, path_option]
        self.leftLeaves = ["L0"] ### keep track of leaves in the left tree
        self.rightLeaves = ["R0"] ### keep track of leaves in the right tree

        ################# testing #################
        self.num_mutation = 0
        self.mutation_time_list = []
        self.avg_time = 0.0
        self.total_time = 0.0

        ################## results ################
        self.isConnected = False
        self.best_solution_cost = np.inf
        ### the whole_path is a list of items and each item has the following format
        ### [("node1_id", node2_id), {2:path2, 1:path1, ...}]
        self.totalActions = 0 ### record the total number of actions
        self.numLeftBranches = 0 ### record the number of left branches in the solution
        self.numRightBranches = 0 ### record the number of right branches in the solution
        self.numNodesInLeftTree = 0 ### record the total number of nodes in the left tree
        self.numNodesInRightTree = 0 ### record the total number of nodes in the right tree

        ### start ruuning
        self.left_idx = 1
        self.right_idx = 1
        ### initial connection attempt

        self.growSubTree(self.treeL["L0"], self.treeR["R0"], "Left")
        if (self.isConnected != True):
            self.growSubTree(self.treeR["R0"], self.treeL["L0"], "Right")

        totalTime_allowed = 900.0 ### allow 500s for the total search tree construction
        start_time = time.clock()        

        while (self.isConnected != True and time.clock() - start_time < totalTime_allowed):
            ### The problem is not monotone
            if len(self.treeL_candidates) == 0:
                left_best_candidate = ("L" + str(random.choice(range(len(self.treeL)))), 0)
            elif (len(self.treeL_candidates) >= len(self.treeR_candidates)):
                left_best_candidate = self.treeL_candidates.pop(0)
            else:
                left_best_candidate = self.treeL_candidates[0]
            if len(self.treeR_candidates) == 0:
                right_best_candidate = ("R" + str(random.choice(range(len(self.treeR)))), 0)
            elif (len(self.treeL_candidates) < len(self.treeR_candidates)):
                right_best_candidate = self.treeR_candidates.pop(0)
            else:
                right_best_candidate = self.treeR_candidates[0]

            start = time.time()
            _, index = self.potentialObstacles(self.treeL[left_best_candidate[0]], self.treeR["R0"])
            newChild_nodeID = self.improvedMutateLeftChild( index, left_best_candidate)
            if newChild_nodeID != None:
                self.growSubTree(self.treeL[newChild_nodeID], self.treeR["R0"], "Left")
            self.mutation_time_list.append(time.time()-start)
            self.num_mutation += 1

            start = time.time()
            _, index = self.potentialObstacles(self.treeR[right_best_candidate[0]], self.treeL["L0"])
            newChild_nodeID = self.improvedMutateRightChild( index, right_best_candidate)
            if newChild_nodeID != None:
                self.growSubTree(self.treeR[newChild_nodeID], self.treeL["L0"], "Right")
            self.mutation_time_list.append(time.time()-start)
            self.num_mutation += 1
            
            # start = time.time()
            # newChild_nodeID = self.mutateLeftChild()
            # if newChild_nodeID != None:
            #     self.growSubTree(self.treeL[newChild_nodeID], self.treeR["R0"], "Left")
            # self.mutation_time_list.append(time.time()-start)
            # self.num_mutation += 1


            # start = time.time()
            # if (self.isConnected != True):
            #     newChild_nodeID = self.mutateRightChild()
            #     if newChild_nodeID != None:
            #         self.growSubTree(self.treeR[newChild_nodeID], self.treeL["L0"], "Right")
            # self.mutation_time_list.append(time.time()-start)
            # self.num_mutation += 1


        if self.isConnected:
            self.getTheStat()

        if self.isConnected == False:
            print("fail the find a solution within " + str(totalTime_allowed) + " seconds...")

    def update_candidates(self, new_node, root_node, treeside):
        num_candidate = 10

        if treeside == "Left":
            diff, _ = self.potentialObstacles(new_node, root_node)
            total_cost = diff + new_node.cost_to_come
            insert_index = -1
            for i in range(len(self.treeL_candidates)):
                if self.treeL_candidates[i][1] < total_cost:
                    continue
                else:
                    insert_index = i
                    break
            if insert_index == -1:
                if len(self.treeL_candidates) >= num_candidate:
                    return
                self.treeL_candidates.append((new_node.node_id, total_cost))
            self.treeL_candidates.insert(insert_index, (new_node.node_id, total_cost))
            if len(self.treeL_candidates)>num_candidate:
                self.treeL_candidates.pop(-1)

        if treeside == "Right":
            diff, _ = self.potentialObstacles(new_node, root_node)
            total_cost = diff + new_node.cost_to_come
            insert_index = -1
            for i in range(len(self.treeR_candidates)):
                if self.treeR_candidates[i][1] < total_cost:
                    continue
                else:
                    insert_index = i
                    break
            if insert_index == -1:
                if len(self.treeR_candidates) >= num_candidate:
                    return
                self.treeR_candidates.append((new_node.node_id, total_cost))
            self.treeR_candidates.insert(insert_index, (new_node.node_id, total_cost))
            if len(self.treeR_candidates)>num_candidate:
                self.treeR_candidates.pop(-1)
                    
                


    def potentialObstacles(self, initNode, goalNode):
        diff = 0
        # potential collision times as obstacles to other objects
        collision_times = {}
        for i in range(len(initNode.arrangement)):
            collision_times[i] = 0
        
        for i in range(len(initNode.arrangement)):
            if initNode.arrangement[i] == goalNode.arrangement[i]:
                continue
            init_point = self.points[initNode.arrangement[i]]
            goal_point = self.points[goalNode.arrangement[i]]
            for j in range(len(initNode.arrangement)):
                if i == j:
                    continue
                obstacle = self.points[initNode.arrangement[j]]
                if self.collision_check(init_point, goal_point, obstacle):
                    diff += 0.5
                    collision_times[j] += 0.5
                obstacle = self.points[goalNode.arrangement[j]]
                if self.collision_check(init_point, goal_point, obstacle):
                    diff += 0.5
                    collision_times[j] += 0.5
        max_collision = 0
        most_collision_index = 0
        for key in collision_times.keys():
            if collision_times[key] > max_collision:
                max_collision = collision_times[key]
                most_collision_index = key
        return diff, most_collision_index
        
    def collision_check(self, init, goal, obst):
        dx = goal[0] - init[0]
        dy = goal[1] - init[1]
        # init->goal dyx-dxy+c1 = 0
        c1 = dx*init[1] - dy*init[0]
        # obstacle: dxx+dyy+c2 = 0
        c2 = -dx*init[0] -dy*init[1]

        # foot
        fx = (c1*dx-c2*dy)/(dx**2+dy**2)
        fy = (c2*dx-c1*dy)/(dx**2+dy**2)

        # alpha
        if fx == init[0]:
            alpha = (fy-init[1])/(goal[1]-init[1])
        else:
            alpha = (fx-init[0])/(goal[0]-init[0])
    
        if alpha>1.0:
            dist = math.sqrt((obst[0]-goal[0])**2 + (obst[1]-goal[1])**2)
        elif alpha <0.0:
            dist = math.sqrt((obst[0]-init[0])**2 + (obst[1]-init[1])**2)
        else:
            dist = math.sqrt((obst[0]-fx)**2 + (obst[1]-fy)**2)

        if dist < 2* self.RAD:
            return True
        else:
            return False



    def mutateRightChild(self):
        ### first choose a node to mutate
        # print("Right mutation")
        mutate_id = "R" + str(random.choice(range(len(self.treeR))))
        mutated_arrangement = self.treeR[mutate_id].arrangement
        ### choose an object to move
        obj_idx = random.choice(range(self.numObjs))
        ### choose a slot to put the object
        pose_idx = random.choice(self.allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))

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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [new_arrangement[obj_idx], mutated_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeR[mutate_id].cost_to_come
            self.treeR["R"+str(self.right_idx)] = ArrNode(
                        new_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.update_candidates(self.treeR["R"+str(self.right_idx)], self.treeL["L0"], "Right")
            self.arrRightRegistr.append(new_arrangement)
            self.idRightRegistr.append("R"+str(self.right_idx))
            self.right_idx += 1
            return self.idRightRegistr[self.arrRightRegistr.index(new_arrangement)]

    def mutateLeftChild(self):
        ### first choose a node to mutate
        # print("Left mutation")
        mutate_id = "L" + str(random.choice(range(len(self.treeL))))
        mutated_arrangement = self.treeL[mutate_id].arrangement
        ### choose an object to move
        obj_idx = random.choice(range(self.numObjs))
        ### choose a slot to put the object
        pose_idx = random.choice(self.allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))

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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [mutated_arrangement[obj_idx], new_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeL[mutate_id].cost_to_come
            self.treeL["L"+str(self.left_idx)] = ArrNode(
                        new_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.update_candidates(self.treeL["L"+str(self.left_idx)], self.treeR["R0"], "Left")
            self.arrLeftRegistr.append(new_arrangement)
            self.idLeftRegistr.append("L"+str(self.left_idx))
            self.left_idx += 1
            return self.idLeftRegistr[self.arrLeftRegistr.index(new_arrangement)]

    def improvedMutateRightChild(self, index, left_best_candidate):
        ### first choose a node to mutate
        # print("Left mutation")
        mutate_id = left_best_candidate[0]
        mutated_arrangement = self.treeR[mutate_id].arrangement
        ### choose an object to move
        obj_idx = index
        ### choose a slot to put the object
        pose_idx = self.furthest_pose(self.treeR[mutate_id].arrangement, obj_idx)

        if pose_idx == -1:
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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [mutated_arrangement[obj_idx], new_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeR[mutate_id].cost_to_come
            self.treeR["R"+str(self.right_idx)] = ArrNode(
                        new_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.update_candidates(self.treeR["R"+str(self.right_idx)], self.treeL["L0"], "Right")
            self.arrRightRegistr.append(new_arrangement)
            self.idRightRegistr.append("R"+str(self.right_idx))
            self.right_idx += 1

            ### Check whether the new arrangement is at the other side
            if new_arrangement in self.arrLeftRegistr:
                ### this is a sign that two trees are connected
                ### check if it is really a bridge
                print("a bridge is found")
                self.isConnected = True
                ### check if it leads to a better solution
                temp_rightKey = self.idRightRegistr[self.arrRightRegistr.index(self.treeR[mutate_id].arrangement)]
                temp_leftKey = self.idLeftRegistr[self.arrLeftRegistr.index(new_arrangement)]
                temp_cost = self.treeL[temp_rightKey].cost_to_come + self.treeR[temp_leftKey].cost_to_come + 1
                if temp_cost < self.best_solution_cost:
                    ### This is a better solution
                    self.best_solution_cost = temp_cost
                    ### let's update the bridge
                    self.leftKey = temp_leftKey
                    self.rightKey = temp_rightKey
                    self.bridge_transition = temp_transition
                    self.bridge_objectMoved = temp_object_idx
                    self.bridge_path_option = temp_path_option
                    self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]
                return None
                
            return self.idRightRegistr[self.arrRightRegistr.index(new_arrangement)]

    def improvedMutateLeftChild(self, index, left_best_candidate):
        ### first choose a node to mutate
        # print("Left mutation")
        mutate_id = left_best_candidate[0]
        mutated_arrangement = self.treeL[mutate_id].arrangement
        ### choose an object to move
        obj_idx = index
        ### choose a slot to put the object
        pose_idx = self.furthest_pose(self.treeL[mutate_id].arrangement, obj_idx)

        if pose_idx == -1:
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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [mutated_arrangement[obj_idx], new_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeL[mutate_id].cost_to_come
            self.treeL["L"+str(self.left_idx)] = ArrNode(
                        new_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.update_candidates(self.treeL["L"+str(self.left_idx)], self.treeR["R0"], "Left")
            self.arrLeftRegistr.append(new_arrangement)
            self.idLeftRegistr.append("L"+str(self.left_idx))
            self.left_idx += 1

            ### Check whether the new arrangement is at the other side
            if new_arrangement in self.arrRightRegistr:
                ### this is a sign that two trees are connected
                ### check if it is really a bridge
                print("a bridge is found")
                self.isConnected = True
                ### check if it leads to a better solution
                temp_leftKey = self.idLeftRegistr[self.arrLeftRegistr.index(self.treeL[mutate_id].arrangement)]
                temp_rightKey = self.idRightRegistr[self.arrRightRegistr.index(new_arrangement)]
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
                    self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]
                return None

            return self.idLeftRegistr[self.arrLeftRegistr.index(new_arrangement)]


    def furthest_pose(self, arrangement, obj_idx):
        occupied_poses = []
        for i in range(len(arrangement)):
            if i == obj_idx:
                continue
            occupied_poses.append(arrangement[i])
        
        Available_Regions = []
        for region in self.region_dict.keys():
            OCCUPIED = False
            for pose in region:
                if pose in occupied_poses:
                    OCCUPIED = True
                    break
            if not OCCUPIED:
                Available_Regions.append(self.region_dict[region])

        available_goals_dict = {}
        for i in self.allPoses:
            if (self.region_dict[self.Object_locations[i]] in Available_Regions):
                available_goals_dict[self.region_dict[self.Object_locations[i]]] = i
        
        furthest_available_pose = -1
        parents = {}
        explored = {}
        for key in self.region_dict.values():
            explored[key] = 0
        queue = [self.region_dict[self.Object_locations[arrangement[obj_idx]]]]
        explored[self.region_dict[self.Object_locations[arrangement[obj_idx]]]] = 1
        while (len(queue) >0):
            # stack(-1) for DFS and queue(0) for BFS
            old_node = queue.pop(-1)
            if old_node in self.linked_list:
                for region in self.linked_list[old_node]:
                    if explored[region]:
                        continue
                    if region not in Available_Regions:
                        continue
                    parents[region] = old_node
                    if region in available_goals_dict.keys():
                        furthest_available_pose = available_goals_dict[region]
                    queue.append(region)
                    explored[region] = 1
        return furthest_available_pose


    def growSubTree(self, initNode, goalNode, treeSide):
        ### construct start_poses and goal_poses
        start_poses = {}
        goal_poses = {}
        for i in range(len(initNode.arrangement)):
            start_poses[i] = initNode.arrangement[i]
        for i in range(len(goalNode.arrangement)):
            goal_poses[i] = goalNode.arrangement[i]

        subTree = DFS_Rec_for_Monotone_General(
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
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
        while(len(queue) != 0):
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
                        self.update_candidates(self.treeR[child_nodeID], self.treeL["L0"], "Right")

                elif child_arrangement in self.arrLeftRegistr:
                    ### this is a sign that two trees are connected
                    ### check if it is really a bridge
                    if parent_arrangement not in self.arrLeftRegistr:
                        print("a bridge is found")
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
                            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeR[parent_nodeID].cost_to_come + 1
                    self.treeR["R"+str(self.right_idx)] = ArrNode(
                        child_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_cost_to_come, parent_nodeID)
                    self.update_candidates(self.treeR["R"+str(self.right_idx)], self.treeL["L0"], "Right")
                    self.arrRightRegistr.append(child_arrangement)
                    self.idRightRegistr.append("R"+str(self.right_idx))
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
        while(len(queue) != 0):
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
                        self.update_candidates(self.treeL[child_nodeID], self.treeR["R0"], "Left")

                elif child_arrangement in self.arrRightRegistr:
                    ### this is a sign that two trees are connected
                    ### check if it is really a bridge
                    if parent_arrangement not in self.arrRightRegistr:
                        print("a bridge is found")
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
                            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeL[parent_nodeID].cost_to_come + 1
                    self.treeL["L"+str(self.left_idx)] = ArrNode(
                        child_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_cost_to_come, parent_nodeID)
                    self.update_candidates(self.treeL["L"+str(self.left_idx)], self.treeR["R0"], "Left")
                    self.arrLeftRegistr.append(child_arrangement)
                    self.idLeftRegistr.append("L"+str(self.left_idx))
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

        print("path: " + str(self.simplePath))
        self.totalActions = len(self.simplePath) - 1
        print("total action: " + str(self.totalActions))
        print("solution cost: " + str(self.best_solution_cost)) 


    def getMagicNumber(self, numObjs):
        temp_str = ''
        for i in range(numObjs):
            temp_str += '1'

        return int(temp_str, 2)


class BiDirDPPlanner_Random_Range(object):
    ### Input:
    ### (1) initial_arrangement (a list of pose_ids, each of which indicating the initial pose for an object)
    ### (2) final_arrangement (a list of pose_ids, each of which indicating the final pose for an object)
    ### instance 
    ### (i) workspace, (ii) object centers/slots, (iii) buffer centers/slots
    ### visualTool: a visualization tool as a debugging purpose
    ### Output:
    ### the whole plan 
    def __init__(self, init_arr, final_arr, instance, Object_locations, region_dict, linked_list, visualTool):
        self.initial_arrangement = init_arr
        self.final_arrangement = final_arr
        self.points = instance.points + instance.buffer_points
        self.objects = instance.objects + instance.buffers
        self.numObjs = len(self.initial_arrangement)
        self.nPoses = len(self.points)
        self.allPoses = range(self.nPoses)
        # self.magicNumber = self.getMagicNumber(self.numObjs)
        # print("Magic number: " + str(self.magicNumber))

        ### initialize dependency_dict and path_dict as empty dict
        ### since now we are going to increment these two dicts online, instead of offline
        self.dependency_dict = {}
        self.path_dict = {}
        self.Object_locations = copy.deepcopy(Object_locations)
        self.region_dict = copy.deepcopy(region_dict)
        self.linked_list = copy.deepcopy(linked_list)

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
        self.bridge = [None, None, None, None, None] ### [leftKey, rightKey, object_transition, objectMoved, path_option]
        self.leftLeaves = {"L0"} ### keep track of leaves in the left tree
        self.rightLeaves = {"R0"} ### keep track of leaves in the right tree

        ################# testing #################
        self.num_mutation = 0
        self.mutation_time_list = []
        self.avg_time = 0.0
        self.total_time = 0.0

        ################## results ################
        self.isConnected = False
        self.best_solution_cost = np.inf
        ### the whole_path is a list of items and each item has the following format
        ### [("node1_id", node2_id), {2:path2, 1:path1, ...}]
        self.totalActions = 0 ### record the total number of actions
        self.numLeftBranches = 0 ### record the number of left branches in the solution
        self.numRightBranches = 0 ### record the number of right branches in the solution
        self.numNodesInLeftTree = 0 ### record the total number of nodes in the left tree
        self.numNodesInRightTree = 0 ### record the total number of nodes in the right tree

        ### start ruuning
        self.left_idx = 1
        self.right_idx = 1
        ### initial connection attempt

        self.growSubTree(self.treeL["L0"], self.treeR["R0"], "Left")
        if (self.isConnected != True):
            self.growSubTree(self.treeR["R0"], self.treeL["L0"], "Right")

        totalTime_allowed = 900.0 ### allow 500s for the total search tree construction
        start_time = time.clock()        

        while (self.isConnected != True and time.clock() - start_time < totalTime_allowed):
            ### The problem is not monotone
            start = time.time()
            newChild_nodeID = self.mutateLeftChild()
            if newChild_nodeID != None:
                best_goal = self.goal_arrangement_in_range(self.treeL[newChild_nodeID].arrangement, "Right")
                self.growSubTree(self.treeL[newChild_nodeID], self.treeR[best_goal], "Left")
            self.mutation_time_list.append(time.time()-start)
            self.num_mutation += 1


            start = time.time()
            if (self.isConnected != True):
                newChild_nodeID = self.mutateRightChild()
                if newChild_nodeID != None:
                    best_goal = self.goal_arrangement_in_range(self.treeR[newChild_nodeID].arrangement, "Left")
                    self.growSubTree(self.treeR[newChild_nodeID], self.treeL[best_goal], "Right")
            self.mutation_time_list.append(time.time()-start)
            self.num_mutation += 1


        if self.isConnected:
            self.getTheStat()

        if self.isConnected == False:
            print("fail the find a solution within " + str(totalTime_allowed) + " seconds...")

    def goal_arrangement_in_range(self, initial_arrangement, treeside):
        # goal_range = int(math.ceil(self.numObjs/3.0))
        goal_range = 3
        shortest_distance = float('inf')
        shortest_A_star_distance = float('inf')

        if treeside == "Right":
            for node in self.treeR.values():
                temp_goal_arrangement = node.arrangement
                distance = 0
                A_star_distance = 0
                for i in range(len(temp_goal_arrangement)):
                    if temp_goal_arrangement[i] != initial_arrangement[i]:
                        distance += 1
                A_star_distance = distance
                for i in range(len(temp_goal_arrangement)):
                    if temp_goal_arrangement[i] != self.treeR["R0"].arrangement[i]:
                        A_star_distance += 1
                if A_star_distance < shortest_A_star_distance:
                    shortest_distance = distance
                    shortest_A_star_distance = A_star_distance
                    best_goal = node.node_id
                if shortest_distance <= goal_range:
                    return best_goal
        
        if treeside == "Left":
            for node in self.treeL.values():
                temp_goal_arrangement = node.arrangement
                distance = 0
                A_star_distance = 0
                for i in range(len(temp_goal_arrangement)):
                    if temp_goal_arrangement[i] != initial_arrangement[i]:
                        distance += 1
                A_star_distance = distance
                for i in range(len(temp_goal_arrangement)):
                    if temp_goal_arrangement[i] != self.treeL["L0"].arrangement[i]:
                        A_star_distance += 1
                if A_star_distance < shortest_A_star_distance:
                    shortest_distance = distance
                    shortest_A_star_distance = A_star_distance
                    best_goal = node.node_id
                if shortest_distance <= goal_range:
                    return best_goal

        return best_goal




    def mutateRightChild(self):
        ### first choose a node to mutate
        # print("Right mutation")
        mutate_id = "R" + str(random.choice(range(len(self.treeR))))
        # mutate_id = random.choice(list(self.rightLeaves))
        mutated_arrangement = self.treeR[mutate_id].arrangement
        ### choose an object to move
        obj_idx = random.choice(range(self.numObjs))
        ### choose a slot to put the object
        pose_idx = random.choice(self.allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))

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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [new_arrangement[obj_idx], mutated_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeR[mutate_id].cost_to_come
            self.treeR["R"+str(self.right_idx)] = ArrNode(
                        new_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.rightLeaves.difference({mutate_id})
            self.rightLeaves.add("R"+str(self.right_idx))
            self.arrRightRegistr.append(new_arrangement)
            self.idRightRegistr.append("R"+str(self.right_idx))
            self.right_idx += 1
            return self.idRightRegistr[self.arrRightRegistr.index(new_arrangement)]



    def mutateLeftChild(self):
        ### first choose a node to mutate
        # print("Left mutation")
        mutate_id = "L" + str(random.choice(range(len(self.treeL))))
        # mutate_id = random.choice(list(self.leftLeaves))
        mutated_arrangement = self.treeL[mutate_id].arrangement
        ### choose an object to move
        obj_idx = random.choice(range(self.numObjs))
        ### choose a slot to put the object
        pose_idx = random.choice(self.allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))

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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [mutated_arrangement[obj_idx], new_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeL[mutate_id].cost_to_come
            self.treeL["L"+str(self.left_idx)] = ArrNode(
                        new_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.leftLeaves.difference({mutate_id})
            self.leftLeaves.add("L"+str(self.left_idx))
            self.arrLeftRegistr.append(new_arrangement)
            self.idLeftRegistr.append("L"+str(self.left_idx))
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

        subTree = DFS_Rec_for_Monotone_General(
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
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
        while(len(queue) != 0):
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
                        self.rightLeaves.difference({parent_nodeID})

                elif child_arrangement in self.arrLeftRegistr:
                    ### this is a sign that two trees are connected
                    ### check if it is really a bridge
                    if parent_arrangement not in self.arrLeftRegistr:
                        print("a bridge is found")
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
                            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeR[parent_nodeID].cost_to_come + 1
                    self.treeR["R"+str(self.right_idx)] = ArrNode(
                        child_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_cost_to_come, parent_nodeID)
                    self.rightLeaves.difference({parent_nodeID})
                    self.rightLeaves.add("R"+str(self.right_idx))
                    self.arrRightRegistr.append(child_arrangement)
                    self.idRightRegistr.append("R"+str(self.right_idx))
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
        while(len(queue) != 0):
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
                        self.leftLeaves.difference({parent_nodeID})

                elif child_arrangement in self.arrRightRegistr:
                    ### this is a sign that two trees are connected
                    ### check if it is really a bridge
                    if parent_arrangement not in self.arrRightRegistr:
                        print("a bridge is found")
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
                            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeL[parent_nodeID].cost_to_come + 1
                    self.treeL["L"+str(self.left_idx)] = ArrNode(
                        child_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_cost_to_come, parent_nodeID)
                    self.leftLeaves.difference({parent_nodeID})
                    self.leftLeaves.add("L"+str(self.left_idx))
                    self.arrLeftRegistr.append(child_arrangement)
                    self.idLeftRegistr.append("L"+str(self.left_idx))
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

        print("path: " + str(self.simplePath))
        self.totalActions = len(self.simplePath) - 1
        print("total action: " + str(self.totalActions))
        print("solution cost: " + str(self.best_solution_cost)) 


    def getMagicNumber(self, numObjs):
        temp_str = ''
        for i in range(numObjs):
            temp_str += '1'

        return int(temp_str, 2)

class BiDirDPPlanner_Random_Nearest(object):
    ### Input:
    ### (1) initial_arrangement (a list of pose_ids, each of which indicating the initial pose for an object)
    ### (2) final_arrangement (a list of pose_ids, each of which indicating the final pose for an object)
    ### instance 
    ### (i) workspace, (ii) object centers/slots, (iii) buffer centers/slots
    ### visualTool: a visualization tool as a debugging purpose
    ### Output:
    ### the whole plan 
    def __init__(self, init_arr, final_arr, instance, Object_locations, region_dict, linked_list, visualTool):
        self.initial_arrangement = init_arr
        self.final_arrangement = final_arr
        self.points = instance.points + instance.buffer_points
        self.objects = instance.objects + instance.buffers
        self.numObjs = len(self.initial_arrangement)
        self.nPoses = len(self.points)
        self.allPoses = range(self.nPoses)
        # self.magicNumber = self.getMagicNumber(self.numObjs)
        # print("Magic number: " + str(self.magicNumber))

        ### initialize dependency_dict and path_dict as empty dict
        ### since now we are going to increment these two dicts online, instead of offline
        self.dependency_dict = {}
        self.path_dict = {}
        self.Object_locations = copy.deepcopy(Object_locations)
        self.region_dict = copy.deepcopy(region_dict)
        self.linked_list = copy.deepcopy(linked_list)

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
        self.bridge = [None, None, None, None, None] ### [leftKey, rightKey, object_transition, objectMoved, path_option]
        self.leftLeaves = {"L0"} ### keep track of leaves in the left tree
        self.rightLeaves = {"R0"} ### keep track of leaves in the right tree

        ################# testing #################
        self.num_mutation = 0
        self.mutation_time_list = []
        self.avg_time = 0.0
        self.total_time = 0.0

        ################## results ################
        self.isConnected = False
        self.best_solution_cost = np.inf
        ### the whole_path is a list of items and each item has the following format
        ### [("node1_id", node2_id), {2:path2, 1:path1, ...}]
        self.totalActions = 0 ### record the total number of actions
        self.numLeftBranches = 0 ### record the number of left branches in the solution
        self.numRightBranches = 0 ### record the number of right branches in the solution
        self.numNodesInLeftTree = 0 ### record the total number of nodes in the left tree
        self.numNodesInRightTree = 0 ### record the total number of nodes in the right tree

        ### start ruuning
        self.left_idx = 1
        self.right_idx = 1
        ### initial connection attempt

        self.growSubTree(self.treeL["L0"], self.treeR["R0"], "Left")
        if (self.isConnected != True):
            self.growSubTree(self.treeR["R0"], self.treeL["L0"], "Right")

        totalTime_allowed = 900.0 ### allow 500s for the total search tree construction
        start_time = time.clock()        

        while (self.isConnected != True and time.clock() - start_time < totalTime_allowed):
            ### The problem is not monotone
            start = time.time()
            newChild_nodeID = self.mutateLeftChild()
            if newChild_nodeID != None:
                best_goal = self.goal_arrangement_nearest(self.treeL[newChild_nodeID].arrangement, "Right")
                self.growSubTree(self.treeL[newChild_nodeID], self.treeR[best_goal], "Left")
            self.mutation_time_list.append(time.time()-start)
            self.num_mutation += 1


            start = time.time()
            if (self.isConnected != True):
                newChild_nodeID = self.mutateRightChild()
                if newChild_nodeID != None:
                    best_goal = self.goal_arrangement_nearest(self.treeR[newChild_nodeID].arrangement, "Left")
                    self.growSubTree(self.treeR[newChild_nodeID], self.treeL[best_goal], "Right")
            self.mutation_time_list.append(time.time()-start)
            self.num_mutation += 1


        if self.isConnected:
            self.getTheStat()

        if self.isConnected == False:
            print("fail the find a solution within " + str(totalTime_allowed) + " seconds...")

    def goal_arrangement_nearest(self, initial_arrangement, treeside):
        shortest_distance = float('inf')
        shortest_A_star_distance = float('inf')

        if treeside == "Right":
            for node in self.treeR.values():
                temp_goal_arrangement = node.arrangement
                distance = 0
                A_star_distance = 0
                for i in range(len(temp_goal_arrangement)):
                    if temp_goal_arrangement[i] != initial_arrangement[i]:
                        distance += 1
                A_star_distance = distance
                for i in range(len(temp_goal_arrangement)):
                    if temp_goal_arrangement[i] != self.treeR["R0"].arrangement[i]:
                        A_star_distance += 1
                if A_star_distance < shortest_A_star_distance:
                    shortest_distance = distance
                    shortest_A_star_distance = A_star_distance
                    best_goal = node.node_id
        
        if treeside == "Left":
            for node in self.treeL.values():
                temp_goal_arrangement = node.arrangement
                distance = 0
                A_star_distance = 0
                for i in range(len(temp_goal_arrangement)):
                    if temp_goal_arrangement[i] != initial_arrangement[i]:
                        distance += 1
                A_star_distance = distance
                for i in range(len(temp_goal_arrangement)):
                    if temp_goal_arrangement[i] != self.treeL["L0"].arrangement[i]:
                        A_star_distance += 1
                if A_star_distance < shortest_A_star_distance:
                    shortest_distance = distance
                    shortest_A_star_distance = A_star_distance
                    best_goal = node.node_id

        return best_goal




    def mutateRightChild(self):
        ### first choose a node to mutate
        # print("Right mutation")
        # mutate_id = "R" + str(random.choice(range(len(self.treeR))))
        mutate_id = random.choice(list(self.rightLeaves))
        mutated_arrangement = self.treeR[mutate_id].arrangement
        ### choose an object to move
        obj_idx = random.choice(range(self.numObjs))
        ### choose a slot to put the object
        pose_idx = random.choice(self.allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))

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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [new_arrangement[obj_idx], mutated_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeR[mutate_id].cost_to_come
            self.treeR["R"+str(self.right_idx)] = ArrNode(
                        new_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.rightLeaves.difference({mutate_id})
            self.rightLeaves.add("R"+str(self.right_idx))
            self.arrRightRegistr.append(new_arrangement)
            self.idRightRegistr.append("R"+str(self.right_idx))
            self.right_idx += 1
            return self.idRightRegistr[self.arrRightRegistr.index(new_arrangement)]



    def mutateLeftChild(self):
        ### first choose a node to mutate
        # print("Left mutation")
        # mutate_id = "L" + str(random.choice(range(len(self.treeL))))
        mutate_id = random.choice(list(self.leftLeaves))
        mutated_arrangement = self.treeL[mutate_id].arrangement
        ### choose an object to move
        obj_idx = random.choice(range(self.numObjs))
        ### choose a slot to put the object
        pose_idx = random.choice(self.allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))

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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [mutated_arrangement[obj_idx], new_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeL[mutate_id].cost_to_come
            self.treeL["L"+str(self.left_idx)] = ArrNode(
                        new_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.leftLeaves.difference({mutate_id})
            self.leftLeaves.add("L"+str(self.left_idx))
            self.arrLeftRegistr.append(new_arrangement)
            self.idLeftRegistr.append("L"+str(self.left_idx))
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

        subTree = DFS_Rec_for_Monotone_General(
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
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
        while(len(queue) != 0):
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
                        self.rightLeaves.difference({parent_nodeID})

                elif child_arrangement in self.arrLeftRegistr:
                    ### this is a sign that two trees are connected
                    ### check if it is really a bridge
                    if parent_arrangement not in self.arrLeftRegistr:
                        print("a bridge is found")
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
                            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeR[parent_nodeID].cost_to_come + 1
                    self.treeR["R"+str(self.right_idx)] = ArrNode(
                        child_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_cost_to_come, parent_nodeID)
                    self.rightLeaves.difference({parent_nodeID})
                    self.rightLeaves.add("R"+str(self.right_idx))
                    self.arrRightRegistr.append(child_arrangement)
                    self.idRightRegistr.append("R"+str(self.right_idx))
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
        while(len(queue) != 0):
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
                        self.leftLeaves.difference({parent_nodeID})

                elif child_arrangement in self.arrRightRegistr:
                    ### this is a sign that two trees are connected
                    ### check if it is really a bridge
                    if parent_arrangement not in self.arrRightRegistr:
                        print("a bridge is found")
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
                            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeL[parent_nodeID].cost_to_come + 1
                    self.treeL["L"+str(self.left_idx)] = ArrNode(
                        child_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_cost_to_come, parent_nodeID)
                    self.leftLeaves.difference({parent_nodeID})
                    self.leftLeaves.add("L"+str(self.left_idx))
                    self.arrLeftRegistr.append(child_arrangement)
                    self.idLeftRegistr.append("L"+str(self.left_idx))
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

        print("path: " + str(self.simplePath))
        self.totalActions = len(self.simplePath) - 1
        print("total action: " + str(self.totalActions))
        print("solution cost: " + str(self.best_solution_cost)) 


    def getMagicNumber(self, numObjs):
        temp_str = ''
        for i in range(numObjs):
            temp_str += '1'

        return int(temp_str, 2)


class BiDirDPPlanner_Leaf_Root(object):
    ### Input:
    ### (1) initial_arrangement (a list of pose_ids, each of which indicating the initial pose for an object)
    ### (2) final_arrangement (a list of pose_ids, each of which indicating the final pose for an object)
    ### instance 
    ### (i) workspace, (ii) object centers/slots, (iii) buffer centers/slots
    ### visualTool: a visualization tool as a debugging purpose
    ### Output:
    ### the whole plan 
    def __init__(self, init_arr, final_arr, instance, Object_locations, region_dict, linked_list, visualTool):
        self.initial_arrangement = init_arr
        self.final_arrangement = final_arr
        self.points = instance.points + instance.buffer_points
        self.objects = instance.objects + instance.buffers
        self.numObjs = len(self.initial_arrangement)
        self.nPoses = len(self.points)
        self.allPoses = range(self.nPoses)
        # self.magicNumber = self.getMagicNumber(self.numObjs)
        # print("Magic number: " + str(self.magicNumber))

        ### initialize dependency_dict and path_dict as empty dict
        ### since now we are going to increment these two dicts online, instead of offline
        self.dependency_dict = {}
        self.path_dict = {}
        self.Object_locations = copy.deepcopy(Object_locations)
        self.region_dict = copy.deepcopy(region_dict)
        self.linked_list = copy.deepcopy(linked_list)

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
        self.bridge = [None, None, None, None, None] ### [leftKey, rightKey, object_transition, objectMoved, path_option]
        self.leftLeaves = {"L0"} ### keep track of leaves in the left tree
        self.rightLeaves = {"R0"} ### keep track of leaves in the right tree

        ################# testing #################
        self.num_mutation = 0
        self.mutation_time_list = []
        self.avg_time = 0.0
        self.total_time = 0.0

        ################## results ################
        self.isConnected = False
        self.best_solution_cost = np.inf
        ### the whole_path is a list of items and each item has the following format
        ### [("node1_id", node2_id), {2:path2, 1:path1, ...}]
        self.totalActions = 0 ### record the total number of actions
        self.numLeftBranches = 0 ### record the number of left branches in the solution
        self.numRightBranches = 0 ### record the number of right branches in the solution
        self.numNodesInLeftTree = 0 ### record the total number of nodes in the left tree
        self.numNodesInRightTree = 0 ### record the total number of nodes in the right tree

        ### start ruuning
        self.left_idx = 1
        self.right_idx = 1
        ### initial connection attempt

        self.growSubTree(self.treeL["L0"], self.treeR["R0"], "Left")
        if (self.isConnected != True):
            self.growSubTree(self.treeR["R0"], self.treeL["L0"], "Right")

        totalTime_allowed = 900.0 ### allow 500s for the total search tree construction
        start_time = time.clock()        

        while (self.isConnected != True and time.clock() - start_time < totalTime_allowed):
            ### The problem is not monotone
            start = time.time()
            newChild_nodeID = self.mutateLeftChild()
            if newChild_nodeID != None:
                self.growSubTree(self.treeL[newChild_nodeID], self.treeR["R0"], "Left")
            self.mutation_time_list.append(time.time()-start)
            self.num_mutation += 1


            start = time.time()
            if (self.isConnected != True):
                newChild_nodeID = self.mutateRightChild()
                if newChild_nodeID != None:
                    self.growSubTree(self.treeR[newChild_nodeID], self.treeL["L0"], "Right")
            self.mutation_time_list.append(time.time()-start)
            self.num_mutation += 1


        if self.isConnected:
            self.getTheStat()

        if self.isConnected == False:
            print("fail the find a solution within " + str(totalTime_allowed) + " seconds...")

    def mutateRightChild(self):
        ### first choose a node to mutate
        # print("Right mutation")
        # mutate_id = "R" + str(random.choice(range(len(self.treeR))))
        mutate_id = random.choice(list(self.rightLeaves))
        mutated_arrangement = self.treeR[mutate_id].arrangement
        ### choose an object to move
        obj_idx = random.choice(range(self.numObjs))
        ### choose a slot to put the object
        pose_idx = random.choice(self.allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))

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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [new_arrangement[obj_idx], mutated_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeR[mutate_id].cost_to_come
            self.treeR["R"+str(self.right_idx)] = ArrNode(
                        new_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.rightLeaves.difference({mutate_id})
            self.rightLeaves.add("R"+str(self.right_idx))
            self.arrRightRegistr.append(new_arrangement)
            self.idRightRegistr.append("R"+str(self.right_idx))
            self.right_idx += 1
            return self.idRightRegistr[self.arrRightRegistr.index(new_arrangement)]



    def mutateLeftChild(self):
        ### first choose a node to mutate
        # print("Left mutation")
        # mutate_id = "L" + str(random.choice(range(len(self.treeL))))
        mutate_id = random.choice(list(self.leftLeaves))
        mutated_arrangement = self.treeL[mutate_id].arrangement
        ### choose an object to move
        obj_idx = random.choice(range(self.numObjs))
        ### choose a slot to put the object
        pose_idx = random.choice(self.allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))

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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [mutated_arrangement[obj_idx], new_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeL[mutate_id].cost_to_come
            self.treeL["L"+str(self.left_idx)] = ArrNode(
                        new_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.leftLeaves.difference({mutate_id})
            self.leftLeaves.add("L"+str(self.left_idx))
            self.arrLeftRegistr.append(new_arrangement)
            self.idLeftRegistr.append("L"+str(self.left_idx))
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

        subTree = DFS_Rec_for_Monotone_General(
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
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
        while(len(queue) != 0):
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
                        self.rightLeaves.difference({parent_nodeID})

                elif child_arrangement in self.arrLeftRegistr:
                    ### this is a sign that two trees are connected
                    ### check if it is really a bridge
                    if parent_arrangement not in self.arrLeftRegistr:
                        print("a bridge is found")
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
                            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeR[parent_nodeID].cost_to_come + 1
                    self.treeR["R"+str(self.right_idx)] = ArrNode(
                        child_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_cost_to_come, parent_nodeID)
                    self.rightLeaves.difference({parent_nodeID})
                    self.rightLeaves.add("R"+str(self.right_idx))
                    self.arrRightRegistr.append(child_arrangement)
                    self.idRightRegistr.append("R"+str(self.right_idx))
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
        while(len(queue) != 0):
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
                        self.leftLeaves.difference({parent_nodeID})
                        

                elif child_arrangement in self.arrRightRegistr:
                    ### this is a sign that two trees are connected
                    ### check if it is really a bridge
                    if parent_arrangement not in self.arrRightRegistr:
                        print("a bridge is found")
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
                            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeL[parent_nodeID].cost_to_come + 1
                    self.treeL["L"+str(self.left_idx)] = ArrNode(
                        child_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_cost_to_come, parent_nodeID)
                    self.leftLeaves.difference({parent_nodeID})
                    self.leftLeaves.add("L"+str(self.left_idx))
                    self.arrLeftRegistr.append(child_arrangement)
                    self.idLeftRegistr.append("L"+str(self.left_idx))
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

        print("path: " + str(self.simplePath))
        self.totalActions = len(self.simplePath) - 1
        print("total action: " + str(self.totalActions))
        print("solution cost: " + str(self.best_solution_cost)) 


    def getMagicNumber(self, numObjs):
        temp_str = ''
        for i in range(numObjs):
            temp_str += '1'

        return int(temp_str, 2)

class BiDirDPPlanner_Leaf_Root_Improved_Mutation(object):
    ### Input:
    ### (1) initial_arrangement (a list of pose_ids, each of which indicating the initial pose for an object)
    ### (2) final_arrangement (a list of pose_ids, each of which indicating the final pose for an object)
    ### instance 
    ### (i) workspace, (ii) object centers/slots, (iii) buffer centers/slots
    ### visualTool: a visualization tool as a debugging purpose
    ### Output:
    ### the whole plan 
    def __init__(self, init_arr, final_arr, instance, Object_locations, region_dict, linked_list, points, RAD, visualTool):
        self.initial_arrangement = init_arr
        self.final_arrangement = final_arr
        self.points = instance.points + instance.buffer_points
        self.objects = instance.objects + instance.buffers
        self.numObjs = len(self.initial_arrangement)
        self.nPoses = len(self.points)
        self.allPoses = range(self.nPoses)
        # self.magicNumber = self.getMagicNumber(self.numObjs)
        # print("Magic number: " + str(self.magicNumber))

        ### initialize dependency_dict and path_dict as empty dict
        ### since now we are going to increment these two dicts online, instead of offline
        self.dependency_dict = {}
        self.path_dict = {}
        self.Object_locations = copy.deepcopy(Object_locations)
        self.region_dict = copy.deepcopy(region_dict)
        self.linked_list = copy.deepcopy(linked_list)
        self.points = points
        self.RAD = RAD

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
        self.bridge = [None, None, None, None, None] ### [leftKey, rightKey, object_transition, objectMoved, path_option]
        self.leftLeaves = {"L0"} ### keep track of leaves in the left tree
        self.rightLeaves = {"R0"} ### keep track of leaves in the right tree

        ################# testing #################
        self.num_mutation = 0
        self.mutation_time_list = []
        self.avg_time = 0.0
        self.total_time = 0.0

        ################## results ################
        self.isConnected = False
        self.best_solution_cost = np.inf
        ### the whole_path is a list of items and each item has the following format
        ### [("node1_id", node2_id), {2:path2, 1:path1, ...}]
        self.totalActions = 0 ### record the total number of actions
        self.numLeftBranches = 0 ### record the number of left branches in the solution
        self.numRightBranches = 0 ### record the number of right branches in the solution
        self.numNodesInLeftTree = 0 ### record the total number of nodes in the left tree
        self.numNodesInRightTree = 0 ### record the total number of nodes in the right tree

        ### start ruuning
        self.left_idx = 1
        self.right_idx = 1
        ### initial connection attempt

        self.growSubTree(self.treeL["L0"], self.treeR["R0"], "Left")
        if (self.isConnected != True):
            self.growSubTree(self.treeR["R0"], self.treeL["L0"], "Right")

        totalTime_allowed = 900.0 ### allow 500s for the total search tree construction
        start_time = time.clock()        

        while (self.isConnected != True and time.clock() - start_time < totalTime_allowed):
            ### The problem is not monotone
            start = time.time()
            # newChild_nodeID = self.mutateLeftChild()
            left_leaf_candidate = random.choice(list(self.leftLeaves))
            _, index = self.potentialObstacles(self.treeL[left_leaf_candidate], self.treeR["R0"])
            newChild_nodeID = self.improvedMutateLeftChild(index, left_leaf_candidate)
            if newChild_nodeID != None:
                self.growSubTree(self.treeL[newChild_nodeID], self.treeR["R0"], "Left")
            self.mutation_time_list.append(time.time()-start)
            self.num_mutation += 1


            start = time.time()
            if (self.isConnected != True):
                # newChild_nodeID = self.mutateRightChild()
                right_leaf_candidate = random.choice(list(self.rightLeaves))
                _, index = self.potentialObstacles(self.treeR[right_leaf_candidate], self.treeL["L0"])
                newChild_nodeID = self.improvedMutateRightChild(index, right_leaf_candidate)
                if newChild_nodeID != None:
                    self.growSubTree(self.treeR[newChild_nodeID], self.treeL["L0"], "Right")
            self.mutation_time_list.append(time.time()-start)
            self.num_mutation += 1


        if self.isConnected:
            self.getTheStat()

        if self.isConnected == False:
            print("fail the find a solution within " + str(totalTime_allowed) + " seconds...")

    def mutateRightChild(self):
        ### first choose a node to mutate
        # print("Right mutation")
        # mutate_id = "R" + str(random.choice(range(len(self.treeR))))
        mutate_id = random.choice(list(self.rightLeaves))
        mutated_arrangement = self.treeR[mutate_id].arrangement
        ### choose an object to move
        obj_idx = random.choice(range(self.numObjs))
        ### choose a slot to put the object
        pose_idx = random.choice(self.allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))

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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [new_arrangement[obj_idx], mutated_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeR[mutate_id].cost_to_come
            self.treeR["R"+str(self.right_idx)] = ArrNode(
                        new_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.rightLeaves.difference({mutate_id})
            self.rightLeaves.add("R"+str(self.right_idx))
            self.arrRightRegistr.append(new_arrangement)
            self.idRightRegistr.append("R"+str(self.right_idx))
            self.right_idx += 1
            return self.idRightRegistr[self.arrRightRegistr.index(new_arrangement)]



    def mutateLeftChild(self):
        ### first choose a node to mutate
        # print("Left mutation")
        # mutate_id = "L" + str(random.choice(range(len(self.treeL))))
        mutate_id = random.choice(list(self.leftLeaves))
        mutated_arrangement = self.treeL[mutate_id].arrangement
        ### choose an object to move
        obj_idx = random.choice(range(self.numObjs))
        ### choose a slot to put the object
        pose_idx = random.choice(self.allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))

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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [mutated_arrangement[obj_idx], new_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeL[mutate_id].cost_to_come
            self.treeL["L"+str(self.left_idx)] = ArrNode(
                        new_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.leftLeaves.difference({mutate_id})
            self.leftLeaves.add("L"+str(self.left_idx))
            self.arrLeftRegistr.append(new_arrangement)
            self.idLeftRegistr.append("L"+str(self.left_idx))
            self.left_idx += 1
            return self.idLeftRegistr[self.arrLeftRegistr.index(new_arrangement)]



    def improvedMutateRightChild(self, index, right_index):
        ### first choose a node to mutate
        # print("Left mutation")
        mutate_id = right_index
        mutated_arrangement = self.treeR[mutate_id].arrangement
        ### choose an object to move
        obj_idx = index
        ### choose a slot to put the object
        pose_idx = self.furthest_pose(self.treeR[mutate_id].arrangement, obj_idx)

        if pose_idx == -1:
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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [mutated_arrangement[obj_idx], new_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeR[mutate_id].cost_to_come
            self.treeR["R"+str(self.right_idx)] = ArrNode(
                        new_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.arrRightRegistr.append(new_arrangement)
            self.idRightRegistr.append("R"+str(self.right_idx))
            self.right_idx += 1

            ### Check whether the new arrangement is at the other side
            if new_arrangement in self.arrLeftRegistr:
                ### this is a sign that two trees are connected
                ### check if it is really a bridge
                print("a bridge is found")
                self.isConnected = True
                ### check if it leads to a better solution
                temp_rightKey = self.idRightRegistr[self.arrRightRegistr.index(self.treeR[mutate_id].arrangement)]
                temp_leftKey = self.idLeftRegistr[self.arrLeftRegistr.index(new_arrangement)]
                temp_cost = self.treeR[temp_rightKey].cost_to_come + self.treeL[temp_leftKey].cost_to_come + 1
                if temp_cost < self.best_solution_cost:
                    ### This is a better solution
                    self.best_solution_cost = temp_cost
                    ### let's update the bridge
                    self.leftKey = temp_leftKey
                    self.rightKey = temp_rightKey
                    self.bridge_transition = temp_transition
                    self.bridge_objectMoved = temp_object_idx
                    self.bridge_path_option = temp_path_option
                    self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]
                return None
                
            return self.idRightRegistr[self.arrRightRegistr.index(new_arrangement)]

    def improvedMutateLeftChild(self, index, left_index):
        ### first choose a node to mutate
        # print("Left mutation")
        mutate_id = left_index
        mutated_arrangement = self.treeL[mutate_id].arrangement
        ### choose an object to move
        obj_idx = index
        ### choose a slot to put the object
        pose_idx = self.furthest_pose(self.treeL[mutate_id].arrangement, obj_idx)

        if pose_idx == -1:
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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [mutated_arrangement[obj_idx], new_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeL[mutate_id].cost_to_come
            self.treeL["L"+str(self.left_idx)] = ArrNode(
                        new_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.arrLeftRegistr.append(new_arrangement)
            self.idLeftRegistr.append("L"+str(self.left_idx))
            self.left_idx += 1

            ### Check whether the new arrangement is at the other side
            if new_arrangement in self.arrRightRegistr:
                ### this is a sign that two trees are connected
                ### check if it is really a bridge
                print("a bridge is found")
                self.isConnected = True
                ### check if it leads to a better solution
                temp_leftKey = self.idLeftRegistr[self.arrLeftRegistr.index(self.treeL[mutate_id].arrangement)]
                temp_rightKey = self.idRightRegistr[self.arrRightRegistr.index(new_arrangement)]
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
                    self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]
                return None

            return self.idLeftRegistr[self.arrLeftRegistr.index(new_arrangement)]

    def furthest_pose(self, arrangement, obj_idx):
        occupied_poses = []
        for i in range(len(arrangement)):
            if i == obj_idx:
                continue
            occupied_poses.append(arrangement[i])
        
        Available_Regions = []
        for region in self.region_dict.keys():
            OCCUPIED = False
            for pose in region:
                if pose in occupied_poses:
                    OCCUPIED = True
                    break
            if not OCCUPIED:
                Available_Regions.append(self.region_dict[region])

        available_goals_dict = {}
        for i in self.allPoses:
            if (self.region_dict[self.Object_locations[i]] in Available_Regions):
                available_goals_dict[self.region_dict[self.Object_locations[i]]] = i
        
        furthest_available_pose = -1
        parents = {}
        explored = {}
        for key in self.region_dict.values():
            explored[key] = 0
        queue = [self.region_dict[self.Object_locations[arrangement[obj_idx]]]]
        explored[self.region_dict[self.Object_locations[arrangement[obj_idx]]]] = 1
        while (len(queue) >0):
            # stack(-1) for DFS and queue(0) for BFS
            old_node = queue.pop(-1)
            if old_node in self.linked_list:
                for region in self.linked_list[old_node]:
                    if explored[region]:
                        continue
                    if region not in Available_Regions:
                        continue
                    parents[region] = old_node
                    if region in available_goals_dict.keys():
                        furthest_available_pose = available_goals_dict[region]
                    queue.append(region)
                    explored[region] = 1
        return furthest_available_pose

    def potentialObstacles(self, initNode, goalNode):
        diff = 0
        # potential collision times as obstacles to other objects
        collision_times = {}
        for i in range(len(initNode.arrangement)):
            collision_times[i] = 0
        
        for i in range(len(initNode.arrangement)):
            if initNode.arrangement[i] == goalNode.arrangement[i]:
                continue
            init_point = self.points[initNode.arrangement[i]]
            goal_point = self.points[goalNode.arrangement[i]]
            for j in range(len(initNode.arrangement)):
                if i == j:
                    continue
                obstacle = self.points[initNode.arrangement[j]]
                if self.collision_check(init_point, goal_point, obstacle):
                    diff += 0.5
                    collision_times[j] += 0.5
                obstacle = self.points[goalNode.arrangement[j]]
                if self.collision_check(init_point, goal_point, obstacle):
                    diff += 0.5
                    collision_times[j] += 0.5
        max_collision = 0
        most_collision_index = 0
        for key in collision_times.keys():
            if collision_times[key] > max_collision:
                max_collision = collision_times[key]
                most_collision_index = key
        return diff, most_collision_index

    def collision_check(self, init, goal, obst):
        dx = goal[0] - init[0]
        dy = goal[1] - init[1]
        # init->goal dyx-dxy+c1 = 0
        c1 = dx*init[1] - dy*init[0]
        # obstacle: dxx+dyy+c2 = 0
        c2 = -dx*init[0] -dy*init[1]

        # foot
        fx = (c1*dx-c2*dy)/(dx**2+dy**2)
        fy = (c2*dx-c1*dy)/(dx**2+dy**2)

        # alpha
        if fx == init[0]:
            alpha = (fy-init[1])/(goal[1]-init[1])
        else:
            alpha = (fx-init[0])/(goal[0]-init[0])
    
        if alpha>1.0:
            dist = math.sqrt((obst[0]-goal[0])**2 + (obst[1]-goal[1])**2)
        elif alpha <0.0:
            dist = math.sqrt((obst[0]-init[0])**2 + (obst[1]-init[1])**2)
        else:
            dist = math.sqrt((obst[0]-fx)**2 + (obst[1]-fy)**2)

        if dist < 2* self.RAD:
            return True
        else:
            return False


    def growSubTree(self, initNode, goalNode, treeSide):
        ### construct start_poses and goal_poses
        start_poses = {}
        goal_poses = {}
        for i in range(len(initNode.arrangement)):
            start_poses[i] = initNode.arrangement[i]
        for i in range(len(goalNode.arrangement)):
            goal_poses[i] = goalNode.arrangement[i]

        subTree = DFS_Rec_for_Monotone_General(
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
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
        while(len(queue) != 0):
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
                        self.rightLeaves.difference({parent_nodeID})

                elif child_arrangement in self.arrLeftRegistr:
                    ### this is a sign that two trees are connected
                    ### check if it is really a bridge
                    if parent_arrangement not in self.arrLeftRegistr:
                        print("a bridge is found")
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
                            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeR[parent_nodeID].cost_to_come + 1
                    self.treeR["R"+str(self.right_idx)] = ArrNode(
                        child_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_cost_to_come, parent_nodeID)
                    self.rightLeaves.difference({parent_nodeID})
                    self.rightLeaves.add("R"+str(self.right_idx))
                    self.arrRightRegistr.append(child_arrangement)
                    self.idRightRegistr.append("R"+str(self.right_idx))
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
        while(len(queue) != 0):
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
                        self.leftLeaves.difference({parent_nodeID})
                        

                elif child_arrangement in self.arrRightRegistr:
                    ### this is a sign that two trees are connected
                    ### check if it is really a bridge
                    if parent_arrangement not in self.arrRightRegistr:
                        print("a bridge is found")
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
                            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeL[parent_nodeID].cost_to_come + 1
                    self.treeL["L"+str(self.left_idx)] = ArrNode(
                        child_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_cost_to_come, parent_nodeID)
                    self.leftLeaves.difference({parent_nodeID})
                    self.leftLeaves.add("L"+str(self.left_idx))
                    self.arrLeftRegistr.append(child_arrangement)
                    self.idLeftRegistr.append("L"+str(self.left_idx))
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

        print("path: " + str(self.simplePath))
        self.totalActions = len(self.simplePath) - 1
        print("total action: " + str(self.totalActions))
        print("solution cost: " + str(self.best_solution_cost)) 


    def getMagicNumber(self, numObjs):
        temp_str = ''
        for i in range(numObjs):
            temp_str += '1'

        return int(temp_str, 2)


class BiDirDPPlanner_Leaf_Small_Range(object):
    ### Input:
    ### (1) initial_arrangement (a list of pose_ids, each of which indicating the initial pose for an object)
    ### (2) final_arrangement (a list of pose_ids, each of which indicating the final pose for an object)
    ### instance 
    ### (i) workspace, (ii) object centers/slots, (iii) buffer centers/slots
    ### visualTool: a visualization tool as a debugging purpose
    ### Output:
    ### the whole plan 
    def __init__(self, init_arr, final_arr, instance, Object_locations, region_dict, linked_list, visualTool):
        self.initial_arrangement = init_arr
        self.final_arrangement = final_arr
        self.points = instance.points + instance.buffer_points
        self.objects = instance.objects + instance.buffers
        self.numObjs = len(self.initial_arrangement)
        self.nPoses = len(self.points)
        self.allPoses = range(self.nPoses)
        # self.magicNumber = self.getMagicNumber(self.numObjs)
        # print("Magic number: " + str(self.magicNumber))

        ### initialize dependency_dict and path_dict as empty dict
        ### since now we are going to increment these two dicts online, instead of offline
        self.dependency_dict = {}
        self.path_dict = {}
        self.Object_locations = copy.deepcopy(Object_locations)
        self.region_dict = copy.deepcopy(region_dict)
        self.linked_list = copy.deepcopy(linked_list)

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
        self.bridge = [None, None, None, None, None] ### [leftKey, rightKey, object_transition, objectMoved, path_option]
        self.leftLeaves = {"L0"} ### keep track of leaves in the left tree
        self.rightLeaves = {"R0"} ### keep track of leaves in the right tree

        ################# testing #################
        self.num_mutation = 0
        self.mutation_time_list = []
        self.avg_time = 0.0
        self.total_time = 0.0

        ################## results ################
        self.isConnected = False
        self.best_solution_cost = np.inf
        ### the whole_path is a list of items and each item has the following format
        ### [("node1_id", node2_id), {2:path2, 1:path1, ...}]
        self.totalActions = 0 ### record the total number of actions
        self.numLeftBranches = 0 ### record the number of left branches in the solution
        self.numRightBranches = 0 ### record the number of right branches in the solution
        self.numNodesInLeftTree = 0 ### record the total number of nodes in the left tree
        self.numNodesInRightTree = 0 ### record the total number of nodes in the right tree

        ### start ruuning
        self.left_idx = 1
        self.right_idx = 1
        ### initial connection attempt

        self.growSubTree(self.treeL["L0"], self.treeR["R0"], "Left")
        if (self.isConnected != True):
            self.growSubTree(self.treeR["R0"], self.treeL["L0"], "Right")

        totalTime_allowed = 900.0 ### allow 500s for the total search tree construction
        start_time = time.clock()        

        while (self.isConnected != True and time.clock() - start_time < totalTime_allowed):
            ### The problem is not monotone
            start = time.time()
            newChild_nodeID = self.mutateLeftChild()
            if newChild_nodeID != None:
                best_goal = self.goal_arrangement_in_range(self.treeL[newChild_nodeID].arrangement, "Right")
                self.growSubTree(self.treeL[newChild_nodeID], self.treeR[best_goal], "Left")
            self.mutation_time_list.append(time.time()-start)
            self.num_mutation += 1


            start = time.time()
            if (self.isConnected != True):
                newChild_nodeID = self.mutateRightChild()
                if newChild_nodeID != None:
                    best_goal = self.goal_arrangement_in_range(self.treeR[newChild_nodeID].arrangement, "Left")
                    self.growSubTree(self.treeR[newChild_nodeID], self.treeL[best_goal], "Right")
            self.mutation_time_list.append(time.time()-start)
            self.num_mutation += 1


        if self.isConnected:
            self.getTheStat()

        if self.isConnected == False:
            print("fail the find a solution within " + str(totalTime_allowed) + " seconds...")

    def goal_arrangement_in_range(self, initial_arrangement, treeside):
        # goal_range = int(math.ceil(self.numObjs/3.0))
        goal_range = 3
        shortest_A_star_distance = float('inf')

        if treeside == "Right":
            best_goal = "R0"
            for node in self.treeR.values():
                temp_goal_arrangement = node.arrangement
                distance = 0
                A_star_distance = 0
                for i in range(len(temp_goal_arrangement)):
                    if temp_goal_arrangement[i] != initial_arrangement[i]:
                        distance += 1
                if distance<=goal_range:
                    A_star_distance = distance
                    for i in range(len(temp_goal_arrangement)):
                        if temp_goal_arrangement[i] != self.treeR["R0"].arrangement[i]:
                            A_star_distance += 1
                    if A_star_distance < shortest_A_star_distance:
                        shortest_A_star_distance = A_star_distance
                        best_goal = node.node_id
        
        if treeside == "Left":
            best_goal = "L0"
            for node in self.treeL.values():
                temp_goal_arrangement = node.arrangement
                distance = 0
                A_star_distance = 0
                for i in range(len(temp_goal_arrangement)):
                    if temp_goal_arrangement[i] != initial_arrangement[i]:
                        distance += 1
                if distance <= goal_range:
                    A_star_distance = distance
                    for i in range(len(temp_goal_arrangement)):
                        if temp_goal_arrangement[i] != self.treeL["L0"].arrangement[i]:
                            A_star_distance += 1
                    if A_star_distance < shortest_A_star_distance:
                        shortest_A_star_distance = A_star_distance
                        best_goal = node.node_id

        return best_goal




    def mutateRightChild(self):
        ### first choose a node to mutate
        # print("Right mutation")
        # mutate_id = "R" + str(random.choice(range(len(self.treeR))))
        mutate_id = random.choice(list(self.rightLeaves))
        mutated_arrangement = self.treeR[mutate_id].arrangement
        ### choose an object to move
        obj_idx = random.choice(range(self.numObjs))
        ### choose a slot to put the object
        pose_idx = random.choice(self.allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))

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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [new_arrangement[obj_idx], mutated_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeR[mutate_id].cost_to_come
            self.treeR["R"+str(self.right_idx)] = ArrNode(
                        new_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.rightLeaves.difference({mutate_id})
            self.rightLeaves.add("R"+str(self.right_idx))
            self.arrRightRegistr.append(new_arrangement)
            self.idRightRegistr.append("R"+str(self.right_idx))
            self.right_idx += 1
            return self.idRightRegistr[self.arrRightRegistr.index(new_arrangement)]



    def mutateLeftChild(self):
        ### first choose a node to mutate
        # print("Left mutation")
        # mutate_id = "L" + str(random.choice(range(len(self.treeL))))
        mutate_id = random.choice(list(self.leftLeaves))
        mutated_arrangement = self.treeL[mutate_id].arrangement
        ### choose an object to move
        obj_idx = random.choice(range(self.numObjs))
        ### choose a slot to put the object
        pose_idx = random.choice(self.allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))

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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [mutated_arrangement[obj_idx], new_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeL[mutate_id].cost_to_come
            self.treeL["L"+str(self.left_idx)] = ArrNode(
                        new_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.leftLeaves.difference({mutate_id})
            self.leftLeaves.add("L"+str(self.left_idx))
            self.arrLeftRegistr.append(new_arrangement)
            self.idLeftRegistr.append("L"+str(self.left_idx))
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

        subTree = DFS_Rec_for_Monotone_General(
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
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
        while(len(queue) != 0):
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
                        self.rightLeaves.difference({parent_nodeID})

                elif child_arrangement in self.arrLeftRegistr:
                    ### this is a sign that two trees are connected
                    ### check if it is really a bridge
                    if parent_arrangement not in self.arrLeftRegistr:
                        print("a bridge is found")
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
                            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeR[parent_nodeID].cost_to_come + 1
                    self.treeR["R"+str(self.right_idx)] = ArrNode(
                        child_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_cost_to_come, parent_nodeID)
                    self.rightLeaves.difference({parent_nodeID})
                    self.rightLeaves.add("R"+str(self.right_idx))
                    self.arrRightRegistr.append(child_arrangement)
                    self.idRightRegistr.append("R"+str(self.right_idx))
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
        while(len(queue) != 0):
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
                        self.leftLeaves.difference({parent_nodeID})

                elif child_arrangement in self.arrRightRegistr:
                    ### this is a sign that two trees are connected
                    ### check if it is really a bridge
                    if parent_arrangement not in self.arrRightRegistr:
                        print("a bridge is found")
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
                            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeL[parent_nodeID].cost_to_come + 1
                    self.treeL["L"+str(self.left_idx)] = ArrNode(
                        child_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_cost_to_come, parent_nodeID)
                    self.leftLeaves.difference({parent_nodeID})
                    self.leftLeaves.add("L"+str(self.left_idx))
                    self.arrLeftRegistr.append(child_arrangement)
                    self.idLeftRegistr.append("L"+str(self.left_idx))
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

        print("path: " + str(self.simplePath))
        self.totalActions = len(self.simplePath) - 1
        print("total action: " + str(self.totalActions))
        print("solution cost: " + str(self.best_solution_cost)) 


    def getMagicNumber(self, numObjs):
        temp_str = ''
        for i in range(numObjs):
            temp_str += '1'

        return int(temp_str, 2)

class BiDirDPPlanner_Leaf_Large_Range(object):
    ### Input:
    ### (1) initial_arrangement (a list of pose_ids, each of which indicating the initial pose for an object)
    ### (2) final_arrangement (a list of pose_ids, each of which indicating the final pose for an object)
    ### instance 
    ### (i) workspace, (ii) object centers/slots, (iii) buffer centers/slots
    ### visualTool: a visualization tool as a debugging purpose
    ### Output:
    ### the whole plan 
    def __init__(self, init_arr, final_arr, instance, Object_locations, region_dict, linked_list, visualTool):
        self.initial_arrangement = init_arr
        self.final_arrangement = final_arr
        self.points = instance.points + instance.buffer_points
        self.objects = instance.objects + instance.buffers
        self.numObjs = len(self.initial_arrangement)
        self.nPoses = len(self.points)
        self.allPoses = range(self.nPoses)
        # self.magicNumber = self.getMagicNumber(self.numObjs)
        # print("Magic number: " + str(self.magicNumber))

        ### initialize dependency_dict and path_dict as empty dict
        ### since now we are going to increment these two dicts online, instead of offline
        self.dependency_dict = {}
        self.path_dict = {}
        self.Object_locations = copy.deepcopy(Object_locations)
        self.region_dict = copy.deepcopy(region_dict)
        self.linked_list = copy.deepcopy(linked_list)

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
        self.bridge = [None, None, None, None, None] ### [leftKey, rightKey, object_transition, objectMoved, path_option]
        self.leftLeaves = {"L0"} ### keep track of leaves in the left tree
        self.rightLeaves = {"R0"} ### keep track of leaves in the right tree

        ################# testing #################
        self.num_mutation = 0
        self.mutation_time_list = []
        self.avg_time = 0.0
        self.total_time = 0.0

        ################## results ################
        self.isConnected = False
        self.best_solution_cost = np.inf
        ### the whole_path is a list of items and each item has the following format
        ### [("node1_id", node2_id), {2:path2, 1:path1, ...}]
        self.totalActions = 0 ### record the total number of actions
        self.numLeftBranches = 0 ### record the number of left branches in the solution
        self.numRightBranches = 0 ### record the number of right branches in the solution
        self.numNodesInLeftTree = 0 ### record the total number of nodes in the left tree
        self.numNodesInRightTree = 0 ### record the total number of nodes in the right tree

        ### start ruuning
        self.left_idx = 1
        self.right_idx = 1
        ### initial connection attempt

        self.growSubTree(self.treeL["L0"], self.treeR["R0"], "Left")
        if (self.isConnected != True):
            self.growSubTree(self.treeR["R0"], self.treeL["L0"], "Right")

        totalTime_allowed = 900.0 ### allow 500s for the total search tree construction
        start_time = time.clock()        

        while (self.isConnected != True and time.clock() - start_time < totalTime_allowed):
            ### The problem is not monotone
            start = time.time()
            newChild_nodeID = self.mutateLeftChild()
            if newChild_nodeID != None:
                best_goal = self.goal_arrangement_in_range(self.treeL[newChild_nodeID].arrangement, "Right")
                self.growSubTree(self.treeL[newChild_nodeID], self.treeR[best_goal], "Left")
            self.mutation_time_list.append(time.time()-start)
            self.num_mutation += 1


            start = time.time()
            if (self.isConnected != True):
                newChild_nodeID = self.mutateRightChild()
                if newChild_nodeID != None:
                    best_goal = self.goal_arrangement_in_range(self.treeR[newChild_nodeID].arrangement, "Left")
                    self.growSubTree(self.treeR[newChild_nodeID], self.treeL[best_goal], "Right")
            self.mutation_time_list.append(time.time()-start)
            self.num_mutation += 1


        if self.isConnected:
            self.getTheStat()

        if self.isConnected == False:
            print("fail the find a solution within " + str(totalTime_allowed) + " seconds...")

    def goal_arrangement_in_range(self, initial_arrangement, treeside):
        goal_range = int(math.ceil(self.numObjs/3.0))
        # goal_range = 3
        shortest_A_star_distance = float('inf')

        if treeside == "Right":
            best_goal = "R0"
            for node in self.treeR.values():
                temp_goal_arrangement = node.arrangement
                distance = 0
                A_star_distance = 0
                for i in range(len(temp_goal_arrangement)):
                    if temp_goal_arrangement[i] != initial_arrangement[i]:
                        distance += 1
                if distance<=goal_range:
                    A_star_distance = distance
                    for i in range(len(temp_goal_arrangement)):
                        if temp_goal_arrangement[i] != self.treeR["R0"].arrangement[i]:
                            A_star_distance += 1
                    if A_star_distance < shortest_A_star_distance:
                        shortest_A_star_distance = A_star_distance
                        best_goal = node.node_id
        
        if treeside == "Left":
            best_goal = "L0"
            for node in self.treeL.values():
                temp_goal_arrangement = node.arrangement
                distance = 0
                A_star_distance = 0
                for i in range(len(temp_goal_arrangement)):
                    if temp_goal_arrangement[i] != initial_arrangement[i]:
                        distance += 1
                if distance <= goal_range:
                    A_star_distance = distance
                    for i in range(len(temp_goal_arrangement)):
                        if temp_goal_arrangement[i] != self.treeL["L0"].arrangement[i]:
                            A_star_distance += 1
                    if A_star_distance < shortest_A_star_distance:
                        shortest_A_star_distance = A_star_distance
                        best_goal = node.node_id
                        
        return best_goal




    def mutateRightChild(self):
        ### first choose a node to mutate
        # print("Right mutation")
        # mutate_id = "R" + str(random.choice(range(len(self.treeR))))
        mutate_id = random.choice(list(self.rightLeaves))
        mutated_arrangement = self.treeR[mutate_id].arrangement
        ### choose an object to move
        obj_idx = random.choice(range(self.numObjs))
        ### choose a slot to put the object
        pose_idx = random.choice(self.allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))

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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [new_arrangement[obj_idx], mutated_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeR[mutate_id].cost_to_come
            self.treeR["R"+str(self.right_idx)] = ArrNode(
                        new_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.rightLeaves.difference({mutate_id})
            self.rightLeaves.add("R"+str(self.right_idx))
            self.arrRightRegistr.append(new_arrangement)
            self.idRightRegistr.append("R"+str(self.right_idx))
            self.right_idx += 1
            return self.idRightRegistr[self.arrRightRegistr.index(new_arrangement)]



    def mutateLeftChild(self):
        ### first choose a node to mutate
        # print("Left mutation")
        # mutate_id = "L" + str(random.choice(range(len(self.treeL))))
        mutate_id = random.choice(list(self.leftLeaves))
        mutated_arrangement = self.treeL[mutate_id].arrangement
        ### choose an object to move
        obj_idx = random.choice(range(self.numObjs))
        ### choose a slot to put the object
        pose_idx = random.choice(self.allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))

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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [mutated_arrangement[obj_idx], new_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeL[mutate_id].cost_to_come
            self.treeL["L"+str(self.left_idx)] = ArrNode(
                        new_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.leftLeaves.difference({mutate_id})
            self.leftLeaves.add("L"+str(self.left_idx))
            self.arrLeftRegistr.append(new_arrangement)
            self.idLeftRegistr.append("L"+str(self.left_idx))
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

        subTree = DFS_Rec_for_Monotone_General(
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
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
        while(len(queue) != 0):
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
                        self.rightLeaves.difference({parent_nodeID})

                elif child_arrangement in self.arrLeftRegistr:
                    ### this is a sign that two trees are connected
                    ### check if it is really a bridge
                    if parent_arrangement not in self.arrLeftRegistr:
                        print("a bridge is found")
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
                            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeR[parent_nodeID].cost_to_come + 1
                    self.treeR["R"+str(self.right_idx)] = ArrNode(
                        child_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_cost_to_come, parent_nodeID)
                    self.rightLeaves.difference({parent_nodeID})
                    self.rightLeaves.add("R"+str(self.right_idx))
                    self.arrRightRegistr.append(child_arrangement)
                    self.idRightRegistr.append("R"+str(self.right_idx))
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
        while(len(queue) != 0):
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
                        self.leftLeaves.difference({parent_nodeID})

                elif child_arrangement in self.arrRightRegistr:
                    ### this is a sign that two trees are connected
                    ### check if it is really a bridge
                    if parent_arrangement not in self.arrRightRegistr:
                        print("a bridge is found")
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
                            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeL[parent_nodeID].cost_to_come + 1
                    self.treeL["L"+str(self.left_idx)] = ArrNode(
                        child_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_cost_to_come, parent_nodeID)
                    self.leftLeaves.difference({parent_nodeID})
                    self.leftLeaves.add("L"+str(self.left_idx))
                    self.arrLeftRegistr.append(child_arrangement)
                    self.idLeftRegistr.append("L"+str(self.left_idx))
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

        print("path: " + str(self.simplePath))
        self.totalActions = len(self.simplePath) - 1
        print("total action: " + str(self.totalActions))
        print("solution cost: " + str(self.best_solution_cost)) 


    def getMagicNumber(self, numObjs):
        temp_str = ''
        for i in range(numObjs):
            temp_str += '1'

        return int(temp_str, 2)

class BiDirDPPlanner_Leaf_Nearest(object):
    ### Input:
    ### (1) initial_arrangement (a list of pose_ids, each of which indicating the initial pose for an object)
    ### (2) final_arrangement (a list of pose_ids, each of which indicating the final pose for an object)
    ### instance 
    ### (i) workspace, (ii) object centers/slots, (iii) buffer centers/slots
    ### visualTool: a visualization tool as a debugging purpose
    ### Output:
    ### the whole plan 
    def __init__(self, init_arr, final_arr, instance, Object_locations, region_dict, linked_list, visualTool):
        self.initial_arrangement = init_arr
        self.final_arrangement = final_arr
        self.points = instance.points + instance.buffer_points
        self.objects = instance.objects + instance.buffers
        self.numObjs = len(self.initial_arrangement)
        self.nPoses = len(self.points)
        self.allPoses = range(self.nPoses)
        # self.magicNumber = self.getMagicNumber(self.numObjs)
        # print("Magic number: " + str(self.magicNumber))

        ### initialize dependency_dict and path_dict as empty dict
        ### since now we are going to increment these two dicts online, instead of offline
        self.dependency_dict = {}
        self.path_dict = {}
        self.Object_locations = copy.deepcopy(Object_locations)
        self.region_dict = copy.deepcopy(region_dict)
        self.linked_list = copy.deepcopy(linked_list)

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
        self.bridge = [None, None, None, None, None] ### [leftKey, rightKey, object_transition, objectMoved, path_option]
        self.leftLeaves = {"L0"} ### keep track of leaves in the left tree
        self.rightLeaves = {"R0"} ### keep track of leaves in the right tree

        ################# testing #################
        self.num_mutation = 0
        self.mutation_time_list = []
        self.avg_time = 0.0
        self.total_time = 0.0

        ################## results ################
        self.isConnected = False
        self.best_solution_cost = np.inf
        ### the whole_path is a list of items and each item has the following format
        ### [("node1_id", node2_id), {2:path2, 1:path1, ...}]
        self.totalActions = 0 ### record the total number of actions
        self.numLeftBranches = 0 ### record the number of left branches in the solution
        self.numRightBranches = 0 ### record the number of right branches in the solution
        self.numNodesInLeftTree = 0 ### record the total number of nodes in the left tree
        self.numNodesInRightTree = 0 ### record the total number of nodes in the right tree

        ### start ruuning
        self.left_idx = 1
        self.right_idx = 1
        ### initial connection attempt

        self.growSubTree(self.treeL["L0"], self.treeR["R0"], "Left")
        if (self.isConnected != True):
            self.growSubTree(self.treeR["R0"], self.treeL["L0"], "Right")

        totalTime_allowed = 900.0 ### allow 500s for the total search tree construction
        start_time = time.clock()        

        while (self.isConnected != True and time.clock() - start_time < totalTime_allowed):
            ### The problem is not monotone
            start = time.time()
            newChild_nodeID = self.mutateLeftChild()
            if newChild_nodeID != None:
                best_goal = self.goal_arrangement_nearest(self.treeL[newChild_nodeID].arrangement, "Right")
                self.growSubTree(self.treeL[newChild_nodeID], self.treeR[best_goal], "Left")
            self.mutation_time_list.append(time.time()-start)
            self.num_mutation += 1


            start = time.time()
            if (self.isConnected != True):
                newChild_nodeID = self.mutateRightChild()
                if newChild_nodeID != None:
                    best_goal = self.goal_arrangement_nearest(self.treeR[newChild_nodeID].arrangement, "Left")
                    self.growSubTree(self.treeR[newChild_nodeID], self.treeL[best_goal], "Right")
            self.mutation_time_list.append(time.time()-start)
            self.num_mutation += 1


        if self.isConnected:
            self.getTheStat()

        if self.isConnected == False:
            print("fail the find a solution within " + str(totalTime_allowed) + " seconds...")

    def goal_arrangement_nearest(self, initial_arrangement, treeside):
        shortest_distance = float('inf')
        shortest_A_star_distance = float('inf')

        if treeside == "Right":
            for node in self.treeR.values():
                temp_goal_arrangement = node.arrangement
                distance = 0
                A_star_distance = 0
                for i in range(len(temp_goal_arrangement)):
                    if temp_goal_arrangement[i] != initial_arrangement[i]:
                        distance += 1
                A_star_distance = distance
                for i in range(len(temp_goal_arrangement)):
                    if temp_goal_arrangement[i] != self.treeR["R0"].arrangement[i]:
                        A_star_distance += 1
                if A_star_distance < shortest_A_star_distance:
                    shortest_distance = distance
                    shortest_A_star_distance = A_star_distance
                    best_goal = node.node_id
        
        if treeside == "Left":
            for node in self.treeL.values():
                temp_goal_arrangement = node.arrangement
                distance = 0
                A_star_distance = 0
                for i in range(len(temp_goal_arrangement)):
                    if temp_goal_arrangement[i] != initial_arrangement[i]:
                        distance += 1
                A_star_distance = distance
                for i in range(len(temp_goal_arrangement)):
                    if temp_goal_arrangement[i] != self.treeL["L0"].arrangement[i]:
                        A_star_distance += 1
                if A_star_distance < shortest_A_star_distance:
                    shortest_distance = distance
                    shortest_A_star_distance = A_star_distance
                    best_goal = node.node_id

        return best_goal




    def mutateRightChild(self):
        ### first choose a node to mutate
        # print("Right mutation")
        # mutate_id = "R" + str(random.choice(range(len(self.treeR))))
        mutate_id = random.choice(list(self.rightLeaves))
        mutated_arrangement = self.treeR[mutate_id].arrangement
        ### choose an object to move
        obj_idx = random.choice(range(self.numObjs))
        ### choose a slot to put the object
        pose_idx = random.choice(self.allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))

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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [new_arrangement[obj_idx], mutated_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeR[mutate_id].cost_to_come
            self.treeR["R"+str(self.right_idx)] = ArrNode(
                        new_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.rightLeaves.difference({mutate_id})
            self.rightLeaves.add("R"+str(self.right_idx))
            self.arrRightRegistr.append(new_arrangement)
            self.idRightRegistr.append("R"+str(self.right_idx))
            self.right_idx += 1
            return self.idRightRegistr[self.arrRightRegistr.index(new_arrangement)]



    def mutateLeftChild(self):
        ### first choose a node to mutate
        # print("Left mutation")
        # mutate_id = "L" + str(random.choice(range(len(self.treeL))))
        mutate_id = random.choice(list(self.leftLeaves))
        mutated_arrangement = self.treeL[mutate_id].arrangement
        ### choose an object to move
        obj_idx = random.choice(range(self.numObjs))
        ### choose a slot to put the object
        pose_idx = random.choice(self.allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))

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
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [mutated_arrangement[obj_idx], new_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_parent_cost = self.treeL[mutate_id].cost_to_come
            self.treeL["L"+str(self.left_idx)] = ArrNode(
                        new_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
            self.leftLeaves.difference({mutate_id})
            self.leftLeaves.add("L"+str(self.left_idx))
            self.arrLeftRegistr.append(new_arrangement)
            self.idLeftRegistr.append("L"+str(self.left_idx))
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

        subTree = DFS_Rec_for_Monotone_General(
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
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
        while(len(queue) != 0):
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
                        self.rightLeaves.difference({parent_nodeID})

                elif child_arrangement in self.arrLeftRegistr:
                    ### this is a sign that two trees are connected
                    ### check if it is really a bridge
                    if parent_arrangement not in self.arrLeftRegistr:
                        print("a bridge is found")
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
                            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeR[parent_nodeID].cost_to_come + 1
                    self.treeR["R"+str(self.right_idx)] = ArrNode(
                        child_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option, temp_cost_to_come, parent_nodeID)
                    self.rightLeaves.difference({parent_nodeID})
                    self.rightLeaves.add("R"+str(self.right_idx))
                    self.arrRightRegistr.append(child_arrangement)
                    self.idRightRegistr.append("R"+str(self.right_idx))
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
        while(len(queue) != 0):
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
                        self.leftLeaves.difference({parent_nodeID})

                elif child_arrangement in self.arrRightRegistr:
                    ### this is a sign that two trees are connected
                    ### check if it is really a bridge
                    if parent_arrangement not in self.arrRightRegistr:
                        print("a bridge is found")
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
                            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeL[parent_nodeID].cost_to_come + 1
                    self.treeL["L"+str(self.left_idx)] = ArrNode(
                        child_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option, temp_cost_to_come, parent_nodeID)
                    self.leftLeaves.difference({parent_nodeID})
                    self.leftLeaves.add("L"+str(self.left_idx))
                    self.arrLeftRegistr.append(child_arrangement)
                    self.idLeftRegistr.append("L"+str(self.left_idx))
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

        print("path: " + str(self.simplePath))
        self.totalActions = len(self.simplePath) - 1
        print("total action: " + str(self.totalActions))
        print("solution cost: " + str(self.best_solution_cost)) 


    def getMagicNumber(self, numObjs):
        temp_str = ''
        for i in range(numObjs):
            temp_str += '1'

        return int(temp_str, 2)


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

    def updateObjectTransition(object_transition):
        self.object_transition = object_transition

    def updateObjectMoved(objectMoved):
        self.objectMoved = objectMoved

    def updatePathOption(path_option):
        self.path_option = path_option

    def updateCostToCome(cost_to_come):
        self.cost_to_come = cost_to_come

    def updateParent(parent_id):
        self.parent_id = parent_id

    def getParentArr(self):
        parent_arr = copy.deepcopy(self.arrangement)
        if self.node_id[0] == 'L':
            parent_arr[self.objectMoved] = self.object_transition[0] ### move to a pose before transition
        else:
            parent_arr[self.objectMoved] = self.object_transition[1] ### move to a pose after transition

        return parent_arr

