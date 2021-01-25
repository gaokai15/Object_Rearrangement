from __future__ import division


# from DPLocalSolver import DFS_Rec_for_Monotone_General
from mRS import MRS_for_Non_Monotone
from util import *
import os
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

class UniDir_mRS_Planner(object):
    ### Input:
    ### (1) initial_arrangement (a list of pose_ids, each of which indicating the initial pose for an object)
    ### (2) final_arrangement (a list of pose_ids, each of which indicating the final pose for an object)
    ### instance 
    ### (i) workspace, (ii) object centers/slots, (iii) buffer centers/slots
    ### visualTool: a visualization tool as a debugging purpose

    ### Output:
    ### the whole plan 
    def __init__(self, init_arr, final_arr, instance, gpd, new_paths, polygon, RAD, visualTool):
        self.initial_arrangement = init_arr
        self.final_arrangement = final_arr
        self.points = instance.points + instance.buffer_points
        self.poses = instance.objects + instance.buffers
        self.numObjs = len(self.initial_arrangement)
        self.nPoses = len(self.points)
        self.allPoses = range(self.nPoses)
        self.polygon = polygon
        self.RAD = RAD
        self.visualTool = visualTool

        ### initialize dependency_dict and path_dict as empty dict
        ### since now we are going to increment these two dicts online, instead of offline
        self.dependency_dict = copy.deepcopy(gpd.dependency_dict)
        self.path_dict = copy.deepcopy(gpd.path_dict)
        self.new_paths = new_paths
        self.pose_locations = copy.deepcopy(gpd.pose_locations)
        self.region_dict = copy.deepcopy(gpd.region_dict)
        self.linked_list = copy.deepcopy(gpd.LL)
        self.getStraightPaths()

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
        ### members in a node: arrangement, node_id, object_transition, objectMoved, path_option, cost_to_come, parent_id
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


        ################## results ################
        self.isConnected = False
        self.best_solution_cost = np.inf
        ### the whole_path is a list of items and each item has the following format
        ### [("node1_id", node2_id), {2:path2, 1:path1, ...}]
        self.whole_path = []
        self.totalActions = 0 ### record the total number of actions
        self.numLeftBranches = 0 ### record the number of left branches in the solution
        self.numRightBranches = 0 ### record the number of right branches in the solution
        self.numNodesInLeftTree = 0 ### record the total number of nodes in the left tree
        self.numNodesInRightTree = 0 ### record the total number of nodes in the right tree

        ### start ruuning
        self.left_idx = 1
        self.right_idx = 1

        self.mutation = 0

        ### initial connection attempt
        self.growSubTree(self.treeL["L0"], self.treeR["R0"], "Left")
        # if (self.isConnected != True):
        #     self.growSubTree(self.treeR["R0"], self.treeL["L0"], "Right")

        totalTime_allowed = 20*self.numObjs ### allow 500s for the total search tree construction
        start_time = time.clock()        

        while (self.isConnected != True and time.clock() - start_time < totalTime_allowed):
            ### The problem is not monotone
            newChild_nodeID = self.mutateLeftChild()
            if newChild_nodeID != None:
                self.growSubTree(self.treeL[newChild_nodeID], self.treeR["R0"], "Left")

            # if (self.isConnected != True and time.clock() - start_time < totalTime_allowed):
            #     newChild_nodeID = self.mutateRightChild()
            #     if newChild_nodeID != None:
            #         self.growSubTree(self.treeR[newChild_nodeID], self.treeL["L0"], "Right")


        if self.isConnected:
            # self.getSolutionStats()
            return 

        ### handle failure
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
        # time_monotoneCall = time.clock()
        # subTree = DFS_Rec_for_Monotone_General(
        #     start_poses, goal_poses, self.dependency_dict, self.path_dict, \
        #     self.pose_locations, self.linked_list, self.region_dict)
        subTree = MRS_for_Non_Monotone(
            start_poses, goal_poses, \
            self.pose_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        # print("time_monotoneCall: " + str(time.clock() - time_monotoneCall))
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is not a duplicate and it can be connected
            temp_transition = [new_arrangement[obj_idx], mutated_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            ### first check if it directly mutated to a node on the other side
            if new_arrangement in self.arrLeftRegistr:
                ### a bridge has been found
                # print("a bridge has been found during mutation!")
                self.isConnected = True
                ### check if it leads to a better solution
                temp_leftKey = self.idLeftRegistr[self.arrLeftRegistr.index(new_arrangement)]
                temp_rightKey = self.idRightRegistr[self.arrRightRegistr.index(mutated_arrangement)]
                temp_cost = self.treeL[temp_leftKey].cost_to_come + self.treeR[temp_rightKey].cost_to_come + 1
                # print("its cost is: " + str(temp_cost))
                if temp_cost < self.best_solution_cost:
                    ### This is a better solution
                    self.best_solution_cost = temp_cost
                    ### let's update the bridge
                    self.leftKey = temp_leftKey
                    self.rightKey = temp_rightKey
                    self.bridge_transition = temp_transition
                    self.bridge_objectMoved = temp_object_idx
                    self.bridge_path_option = temp_path_option
                    self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, \
                                            self.bridge_objectMoved, self.bridge_path_option]
                    # print("best solution cost so far: " + str(self.best_solution_cost))
                return None

            else:
                ### This is a new arrangement, welcome!
                # print("the new arrangement after mutation has been accepted")
                temp_parent_cost = self.treeR[mutate_id].cost_to_come
                self.treeR["R"+str(self.right_idx)] = ArrNode(
                            new_arrangement, "R"+str(self.right_idx), \
                            temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
                self.arrRightRegistr.append(new_arrangement)
                self.idRightRegistr.append("R"+str(self.right_idx))
                ### visualize the newly-added branch
                # self.visualizeLocalBranch("R"+str(self.right_idx), mutate_id, "R,R")
                self.right_idx += 1

                self.mutation += 1
                # print("mutation: " + str(self.mutation))
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
        # time_monotoneCall = time.clock()
        # subTree = DFS_Rec_for_Monotone_General(
        #     start_poses, goal_poses, self.dependency_dict, self.path_dict, \
        #     self.pose_locations, self.linked_list, self.region_dict)
        subTree = MRS_for_Non_Monotone(
            start_poses, goal_poses, \
            self.pose_locations, self.linked_list, self.region_dict)        
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        # print("time_monotoneCall: " + str(time.clock() - time_monotoneCall))
        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            return None
        else:
            ### we reach here since it is not a duplicate and it can be connected
            temp_transition = [mutated_arrangement[obj_idx], new_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            ### first check if it directly mutated to a node on the other side
            if new_arrangement in self.arrRightRegistr:
                ### a bridge has been found
                # print("a bridge has been found during mutation!")
                self.isConnected = True
                ### check if it leads to a better solution
                temp_leftKey = self.idLeftRegistr[self.arrLeftRegistr.index(mutated_arrangement)]
                temp_rightKey = self.idRightRegistr[self.arrRightRegistr.index(new_arrangement)]
                temp_cost = self.treeL[temp_leftKey].cost_to_come + self.treeR[temp_rightKey].cost_to_come + 1
                # print("its cost is: " + str(temp_cost))
                if temp_cost < self.best_solution_cost:
                    ### This is a better solution
                    self.best_solution_cost = temp_cost
                    ### let's update the bridge
                    self.leftKey = temp_leftKey
                    self.rightKey = temp_rightKey
                    self.bridge_transition = temp_transition
                    self.bridge_objectMoved = temp_object_idx
                    self.bridge_path_option = temp_path_option
                    self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, \
                                            self.bridge_objectMoved, self.bridge_path_option]
                    # print("best solution cost so far: " + str(self.best_solution_cost))
                return None

            else:            
                ### This is a new arrangement, welcome!
                # print("the new arrangement after mutation has been accepted")
                temp_parent_cost = self.treeL[mutate_id].cost_to_come
                self.treeL["L"+str(self.left_idx)] = ArrNode(
                            new_arrangement, "L"+str(self.left_idx), \
                            temp_transition, temp_object_idx, temp_path_option, temp_parent_cost+1, mutate_id)
                self.arrLeftRegistr.append(new_arrangement)
                self.idLeftRegistr.append("L"+str(self.left_idx))
                ### visualize the newly-added branch
                # self.visualizeLocalBranch("L"+str(self.left_idx), mutate_id, "L,L")
                self.left_idx += 1

                self.mutation += 1
                # print("mutation: " + str(self.mutation))
                return self.idLeftRegistr[self.arrLeftRegistr.index(new_arrangement)]




    def growSubTree(self, initNode, goalNode, treeSide):
        ### construct start_poses and goal_poses
        start_poses = {}
        goal_poses = {}
        for i in range(len(initNode.arrangement)):
            start_poses[i] = initNode.arrangement[i]
        for i in range(len(goalNode.arrangement)):
            goal_poses[i] = goalNode.arrangement[i]

        # time_monotoneCall = time.clock()
        # subTree = DFS_Rec_for_Monotone_General(
        #     start_poses, goal_poses, self.dependency_dict, self.path_dict, \
        #     self.pose_locations, self.linked_list, self.region_dict)
        subTree = MRS_for_Non_Monotone(
            start_poses, goal_poses, \
            self.pose_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        # print("time_monotoneCall: " + str(time.clock() - time_monotoneCall))

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
                        # print("a bridge is found")
                        self.isConnected = True
                        ### check if it leads to a better solution
                        temp_leftKey = self.idLeftRegistr[self.arrLeftRegistr.index(child_arrangement)]
                        temp_rightKey = self.idRightRegistr[self.arrRightRegistr.index(parent_arrangement)]
                        temp_cost = self.treeL[temp_leftKey].cost_to_come + self.treeR[temp_rightKey].cost_to_come + 1
                        # print("its cost is: " + str(temp_cost))
                        if temp_cost < self.best_solution_cost:
                            ### This is a better solution
                            self.best_solution_cost = temp_cost
                            ### let's update the bridge
                            self.leftKey = temp_leftKey
                            self.rightKey = temp_rightKey
                            self.bridge_transition = temp_transition
                            self.bridge_objectMoved = temp_object_idx
                            self.bridge_path_option = temp_path_option
                            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, \
                                                    self.bridge_objectMoved, self.bridge_path_option]
                            # print("best solution cost so far: " + str(self.best_solution_cost))

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeR[parent_nodeID].cost_to_come + 1
                    self.treeR["R"+str(self.right_idx)] = ArrNode(
                        child_arrangement, "R"+str(self.right_idx), \
                        temp_transition, temp_object_idx, temp_path_option, temp_cost_to_come, parent_nodeID)
                    self.arrRightRegistr.append(child_arrangement)
                    self.idRightRegistr.append("R"+str(self.right_idx))
                    ### visualize the newly-added branch
                    # self.visualizeLocalBranch("R"+str(self.right_idx), parent_nodeID, "R,R")
                    self.right_idx += 1
                    ### add this child node into queue
                    queue.insert(0, child_id)

        # if self.isConnected == True:
        #     self.visualizeLocalBranch(self.leftKey, self.rightKey, "L,R")


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
                        ### It indicates that the current checked parent is a better parent since it costs less
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
                        # print("a bridge is found")
                        self.isConnected = True
                        ### check if it leads to a better solution
                        temp_leftKey = self.idLeftRegistr[self.arrLeftRegistr.index(parent_arrangement)]
                        temp_rightKey = self.idRightRegistr[self.arrRightRegistr.index(child_arrangement)]
                        temp_cost = self.treeL[temp_leftKey].cost_to_come + self.treeR[temp_rightKey].cost_to_come + 1
                        # print("its cost is: " + str(temp_cost))
                        if temp_cost < self.best_solution_cost:
                            ### This is a better solution
                            self.best_solution_cost = temp_cost
                            ### let's update the bridge
                            self.leftKey = temp_leftKey
                            self.rightKey = temp_rightKey
                            self.bridge_transition = temp_transition
                            self.bridge_objectMoved = temp_object_idx
                            self.bridge_path_option = temp_path_option
                            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, \
                                                    self.bridge_objectMoved, self.bridge_path_option]
                            # print("best solution cost so far: " + str(self.best_solution_cost))

                else:
                    ### This is a brand new arrangement, let's add to the tree
                    temp_cost_to_come = self.treeL[parent_nodeID].cost_to_come + 1
                    self.treeL["L"+str(self.left_idx)] = ArrNode(
                        child_arrangement, "L"+str(self.left_idx), \
                        temp_transition, temp_object_idx, temp_path_option, temp_cost_to_come, parent_nodeID)
                    self.arrLeftRegistr.append(child_arrangement)
                    self.idLeftRegistr.append("L"+str(self.left_idx))
                    ### visualize the newly-added branch
                    # self.visualizeLocalBranch("L"+str(self.left_idx), parent_nodeID, "L,L")
                    self.left_idx += 1
                    ### add this child node into the queue(BFS)
                    queue.insert(0, child_id)

        # if self.isConnected == True:
        #     self.visualizeLocalBranch(self.leftKey, self.rightKey, "L,R")



    def getTheObjectMoved(self, child_id, parent_id):
        for i in range(self.numObjs):
            bit_stat1 = checkBitStatusAtPos(child_id, i)
            bit_stat2 = checkBitStatusAtPos(parent_id, i)
            if (bit_stat1 != bit_stat2):
                ### since we know currently it will be just one object
                return i

        return None


    def encodeArrangement(self, new_node_id, init_arrangement, goal_arrangement):
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
                new_arrangement.append(init_arrangement[i])

        return new_arrangement


    def getStraightPaths(self):
        ### Before we perform search and increment the dependency and path dict
        ### Let's use the straight path as the first and backup path
        ### for each pair of pose

        ### set a distance length which is used as landmark to discretize
        dist_piece = self.RAD

        for i in range(self.nPoses):
            for j in range(i, self.nPoses):
                key_pair = (i, j)
                # print("key_pair: " + str(key_pair))
                if key_pair not in self.dependency_dict.keys():
                    self.dependency_dict[key_pair] = []
                    self.path_dict[key_pair] = []

                    path = []
                    dep_set = set()

                if i == j:
                    path.append(self.points[i])
                    self.path_dict[key_pair].append(path)
                    dep_set = dep_set.union({i})
                    self.dependency_dict[key_pair].append(dep_set)
                    continue

                ### i != j
                start_pt = self.points[i]
                goal_pt = self.points[j]
                straight_dist = dist(start_pt, goal_pt)
                nsegs = max(1, int(straight_dist/dist_piece)) ### at least 1 nsegs
                keypts_check = 5 ### only add every 5 pts
                for kk in range(nsegs+1):
                    temp_ptx = start_pt[0] + (goal_pt[0] - start_pt[0]) / nsegs * kk
                    temp_pty = start_pt[1] + (goal_pt[1] - start_pt[1]) / nsegs * kk
                    temp_pt = (temp_ptx, temp_pty)
                    ### check with every pose except its own start and goal pose
                    for ii in range(len(self.poses)):
                        isColliFree = isCollisionFree(self.polygon, temp_pt, self.poses[ii])
                        if not isColliFree:
                            dep_set = dep_set.union({ii})
                    if kk % keypts_check == 0:
                        path.append(temp_pt)
                self.dependency_dict[key_pair].append(dep_set)
                self.path_dict[key_pair].append(path)

        # print("dependency_dict: ")
        # for key_pair, dependency_set in self.dependency_dict.items():
        #     print(str(key_pair) + ": " + str(dependency_set))

        # print("path_dict: ")
        # for key_pair, path in self.path_dict.items():
        #     print(str(key_pair) + ": " + str(path))

    def visualizeLocalBranch(self, child_id, parent_id, connectMode):
        ### Three connect modes
        ### (1) "L,L"
        ### (2) "R,R"
        ### (3) "L,R"
        if connectMode == "L,L":
            childNode = self.treeL[child_id]
            parentNode = self.treeL[parent_id]
            # print "current arrangement: " + str(parent_id) + ", " + str(parentNode.arrangement)
            # print "next arrangement: " + str(child_id) + ", " + str(childNode.arrangement)
            temp_ppath = self.getPath(child_id, parent_id, "Left")
            self.visualTool.drawLocalMotions((parent_id, child_id), temp_ppath,
                self.points, self.poses, parentNode.arrangement, self.final_arrangement, "save-to-tree")


        elif connectMode == "R,R":
            childNode = self.treeR[child_id]
            parentNode = self.treeR[parent_id]          
            # print "current arrangement: " + str(child_id) + ", " + str(childNode.arrangement)
            # print "next arrangement: " + str(parent_id) + ", " + str(parentNode.arrangement)
            temp_ppath = self.getPath(child_id, parent_id, "Right")
            self.visualTool.drawLocalMotions((child_id, parent_id), temp_ppath,
                self.points, self.poses, childNode.arrangement, self.final_arrangement, "save-to-tree")

        else:
            ### connectMode == "L,R"
            leftNode = self.treeL[child_id]
            rightNode = self.treeR[parent_id]
            # print "current arrangement: " + str(child_id) + ", " + str(childNode.arrangement)
            # print "next arrangement: " + str(parent_id) + ", " + str(parentNode.arrangement)   
            temp_ppath = self.getPath(child_id, parent_id, "Bridge")
            self.visualTool.drawLocalMotions((child_id, parent_id), temp_ppath,
                self.points, self.poses, leftNode.arrangement, self.final_arrangement, "save-to-tree")


    def getPath(self, node_id, parent_id, treeMode):

        result_path = {} ### key: objectMoved, value: [(x0,y0), (x1,y1), (x2,y2),..., (xn,yn)]

        if treeMode == "Bridge":
            start_key = self.bridge[2][0]
            goal_key = self.bridge[2][1]
            key = (min(start_key, goal_key), max(start_key, goal_key))
            objectMoved = self.bridge[3]

        else:
            start_key = self.trees[treeMode][node_id].object_transition[0]
            goal_key = self.trees[treeMode][node_id].object_transition[1]
            key = (min(start_key, goal_key), max(start_key, goal_key))
            objectMoved = self.trees[treeMode][node_id].objectMoved   
        
        ### now check if it is straight path or region path
        if (treeMode == "Bridge" and self.bridge[4] == 0) or (treeMode != "Bridge" and self.trees[treeMode][node_id].path_option == 0):
            ### it is a straight path
            start_pt = self.points[start_key]
            goal_pt = self.points[goal_key]
            nsteps = 3
            result_path[objectMoved] = []
            for step in range(nsteps+1):
                pt_x = start_pt[0] + (goal_pt[0] - start_pt[0]) / nsteps * step
                pt_y = start_pt[1] + (goal_pt[1] - start_pt[1]) / nsteps * step
                result_path[objectMoved].append((pt_x, pt_y))                     

        else:
            ### it is a region region path (rpath)
            if treeMode == "Bridge":
                rpath = copy.deepcopy(self.path_dict[key][self.bridge[4]])
            else:
                rpath = copy.deepcopy(self.path_dict[key][self.trees[treeMode][node_id].path_option])

            if start_key > goal_key:
                rpath = list(reversed(rpath))

            ppath = [self.points[start_key]]
            for i in range(len(rpath) - 2):
                region1 = rpath[i]
                region2 = rpath[i + 1]
                if (region1, region2) in self.new_paths.keys():
                    ppath += self.new_paths[(region1, region2)][:-1] ### only add region1 point
                elif (region2, region1) in self.new_paths.keys():
                    ppath += list(reversed(self.new_paths[(region2, region1)]))[:-1]
                else:
                    print "invalid path 1"
                    exit()

            if len(rpath) >= 2:
                ### add the last two region points
                region1 = rpath[-2]
                region2 = rpath[-1]
                if (region1, region2) in self.new_paths.keys():
                    ppath += self.new_paths[(region1, region2)]
                elif (region2, region1) in self.new_paths.keys():
                    ppath += list(reversed(self.new_paths[(region2, region1)]))
                else:
                    print "invalid path 2"
                    exit()
            
            ppath.append(self.points[goal_key])

            ### A single improvement: remove the second and second-to-last point in the ppath
            ppath.pop(1)
            ppath.pop(len(ppath)-2)

            result_path[objectMoved] = ppath

        return result_path



    # def getPath_new(self, node_id, parent_id, treeMode):
    #     ### first get that local path from path_dict
    #     if treeMode == "Bridge":
    #         start_key = self.bridge[2][0]
    #         goal_key = self.bridge[2][1]
    #         key = (min(start_key, goal_key), max(start_key, goal_key))
    #         objectMoved = self.bridge[3]
    #         rpath = copy.deepcopy(self.path_dict[key][self.bridge[4]])
    #         if start_key > goal_key:
    #             rpath = list(reversed(rpath))

    #     else:
    #         start_key = self.trees[treeMode][node_id].object_transition[0]
    #         goal_key = self.trees[treeMode][node_id].object_transition[1]
    #         key = (min(start_key, goal_key), max(start_key, goal_key))
    #         objectMoved = self.trees[treeMode][node_id].objectMoved
    #         rpath = copy.deepcopy(self.path_dict[key][self.trees[treeMode][node_id].path_option])
    #         if start_key > goal_key:
    #             rpath = list(reversed(rpath))

    #     ppath = [self.points[start_key]]
    #     for i in range(len(rpath) - 2):
    #         region1 = rpath[i]
    #         region2 = rpath[i + 1]
    #         if (region1, region2) in self.new_paths.keys():
    #             ppath += self.new_paths[(region1, region2)][:-1] ### only add region1 point
    #         elif (region2, region1) in self.new_paths.keys():
    #             ppath += list(reversed(self.new_paths[(region2, region1)]))[:-1]
    #         else:
    #             print "invalid path 1"
    #             exit()

    #     if len(rpath) >= 2:
    #         ### add the last two region points
    #         region1 = rpath[-2]
    #         region2 = rpath[-1]
    #         if (region1, region2) in self.new_paths.keys():
    #             ppath += self.new_paths[(region1, region2)]
    #         elif (region2, region1) in self.new_paths.keys():
    #             ppath += list(reversed(self.new_paths[(region2, region1)]))
    #         else:
    #             print "invalid path 2"
    #             exit()
        
    #     ppath.append(self.points[goal_key])

    #     ### A single improvement: remove the second and second-to-last point in the ppath
    #     ppath.pop(1)
    #     ppath.pop(len(ppath)-2)

    #     result_path = {}
    #     result_path[objectMoved] = ppath


    #     return result_path


    def constructWholePath(self):
        ### from leftKey, back track to left root via parent search (get all paths from the left tree)
        curr_waypoint = self.leftKey
        # print("construct the path on the left tree")
        while curr_waypoint != "L0":
            temp_parent = self.treeL[curr_waypoint].parent_id
            result_path = self.getPath(curr_waypoint, temp_parent, "Left")
            self.whole_path.insert(0, [(temp_parent, curr_waypoint), result_path])
            ### move to its parents
            curr_waypoint = self.treeL[curr_waypoint].parent_id

        ### Now add the bridge to the whole path
        # print("building the bridge betwen left tree and right tree")
        result_path = self.getPath(self.leftKey, self.rightKey, "Bridge")
        self.whole_path.append([(self.leftKey, self.rightKey), result_path])

        ### from rightKey, back track to right root via parent search (get all paths from the right tree)
        curr_waypoint = self.rightKey
        # print("construct the path on the right tree")
        while curr_waypoint != "R0":
            temp_parent = self.treeR[curr_waypoint].parent_id
            result_path = self.getPath(curr_waypoint, temp_parent, "Right")
            self.whole_path.append([(curr_waypoint, temp_parent), result_path])
            ### move to the its parent
            curr_waypoint = self.treeR[curr_waypoint].parent_id


    def getSolutionStats(self):
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

        enablePrint()
        # print("path: " + str(self.simplePath))
        self.totalActions = len(self.simplePath) - 1
        # print("total action: " + str(self.totalActions))
        # print("solution cost: " + str(self.best_solution_cost))
        # blockPrint()

        time_computeOrdering = time.clock()
        ### add ordering here
        ### add non-monotone actions here
        self.object_ordering = [] ### a list of obj_idx
        self.actions = OrderedDict() ### (a dict) key: nodeID-nodeID, value: [obj_idx, [from_pose_idx, to_pose_idx]]
        for kk in range(1, len(self.simplePath)):
            pointNode_id = self.simplePath[kk]
            prev_pointNode_id = self.simplePath[kk-1]
            if pointNode_id[0] == "L" and prev_pointNode_id[0] == "L":
                self.object_ordering.append(self.treeL[pointNode_id].objectMoved)
                self.actions[prev_pointNode_id+"-"+pointNode_id] = [
                    self.treeL[pointNode_id].objectMoved, self.treeL[pointNode_id].object_transition]

            elif pointNode_id[0] == "R" and prev_pointNode_id[0] == "R":
                self.object_ordering.append(self.treeR[prev_pointNode_id].objectMoved)
                self.actions[prev_pointNode_id+"-"+pointNode_id] = [
                    self.treeR[prev_pointNode_id].objectMoved, self.treeR[prev_pointNode_id].object_transition]

            else:
                ### pointNode_id[0] == "R" and prev_pointNode_id[0] == "L"
                self.object_ordering.append(self.bridge[3])
                self.actions[self.bridge[0]+"-"+self.bridge[1]] = [
                    self.bridge[3], self.bridge[2]]

        print("object_ordering: " + str(self.object_ordering))
        # for transition, obj_pose in self.actions.items():
        #     print(transition + ": " + str(obj_pose))

        self.totalActions = 1
        for oo in range(1, len(self.object_ordering)):
            if self.object_ordering[oo] != self.object_ordering[oo-1]:
                self.totalActions += 1
        print("total action: " + str(self.totalActions))




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

