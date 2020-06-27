from __future__ import division


from DPLocalSolver import DFS_Rec_for_Monotone
from util import *
import copy
import IPython
import random
from random import sample
from collections import OrderedDict

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
        self.magicNumber = self.getMagicNumber(self.numObjs)
        print("Magic number: " + str(self.magicNumber))

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
        self.treeL["L0"] = ArrNode(self.initial_arrangement, "L0", None, None, None)
        self.treeR["R0"] = ArrNode(self.final_arrangement, "R0", None, None, None)
        self.arrLeftRegistr.append(self.initial_arrangement)
        self.arrRightRegistr.append(self.final_arrangement)
        self.idLeftRegistr.append("L0")
        self.idRightRegistr.append("R0")
        self.leftKey = "L0"
        self.rightKey = "R0"
        self.bridge = [] ### [leftKey, rightKey, object_transition, objectMoved, path_option]
        self.leftLeaves = ["L0"] ### keep track of leaves in the left tree
        self.rightLeaves = ["R0"] ### keep track of leaves in the right tree


        ################## results ################
        self.isConnected = False
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

        totalTimes = 2000
        timeout = totalTimes

        while (self.isConnected != True and timeout > 0):
            timeout -= 1
            # print("timeout: " + str(timeout))
            ### The problem is not monotone
            newChild_id = self.mutateLeftChild()
            if newChild_id != None:
                self.growSubTree(self.treeL[newChild_id], self.treeR["R0"], "Left")
            if (self.isConnected != True):
                newChild_id = self.mutateRightChild()
                if newChild_id != None:
                    self.growSubTree(self.treeR[newChild_id], self.treeL["L0"], "Right")


        if self.isConnected:
            self.getTheStat()

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
        subTree = DFS_Rec_for_Monotone(
            mutated_arrangement, new_arrangement, self.dependency_dict, self.path_dict, \
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
            print("the new arrangement after mutation has been accepted")
            temp_transition = [new_arrangement[obj_idx], mutated_arrangement[obj_idx]]
            if temp_transition[0] == temp_transition[1]:
                print("warning!! temp_transition check immediately")
                IPython.embed()
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[self.magicNumber]
            self.treeR["R"+str(self.right_idx)] = ArrNode(
                        new_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option)
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
        subTree = DFS_Rec_for_Monotone(
            mutated_arrangement, new_arrangement, self.dependency_dict, self.path_dict, \
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
            print("the new arrangement after mutation has been accepted")
            temp_transition = [mutated_arrangement[obj_idx], new_arrangement[obj_idx]]
            if temp_transition[0] == temp_transition[1]:
                print("warning!! temp_transition check immediately")
                IPython.embed()
            temp_object_idx = obj_idx
            temp_path_option = subTree.path_option[self.magicNumber]
            self.treeL["L"+str(self.left_idx)] = ArrNode(
                        new_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option)
            self.arrLeftRegistr.append(new_arrangement)
            self.idLeftRegistr.append("L"+str(self.left_idx))
            self.left_idx += 1
            return self.idLeftRegistr[self.arrLeftRegistr.index(new_arrangement)]




    def growSubTree(self, initNode, goalNode, treeSide):
        subTree = DFS_Rec_for_Monotone(
            initNode.arrangement, goalNode.arrangement, self.dependency_dict, self.path_dict, \
            self.Object_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict

        if treeSide == "Left":
            self.engraftingLeftTree(subTree, initNode, goalNode)
        else:
            self.engraftingRightTree(subTree, initNode, goalNode)


    def engraftingRightTree(self, subTree, rootNode, goalNode):
        ### enumerate the parent info (family tree) of the subTree
        ### all the keys represents all the new nodes to be added
        ### the values represents the parents
        print("Right tree")
        # print(subTree.parent)
        # print(subTree.path_option)

        for child_id, parent_id in subTree.parent.items():
            child_arrangement = self.encodeArrangement(child_id, rootNode.arrangement, goalNode.arrangement)
            parent_arrangement = self.encodeArrangement(parent_id, rootNode.arrangement, goalNode.arrangement)
            if child_arrangement == parent_arrangement:
                continue
            temp_object_idx = self.getTheObjectMoved(child_id, parent_id)
            temp_path_option = subTree.path_option[child_id]
            temp_transition = [child_arrangement[temp_object_idx], parent_arrangement[temp_object_idx]]
            # print("child_id " + str(child_id) + ": " + str(child_arrangement))

            if child_arrangement in self.arrRightRegistr:
                ### we don't add duplicate nodes at this point
                # print("duplicate node")
                continue
            elif child_arrangement in self.arrLeftRegistr:
                ### this is a sign that two trees are connected
                ### check if it is really a bridge
                if parent_arrangement not in self.arrLeftRegistr:
                    print("The tree can be connected")
                    self.isConnected = True
                    ### temporary record this node id
                    self.leftKey = self.idLeftRegistr[self.arrLeftRegistr.index(child_arrangement)]
                    # print("leftKey: " + str(self.leftKey))
                    self.rightKey_arr = parent_arrangement
                    self.bridge_objectMoved = temp_object_idx
                    self.bridge_path_option = temp_path_option
                    self.bridge_transition = temp_transition
            else:
                if temp_transition[0] == temp_transition[1]:
                    print("warning!! temp_transition check immediately")
                    IPython.embed()
                ### this is a new arrangement
                self.treeR["R"+str(self.right_idx)] = ArrNode(
                            child_arrangement, "R"+str(self.right_idx), temp_transition, temp_object_idx, temp_path_option)
                self.arrRightRegistr.append(child_arrangement)
                self.idRightRegistr.append("R"+str(self.right_idx))
                self.right_idx += 1

        ### You are reaching here since the subTree has been engrafted
        ### last step: if the bi-directional tree has been claimed to be connected before
        ### update the bridge information
        ### [leftKey, rightKey, object_transition, objectMoved, path_option]
        if self.isConnected:
            self.rightKey = self.idRightRegistr[self.arrRightRegistr.index(self.rightKey_arr)]
            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]



    def engraftingLeftTree(self, subTree, rootNode, goalNode):
        ### enumerate the parent info (family tree) of the subTree
        ### all the keys represents all the new nodes to be added
        ### the values represents the parents
        print("Left tree")
        # print(subTree.parent)
        # print(subTree.path_option)

        for child_id, parent_id in subTree.parent.items():
            child_arrangement = self.encodeArrangement(child_id, rootNode.arrangement, goalNode.arrangement)
            parent_arrangement = self.encodeArrangement(parent_id, rootNode.arrangement, goalNode.arrangement)
            if child_arrangement == parent_arrangement:
                continue
            temp_object_idx = self.getTheObjectMoved(child_id, parent_id)
            temp_path_option = subTree.path_option[child_id]
            temp_transition = [parent_arrangement[temp_object_idx], child_arrangement[temp_object_idx]]
            # print("child_id " + str(child_id) + ": " + str(child_arrangement))

            if child_arrangement in self.arrLeftRegistr:
                ### we don't add duplicate nodes at this point
                # print("duplicate node")
                continue
            elif child_arrangement in self.arrRightRegistr:
                ### this is a sign that two trees are connected
                ### check if it is really a bridge
                if parent_arrangement not in self.arrRightRegistr:
                    print("The tree can be connected")
                    self.isConnected = True
                    ### temporary record this node id
                    self.rightKey = self.idRightRegistr[self.arrRightRegistr.index(child_arrangement)]
                    # print("rightKey: " + str(self.rightKey))
                    self.leftKey_arr = parent_arrangement
                    self.bridge_objectMoved = temp_object_idx
                    self.bridge_path_option = temp_path_option
                    self.bridge_transition = temp_transition
            else:
                if temp_transition[0] == temp_transition[1]:
                    print("warning!! temp_transition check immediately")
                    IPython.embed()
                ### this is a new arrangement
                self.treeL["L"+str(self.left_idx)] = ArrNode(
                            child_arrangement, "L"+str(self.left_idx), temp_transition, temp_object_idx, temp_path_option)
                self.arrLeftRegistr.append(child_arrangement)
                self.idLeftRegistr.append("L"+str(self.left_idx))
                self.left_idx += 1

        ### You are reaching here since the subTree has been engrafted
        ### last step: if the bi-directional tree has been claimed to be connected before
        ### update the bridge information
        ### [leftKey, rightKey, object_transition, objectMoved, path_option]
        if self.isConnected:
            self.leftKey = self.idLeftRegistr[self.arrLeftRegistr.index(self.leftKey_arr)]
            self.bridge = [self.leftKey, self.rightKey, self.bridge_transition, self.bridge_objectMoved, self.bridge_path_option]



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
        print("Let's get stats!!")
        ### This function is used to get the statistics if the path is found
        self.numNodesInLeftTree = len(self.treeL)
        self.numNodesInRightTree = len(self.treeR)
        self.simplePath = []
        ### from leftKey, back track to left root via parent search 
        curr_waypoint_id = self.leftKey
        self.simplePath.insert(0, curr_waypoint_id)
        while curr_waypoint_id != "L0":
            nextNode_arr = self.treeL[curr_waypoint_id].getParentArr()
            curr_waypoint_id = self.idLeftRegistr[self.arrLeftRegistr.index(nextNode_arr)]
            print("curr_waypoint_id: " + str(curr_waypoint_id))
            self.simplePath.insert(0, curr_waypoint_id)
        ### from rightKey, back track to right root via parent search
        curr_waypoint_id = self.rightKey
        self.simplePath.append(curr_waypoint_id)
        while curr_waypoint_id != "R0":
            nextNode_arr = self.treeR[curr_waypoint_id].getParentArr()
            curr_waypoint_id = self.idRightRegistr[self.arrRightRegistr.index(nextNode_arr)]
            print("curr_waypoint_id: " + str(curr_waypoint_id))
            self.simplePath.append(curr_waypoint_id)

        print("path: " + str(self.simplePath))
        self.totalActions = len(self.simplePath) - 1
        print("total action: " + str(self.totalActions))


    def getMagicNumber(self, numObjs):
        temp_str = ''
        for i in range(numObjs):
            temp_str += '1'

        return int(temp_str, 2)



class ArrNode(object):
    def __init__(self, arrangement, node_id, object_transition, objectMoved, path_option):
        self.arrangement = arrangement
        self.node_id = node_id
        ### object_transition indicates the pose for certain objects before and after the transition
        ### For example, object 4 moves from pose 1 to pose 3, then object_transition = [1, 3]
        self.object_transition = object_transition
        self.objectMoved = objectMoved
        self.path_option = path_option

    def getParentArr(self):
        parent_arr = copy.deepcopy(self.arrangement)
        if self.node_id[0] == 'L':
            parent_arr[self.objectMoved] = self.object_transition[0] ### move to a pose before transition
        else:
            parent_arr[self.objectMoved] = self.object_transition[1] ### move to a pose after transition

        return parent_arr

