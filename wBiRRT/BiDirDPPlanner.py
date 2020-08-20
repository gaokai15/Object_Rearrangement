from __future__ import division

# from DPLocalSolver import DFS_Rec_for_Monotone_General
from DPLocalSolver import DFS_Rec_for_Monotone
from util import *
import copy
import IPython
import time
import random
import numpy as np
from random import sample, choice
from collections import OrderedDict
from RegionGraphGenerator import RegionGraphGenerator


class BiDirDPPlanner(object):
    ### Input:
    ### (1) initial_arrangement (a list of pose_ids, each of which indicating the initial pose for an object)
    ### (2) final_arrangement (a list of pose_ids, each of which indicating the final pose for an object)
    ### instance
    ### (i) workspace, (ii) object centers/slots, (iii) buffer centers/slots
    ### visualTool: a visualization tool as a debugging purpose
    ### Output:
    ### the whole plan
    def __init__(
        self, init_arr, final_arr, instance, Object_locations, region_dict, linked_list, visualTool, wall_mink
    ):
        self.visualTool = visualTool
        self.instance = instance
        self.wall_mink = wall_mink
        self.initial_arrangement = init_arr
        self.final_arrangement = final_arr
        self.points = instance.points + instance.buffer_points
        self.objects = instance.objects + instance.buffers
        self.numObjs = len(self.initial_arrangement)
        self.nPoses = len(self.points)
        self.allPoses = range(self.nPoses)
        self.magicNumber = self.getMagicNumber(self.numObjs)
        print("Magic number: " + str(self.magicNumber))
        self.obj_use_buffer = {obj: set() for obj in range(self.numObjs)}

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
        self.bridge = [
            None, None, None, None, None
        ]  ### [leftKey, rightKey, object_transition, objectMoved, path_option]
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

        totalTime_allowed = 1000  ### allow 500s for the total search tree construction
        start_time = time.clock()

        while (self.isConnected != True and time.clock() - start_time < totalTime_allowed):
            ### The problem is not monotone
            newChild_nodeID = self.mutateLeftChild()
            if newChild_nodeID != None:
                self.growSubTree(self.treeL[newChild_nodeID], self.treeR["R0"], "Left")
            if (self.isConnected != True):
                newChild_nodeID = self.mutateRightChild()
                if newChild_nodeID != None:
                    self.growSubTree(self.treeR[newChild_nodeID], self.treeL["L0"], "Right")

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
        # pose_idx = random.choice(self.allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))

        ### START BLOCK ###
        # print("Obj_idx: ", obj_idx)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        ### choose a slot to put the object
        numBuffers = 1
        pickfrom = set(self.allPoses[self.numObjs * 2:])
        print("Pick:", pickfrom)
        pickfrom -= set(sum([list(y) if x != obj_idx else [] for x, y in self.obj_use_buffer.items()], []))
        print("Pick:", pickfrom)
        pose_idx = random.choice(list(pickfrom) + [2 * obj_idx] + [self.allPoses[-1] + 1] * numBuffers)
        # print("Pose_ind: ", pose_idx_ind)
        print("pose_idx: " + str(pose_idx), self.allPoses)

        if pose_idx not in self.allPoses:
            ### MAKE THIS INCREMENTAL LATER

            ### BUFFER POINTS / BUFFERS
            instance = self.instance.copy()
            ppoints = instance.points + instance.buffer_points
            minks = self.instance.minkowski_objs + self.instance.minkowski_buffers

            candidates = self.wall_mink
            end_pos = self.wall_mink
            for ind, obj in enumerate(mutated_arrangement):
                if ind != obj_idx:
                    candidates -= minks[obj]
                    end_pos -= minks[2 * ind]

            if self.visualTool.displayMore:
                self.visualTool.drawRegionGraph({0: [ppoints[mutated_arrangement[obj_idx]]]}, [candidates], label=False)
                self.visualTool.drawRegionGraph({0: [ppoints[2 * ind]]}, [end_pos], label=False)
                self.visualTool.drawRegionGraph(
                    {
                        0: [ppoints[mutated_arrangement[obj_idx]]],
                        1: [ppoints[2 * ind]]
                    }, [candidates & end_pos],
                    label=False
                )

            # b_points = set()
            reach_s = None
            reach_g = None
            for i, comp in enumerate(candidates):
                # check if buffer is reachable to start and goal
                x, y = ppoints[mutated_arrangement[obj_idx]]
                if not candidates.isHole(i) and candidates.isInside(x, y, i):
                    reach_s = comp
                    print("region reachable from start")
                    break
            # print(reach_s)

            for i, comp in enumerate(end_pos):
                x, y = ppoints[2 * obj_idx]
                if not end_pos.isHole(i) and end_pos.isInside(x, y, i):
                    reach_g = comp
                    print("region reachable from goal")
                    break
            # print(reach_g)

            if reach_s is None:
                return None
            if reach_g is None:
                # return None
                # b_points.update(reach_s)
                reach = pn.Polygon(reach)
            else:
                reach = pn.Polygon(reach_s) & pn.Polygon(reach_g)
                if not reach:
                    reach = pn.Polygon(reach_s)
                    # return None
                #     print(pu.pointList(reach))
                #     return None
                #     b_points.update(reach_s)
                # else:
                #     b_points.update(pu.pointList(reach))

            # print(b_points)
            # numBuffers = len(b_points)
            for i in range(numBuffers):
                # point = choice(list(b_points))
                # b_points.remove(point)
                point = reach.sample(random.random)
                instance.buffer_points.append(point)
                buff = instance.polygon + point
                instance.buffers.append([buff.tolist()])
                mink_obj = 2 * instance.polygon + point  ### grown_shape buffer
                instance.minkowski_buffers.append(pn.Polygon(mink_obj))
            ### BUFFER POINTS / BUFFERS
            # print("Before: ", instance.buffer_points)

            ### Now let's generate the region graph and build its connection
            regionGraph = RegionGraphGenerator(instance, self.visualTool, self.wall_mink)
            ### get the region dict and LL from the graph
            region_dict, linked_list = self.linked_list_conversion(regionGraph.graph)
            # print(region_dict)
            Object_locations = regionGraph.obj2reg
            # print(Object_locations)
            # print("After: ", instance.buffer_points)
            points = instance.points + instance.buffer_points
            objects = instance.objects + instance.buffers
            nPoses = len(points)
            self.obj_use_buffer[obj_idx].add(pose_idx)
            # allPoses = range(nPoses)
            # print(points)

        # pose_idx = allPoses[pose_idx_ind]  #random.choice(list(set(allPoses) - set(mutated_arrangement)))
        print(obj_idx, pose_idx, self.numObjs)
        # print(allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))
        ### END BLOCK ###

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
        # if pose_idx_ind >= len(self.allPoses):
        #     subTree = DFS_Rec_for_Monotone_General(
        #         start_poses, goal_poses, self.dependency_dict, self.path_dict, Object_locations, linked_list,
        #         region_dict
        #     )
        # else:
        #     subTree = DFS_Rec_for_Monotone_General(
        #         start_poses, goal_poses, self.dependency_dict, self.path_dict, self.Object_locations, self.linked_list,
        #         self.region_dict
        #     )
        if pose_idx not in self.allPoses:
            subTree = DFS_Rec_for_Monotone(
                mutated_arrangement, new_arrangement, self.dependency_dict, self.path_dict, Object_locations,
                linked_list, region_dict
            )
        else:
            subTree = DFS_Rec_for_Monotone(
                mutated_arrangement, new_arrangement, self.dependency_dict, self.path_dict, self.Object_locations,
                self.linked_list, self.region_dict
            )

        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            if pose_idx not in self.allPoses:
                ### update dependency_dict and path_dict
                self.dependency_dict = subTree.dependency_dict
                self.path_dict = subTree.path_dict
            return None
        else:
            ### update dependency_dict and path_dict
            self.dependency_dict = subTree.dependency_dict
            self.path_dict = subTree.path_dict
            if pose_idx not in self.allPoses:
                self.instance = instance
                self.points = instance.points + instance.buffer_points
                self.objects = instance.objects + instance.buffers
                self.nPoses = len(self.points)
                self.allPoses = range(self.nPoses)
                self.Object_locations = copy.deepcopy(Object_locations)
                self.region_dict = copy.deepcopy(region_dict)
                self.linked_list = linked_list

            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [new_arrangement[obj_idx], mutated_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            # temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_path_option = subTree.path_option[self.magicNumber]
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
        mutate_id = "L" + str(random.choice(range(len(self.treeL))))
        mutated_arrangement = self.treeL[mutate_id].arrangement
        ### choose an object to move
        obj_idx = random.choice(range(self.numObjs))
        ### choose a slot to put the object
        # pose_idx = random.choice(self.allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        print("obj_idx: " + str(obj_idx))

        ### START BLOCK ###
        # print("Obj_idx: ", obj_idx)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        ### choose a slot to put the object
        numBuffers = 1
        pickfrom = set(self.allPoses[self.numObjs * 2:])
        print("Pick:", pickfrom)
        pickfrom -= set(sum([list(y) if x != obj_idx else [] for x, y in self.obj_use_buffer.items()], []))
        print("Pick:", pickfrom)
        pose_idx = random.choice(list(pickfrom) + [2 * obj_idx + 1] + [self.allPoses[-1] + 1] * numBuffers)
        # print("Pose_ind: ", pose_idx_ind)
        print("pose_idx: " + str(pose_idx), self.allPoses)

        if pose_idx not in self.allPoses:
            ### MAKE THIS INCREMENTAL LATER

            ### BUFFER POINTS / BUFFERS
            instance = self.instance.copy()
            ppoints = instance.points + instance.buffer_points
            minks = self.instance.minkowski_objs + self.instance.minkowski_buffers

            candidates = self.wall_mink
            end_pos = self.wall_mink
            for ind, obj in enumerate(mutated_arrangement):
                if ind != obj_idx:
                    candidates -= minks[obj]
                    end_pos -= minks[2 * ind + 1]

            if self.visualTool.displayMore:
                self.visualTool.drawRegionGraph({0: [ppoints[mutated_arrangement[obj_idx]]]}, [candidates], label=False)
                self.visualTool.drawRegionGraph({0: [ppoints[2 * ind + 1]]}, [end_pos], label=False)
                self.visualTool.drawRegionGraph(
                    {
                        0: [ppoints[mutated_arrangement[obj_idx]]],
                        1: [ppoints[2 * ind + 1]]
                    }, [candidates & end_pos],
                    label=False
                )

            # b_points = set()
            reach_s = None
            reach_g = None
            for i, comp in enumerate(candidates):
                # check if buffer is reachable to start and goal
                x, y = ppoints[mutated_arrangement[obj_idx]]
                if not candidates.isHole(i) and candidates.isInside(x, y, i):
                    reach_s = comp
                    print("region reachable from start")
                    break
            # print(reach_s)

            for i, comp in enumerate(end_pos):
                x, y = ppoints[2 * obj_idx + 1]
                if not end_pos.isHole(i) and end_pos.isInside(x, y, i):
                    reach_g = comp
                    print("region reachable from goal")
                    break
            # print(reach_g)

            if reach_s is None:
                return None
            if reach_g is None:
                # return None
                # b_points.update(reach_s)
                reach = pn.Polygon(reach_s)
            else:
                reach = pn.Polygon(reach_s) & pn.Polygon(reach_g)
                if not reach:
                    reach = pn.Polygon(reach_s)
                    # return None
                #     print(pu.pointList(reach))
                #     return None
                #     b_points.update(reach_s)
                # else:
                #     b_points.update(pu.pointList(reach))

            # print(b_points)
            # numBuffers = len(b_points)
            for i in range(numBuffers):
                # point = choice(list(b_points))
                # b_points.remove(point)
                point = reach.sample(random.random)
                instance.buffer_points.append(point)
                buff = instance.polygon + point
                instance.buffers.append([buff.tolist()])
                mink_obj = 2 * instance.polygon + point  ### grown_shape buffer
                instance.minkowski_buffers.append(pn.Polygon(mink_obj))
            ### BUFFER POINTS / BUFFERS

            ### Now let's generate the region graph and build its connection
            regionGraph = RegionGraphGenerator(instance, self.visualTool, self.wall_mink)
            ### get the region dict and LL from the graph
            region_dict, linked_list = self.linked_list_conversion(regionGraph.graph)
            # print(region_dict)
            Object_locations = regionGraph.obj2reg
            # print(Object_locations)
            points = instance.points + instance.buffer_points
            objects = instance.objects + instance.buffers
            nPoses = len(points)
            self.obj_use_buffer[obj_idx].add(pose_idx)
            # print(points)

        # pose_idx = allPoses[pose_idx_ind]  #random.choice(list(set(allPoses) - set(mutated_arrangement)))
        print(obj_idx, pose_idx, self.numObjs)
        # print(allPoses)
        # print("mutated_arrangement: " + str(mutated_arrangement))
        # print("obj_idx: " + str(obj_idx))
        # print("pose_idx: " + str(pose_idx))
        ### END BLOCK ###

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
        # if pose_idx_ind >= len(self.allPoses):
        #     subTree = DFS_Rec_for_Monotone_General(
        #         start_poses, goal_poses, self.dependency_dict, self.path_dict, Object_locations, linked_list,
        #         region_dict
        #     )
        # else:
        #     subTree = DFS_Rec_for_Monotone_General(
        #         start_poses, goal_poses, self.dependency_dict, self.path_dict, self.Object_locations, self.linked_list,
        #         self.region_dict
        #     )
        if pose_idx not in self.allPoses:
            subTree = DFS_Rec_for_Monotone(
                mutated_arrangement, new_arrangement, self.dependency_dict, self.path_dict, Object_locations,
                linked_list, region_dict
            )
        else:
            subTree = DFS_Rec_for_Monotone(
                mutated_arrangement, new_arrangement, self.dependency_dict, self.path_dict, self.Object_locations,
                self.linked_list, self.region_dict
            )

        if subTree.isMonotone == False:
            # print("the mutation node cannot be connected")
            if pose_idx not in self.allPoses:
                ### update dependency_dict and path_dict
                self.dependency_dict = subTree.dependency_dict
                self.path_dict = subTree.path_dict
            return None
        else:
            ### update dependency_dict and path_dict
            self.dependency_dict = subTree.dependency_dict
            self.path_dict = subTree.path_dict
            if pose_idx not in self.allPoses:
                self.instance = instance
                self.points = instance.points + instance.buffer_points
                self.objects = instance.objects + instance.buffers
                self.nPoses = len(self.points)
                self.allPoses = range(self.nPoses)
                self.Object_locations = copy.deepcopy(Object_locations)
                self.region_dict = copy.deepcopy(region_dict)
                self.linked_list = linked_list

            ### we reach here since it is a duplicate and it can be connected
            ### welcome this new arrangement
            # print("the new arrangement after mutation has been accepted")
            temp_transition = [mutated_arrangement[obj_idx], new_arrangement[obj_idx]]
            temp_object_idx = obj_idx
            # temp_path_option = subTree.path_option[subTree.parent.keys()[0]]
            temp_path_option = subTree.path_option[self.magicNumber]
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

        # subTree = DFS_Rec_for_Monotone_General(
        #         start_poses, goal_poses, self.dependency_dict, self.path_dict, \
        #                 self.Object_locations, self.linked_list, self.region_dict)
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

    def linked_list_conversion(self, graph):
        # print "graph"
        # print graph
        region_dict = {}  # (1,2,'a'): 0
        LL = {}  # 0:[1,2,3]
        for key in graph:
            index = len(region_dict.keys())
            region_dict[key] = index
            LL[index] = []
        for key in graph:
            for v in graph[key]:
                LL[region_dict[key]].append(region_dict[v])
        # print "LL"
        # print self.LL
        # print "region dict"
        # print self.region_dict
        return region_dict, LL


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
            parent_arr[self.objectMoved] = self.object_transition[0]  ### move to a pose before transition
        else:
            parent_arr[self.objectMoved] = self.object_transition[1]  ### move to a pose after transition

        return parent_arr
