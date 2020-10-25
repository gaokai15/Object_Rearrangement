from __future__ import division

from ILPSolver import feedback_arc_ILP_buffers
from DPLocalSolver import DFS_Rec_for_Monotone_General, DFS_Rec_for_Non_Monotone_General
from util import *
import os
import copy
import IPython
import time
import random
import numpy as np
from random import sample
from collections import OrderedDict
import Polygon as pn
# from random import uniform, random
from operator import itemgetter, attrgetter

class UltimateHeuristicPlanner(object):
    ### Input:
    ### (1) initial_arrangement (a list of pose_ids, each of which indicating the initial pose for an object)
    ### (2) final_arrangement (a list of pose_ids, each of which indicating the final pose for an object)
    ### instance 
    ### (i) workspace, (ii) object centers/slots, (iii) buffer centers/slots
    ### visualTool: a visualization tool as a debugging purpose

    ### Output:
    ### the whole plan
    def __init__(self, init_arr, final_arr, instance, gpd, new_paths, polygon, RAD, constraint_set, visualTool):
        print("\n")
        # time_initialization = time.clock()

        self.initial_arrangement = init_arr
        self.final_arrangement = final_arr
        self.points = instance.points + instance.buffer_points
        self.poses = instance.objects + instance.buffers
        self.numObjs = len(self.initial_arrangement)
        self.nPoses = len(self.points)
        self.allPoses = range(self.nPoses)
        self.polygon = polygon
        self.RAD = RAD
        self.constraint_set = constraint_set
        self.visualTool = visualTool

        ### this number decides how many leafs we will love to add each time (braching factor)
        self.k = max(4, int(self.numObjs / 2))
        # self.k = self.numObjs

        ### initialize dependency_dict and path_dict as empty dict
        ### since now we are going to increment these two dicts online, instead of offline
        self.dependency_dict = copy.deepcopy(gpd.dependency_dict)
        self.path_dict = copy.deepcopy(gpd.path_dict)
        self.new_paths = new_paths
        self.pose_locations = copy.deepcopy(gpd.pose_locations)
        self.region_dict = copy.deepcopy(gpd.region_dict)
        self.linked_list = copy.deepcopy(gpd.LL)
        # time_getStraightPaths = time.clock()
        self.getStraightPaths()
        # print("time for getting straight paths: " + str(time.clock() - time_getStraightPaths))

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
        ### members in a node: arrangement, node_id, cost_to_come, object_ordering, paths_option, parent_id
        self.treeL["L0"] = ArrNode(self.initial_arrangement, "L0", 0, None, None, None)
        self.treeR["R0"] = ArrNode(self.final_arrangement, "R0", 0, None, None, None)
        self.arrLeftRegistr.append(self.initial_arrangement)
        self.arrRightRegistr.append(self.final_arrangement)
        self.idLeftRegistr.append("L0")
        self.idRightRegistr.append("R0")

        ### some variables ###
        self.simplePath = [] ### a list of node_ids
        self.solution_cost  = np.inf


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
        self.queue = []
        self.node_checked = 0

        totalTime_allowed = 10*self.numObjs ### allow 500s for the total search tree construction
        start_time = time.clock()
        # print("Initialization time: " + str(time.clock() - time_initialization))


        # time_monotoneConnect = time.clock()
        ### initial connection attempt
        self.monotoneConnect(self.treeL["L0"], self.treeR["R0"])
        # print("time_monotoneConnect: " + str(time.clock() - time_monotoneConnect))

        if self.isConnected == True:
            return
        else:
            ####### conduct a BFS for the perturbation process ########
            print("\n\nstart our non-monotone journey")
            while (len(self.queue) > 0) and (self.isConnected == False) and (time.clock() - start_time < totalTime_allowed):
                # startTime_task = time.clock()
                curr_task = self.queue.pop(-1)
                # print("queue empty? " + str(bool(len(self.queue)==0)))
                self.node_checked += 1
                obj_idx = curr_task[0]
                buff_idx = curr_task[1]
                start_node_id = curr_task[2]
                # print("\ncurr_task: " + str(curr_task))
                # print("current queue size: " + str(len(self.queue)))
                # print("#nodes checked: " + str(self.node_checked))
                start_arrangement = self.treeL[start_node_id].arrangement
                goal_arrangement = self.treeR["R0"].arrangement
                # print("start_arrangement: " + str(start_arrangement))
                # print("goal_arrangement: " + str(goal_arrangement))
                n = len(start_arrangement)
                obj_buffer_dict = {}
                obj_buffer_dict[obj_idx] = (n, buff_idx)
                # print("obj_buffer_dict: " + str(obj_buffer_dict))

                start_poses = {}
                goal_poses = {}
                for i in range(len(start_arrangement)):
                    start_poses[i] = start_arrangement[i]
                for i in range(len(goal_arrangement)):
                    goal_poses[i] = goal_arrangement[i]

                nObjectToMove = len(self.computeObjectsToMove(start_arrangement, goal_arrangement))
                # print("#objects to move in this non-monotone call: " + str(nObjectToMove+1))
                # startTime_nonmonDP = time.clock()
                subTree = DFS_Rec_for_Non_Monotone_General(
                    start_poses, goal_poses, self.dependency_dict, self.path_dict, 
                    self.pose_locations, self.linked_list, self.region_dict, 
                    obj_buffer_dict)
                ### update dependency_dict and path_dict
                self.dependency_dict = subTree.dependency_dict
                self.path_dict = subTree.path_dict

                # print("Time for calling non_monotone_general: " + str(time.clock() - startTime_nonmonDP))


                # print("The problem is solved? " + str(bool(subTree.isMonotone)))
                # print("subTree.parent: " + str(subTree.parent))
                # print("subTree.leafs: " + str(subTree.leafs))

                # startTime_harvestMonotoneAfterBuffer = time.clock()
                if subTree.isMonotone == True:
                    ### the problem is solved and we get the solution
                    root_nodeID = subTree.leafs.keys()[0]
                    root_arrangement = self.decodeArrangement_withBuffer(
                        root_nodeID, start_arrangement, goal_arrangement, obj_idx, buff_idx)
                    # print("root arrangement: " + str(root_arrangement))
                    # print("ordering: " + str(subTree.object_ordering))
                    # print("subTree.path_selection_dict: " + str(subTree.path_selection_dict))
                    self.solution_cost = self.treeL[start_node_id].cost_to_come + len(subTree.object_ordering)
                    self.totalActions = self.solution_cost
                    self.isConnected = True
                    ### Let's figure out the intermediate_arrangement between start_arrangement and root_arrangement
                    order_untilBuffer, order_afterBuffer_tillEnd, \
                        pathSelection_untilBuffer, pathSelection_afterBuffer_tillEnd = self.splitOrderingAndPathSelection(
                            subTree.object_ordering, subTree.path_selection_dict, obj_idx)
                    intermediate_arrangement = self.interpolateArr(
                                            start_arrangement, goal_arrangement, order_untilBuffer, obj_idx, buff_idx)
                    # print("order_untilBuffer: " + str(order_untilBuffer))
                    # print("order_afterBuffer_tillEnd: " + str(order_afterBuffer_tillEnd))
                    # print("pathSelection_untilBuffer: " + str(pathSelection_untilBuffer))
                    # print("pathSelection_afterBuffer_tillEnd: " + str(pathSelection_afterBuffer_tillEnd))
                    # print("intermediate_arrangement: " + str(intermediate_arrangement))

                    ### Create this intermediate_arrangement node
                    self.treeL["L" + str(self.left_idx)] = ArrNode(
                            intermediate_arrangement, "L" + str(self.left_idx), \
                            self.solution_cost - len(order_afterBuffer_tillEnd), \
                            order_untilBuffer, pathSelection_untilBuffer, start_node_id)
                    self.arrLeftRegistr.append(intermediate_arrangement)
                    self.idLeftRegistr.append("L" + str(self.left_idx))
                    ### update the information for the right tree root
                    self.treeR["R0"].updateCostToCome(self.solution_cost)
                    self.treeR["R0"].updateObjectOrdering(order_afterBuffer_tillEnd)
                    self.treeR["R0"].updatePathOption(pathSelection_afterBuffer_tillEnd)
                    self.treeR["R0"].updateParent("L" + str(self.left_idx))

                    # self.visualTool.drawCurrArrangement("R0", self.points, self.poses, \
                    #         leaf_arrangement, goalNode.arrangement, "save-to-solution")
                    ## visualize the branch
                    # self.visualizeLocalBranch("L0", "L" + str(self.left_idx), "L,L")
                    # self.visualizeLocalBranch("L" + str(self.left_idx), "R0", "L,R")

                    self.left_idx += 1

                    # print("Time for harvest solution after buffer introduction: " + str(time.clock() - startTime_harvestMonotoneAfterBuffer))
                    return


                
                ### the subproblem is non-monotone even with a help of a buffer
                ### again we have to reason about the failure 
                ### we look at the dependency graph from the mutation node
                if len(subTree.mutation_nodes) == 0:
                    # print("not able to continue" + str(curr_task))
                    ### before the next iteration, check if the queue is empty
                    if (len(self.queue) == 0):
                        # print("the queue is empty and the solution is not found")
                        ### reach here since the queue is empty and the solution is not found
                        ### we need to add some perturbation on existing node and continue the process
                        ### (1) first choose a node for perturbation
                        if (len(self.treeL) < 3):
                            temp_ids = sample(range(0, len(self.treeL)), 1)
                        else:
                            temp_ids = sample(range(0, len(self.treeL)), 3)
                        mutate_ids_choice = ["L" + str(temp_id) for temp_id in temp_ids]
                        # print("mutate_ids_choice: " + str(mutate_ids_choice))
                        temp_cost = [self.treeL[mutate_id].cost_to_come for mutate_id in mutate_ids_choice]
                        # print("temp_cost: " + str(temp_cost))
                        mutate_ids_choice = [x for _,x in sorted(zip(temp_cost, mutate_ids_choice))]
                        for mutate_id in mutate_ids_choice:
                            ### random perturbation
                            obj_idx = random.choice(range(self.numObjs))
                            pose_idx = random.choice(self.allPoses)
                            triple_task = [obj_idx, pose_idx, mutate_id]
                            self.queue.insert(0, triple_task)
                        # print("current queue: " + str(self.queue))

                    continue

                # startTime_addingQueueInTask = time.clock()
                ### there exists a mutation_node
                # print("subTree.mutation_nodes: " + str(subTree.mutation_nodes))
                # print("subTree leafs during perturbation: " + str(subTree.leafs))
                leaf_nodeID = subTree.leafs.keys()[0]
                leaf_object_ordering = subTree.leafs.values()[0]
                leaf_arrangement = self.decodeArrangement_withBuffer(
                                    leaf_nodeID, start_arrangement, goal_arrangement, obj_idx, buff_idx)
                leaf_path_selection_dict = subTree.leaf_path_selection_dict[leaf_nodeID]
                # print("leaf arrangement: " + str(leaf_arrangement))
                # print("ordering: " + str(leaf_object_ordering))
                # print("leaf_path_selection_dict: " + str(leaf_path_selection_dict))
                
                ### add it to the tree
                leaf_cost_to_come = self.treeL[start_node_id].cost_to_come + len(leaf_object_ordering)
                currTreeNode_id = "L"+str(self.left_idx)
                self.treeL[currTreeNode_id] = ArrNode(
                        leaf_arrangement, currTreeNode_id, leaf_cost_to_come, \
                        leaf_object_ordering, leaf_path_selection_dict, start_node_id)
                self.arrLeftRegistr.append(leaf_arrangement)
                self.idLeftRegistr.append(currTreeNode_id)

                # self.visualizeLocalBranch(start_node_id, currTreeNode_id, "L,L")

                self.left_idx += 1
                # print("adding a node in the tree")
                # print(str(currTreeNode_id) + ": " + str(leaf_arrangement) + \
                #     " cost_to_come: " + str(leaf_cost_to_come) + " object_ordering: " + str(leaf_object_ordering) + \
                #     " path_option: " + str(leaf_path_selection_dict) + " parent: " + str(start_node_id))

                ### (2) Now we use the IP solver to get the best dependency graph or the best ordering (minimum feedback arc)
                ### for the query of leaf_arrangement -> goal_arrangement
                # time_computeObjectsToMove_nm = time.clock()
                object_to_move_leaf2goal = self.computeObjectsToMove(leaf_arrangement, goal_arrangement)
                # print("time_computeObjectsToMove_nm: " + str(time.clock() - time_computeObjectsToMove_nm))
                # time_objectDependencyOptsGenerate_nm = time.clock()
                object_dependency_opts = self.object_dependency_opts_generate(
                                                leaf_arrangement, goal_arrangement, object_to_move_leaf2goal)
                # print("time_objectDependencyOptsGenerate_nm: " + str(time.clock() - time_objectDependencyOptsGenerate_nm))
                # time_IPsolver_nm = time.clock()
                # IP_arc_buffers = feedback_arc_ILP_buffers(object_dependency_opts)
                # arc_setSize, arcs, path_selection, object_ordering, DG, dependencyEdge_paths = IP_arc_buffers.optimum
                # print("time_IPsolver_nm: " + str(time.clock() - time_IPsolver_nm))
                # print("arcs: " + str(arcs))
                # print("curr_task: " + str(curr_task))
                ### get the ranking for objects
                # time_objectRank_nm = time.clock()
                object_ranking = self.rankObjectsFromDependency(object_dependency_opts, object_to_move_leaf2goal)
                # print("object_ranking: " + str(object_ranking))

                # object_ranking = self.rankObjects(DG)
                # print("time_objectRank_nm: " + str(time.clock() - time_objectRank_nm))
                ### assign a buffer to each object (so far assign the farthest buffer per object)
                time_assignBuffers_nm = time.clock()
                # objects2buffers = self.assignBuffers(object_ranking, leaf_arrangement, goal_arrangement)
                objects2buffers = self.assignBuffers_constraintSet(object_ranking, leaf_arrangement, goal_arrangement)
                # print("time_assignBuffers_nm: " + str(time.clock() - time_assignBuffers_nm))
                for hh in range(len(objects2buffers)):
                    objects2buffers[hh].append(currTreeNode_id)
                # print("objects2buffers: " + str(objects2buffers))

                ### (3) add these new tasks to the queue
                for triple_task in objects2buffers:
                    # print("triple_task through nm: " + str(triple_task))
                    self.queue.insert(0, triple_task) ### update the queue

                # print("Time for adding tasks to queue during non-monotone query: " + str(time.clock() - startTime_addingQueueInTask))
                # print("Time for a task: " + str(time.clock() - startTime_task) + "\n")
                    
                    
                ### before the next iteration, check if the queue is empty
                if (len(self.queue) == 0):
                    # print("the queue is empty and the solution is not found")
                    ### reach here since the queue is empty and the solution is not found
                    ### we need to add some perturbation on existing node and continue the process
                    ### (1) first choose a node for perturbation
                    if (len(self.treeL) < 3):
                        temp_ids = sample(range(0, len(self.treeL)), 1)
                    else:
                        temp_ids = sample(range(0, len(self.treeL)), 3)
                    mutate_ids_choice = ["L" + str(temp_id) for temp_id in temp_ids]
                    # print("mutate_ids_choice: " + str(mutate_ids_choice))
                    temp_cost = [self.treeL[mutate_id].cost_to_come for mutate_id in mutate_ids_choice]
                    # print("temp_cost: " + str(temp_cost))
                    mutate_ids_choice = [x for _,x in sorted(zip(temp_cost, mutate_ids_choice))]
                    for mutate_id in mutate_ids_choice:
                        ### random perturbation
                        obj_idx = random.choice(range(self.numObjs))
                        pose_idx = random.choice(self.allPoses)
                        triple_task = [obj_idx, pose_idx, mutate_id]
                        self.queue.insert(0, triple_task)
                    # print("current queue: " + str(self.queue))

               


    def monotoneConnect(self, initNode, goalNode):
        ### construct start_poses and goal_poses
        start_poses = {}
        goal_poses = {}
        for obj_idx in range(len(initNode.arrangement)):
            start_poses[obj_idx] = initNode.arrangement[obj_idx]
        for obj_idx in range(len(goalNode.arrangement)):
            goal_poses[obj_idx] = goalNode.arrangement[obj_idx]

        time_theOnlyMonotoneCall = time.clock()
        subTree = DFS_Rec_for_Monotone_General(
            start_poses, goal_poses, self.dependency_dict, self.path_dict, \
            self.pose_locations, self.linked_list, self.region_dict)
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        print("time_theOnlyMonotoneCall: " + str(time.clock() - time_theOnlyMonotoneCall))
        print("Leafs: " + str(subTree.leafs))

        # print("The problem is initially montone? " + str(bool(subTree.isMonotone)))
        # print("subTree.parent: " + str(subTree.parent))
        # print("subTree.leafs: " + str(subTree.leafs))


        if subTree.isMonotone == True:
            ### the problem is solved and we get the solution
            root_nodeID = subTree.leafs.keys()[0]
            root_arrangement = self.decodeArrangement(root_nodeID, initNode.arrangement, goalNode.arrangement)
            # print("root arrangement: " + str(root_arrangement))
            # print("ordering: " + str(subTree.object_ordering))
            # print("subTree.path_selection_dict: " + str(subTree.path_selection_dict))
            self.solution_cost = initNode.cost_to_come + len(subTree.object_ordering)
            self.totalActions = self.solution_cost
            self.isConnected = True
            ### update the information for the right tree root
            self.treeR["R0"].updateCostToCome(self.solution_cost)
            self.treeR["R0"].updateObjectOrdering(subTree.object_ordering)
            self.treeR["R0"].updatePathOption(subTree.path_selection_dict)
            self.treeR["R0"].updateParent("L0")
            # self.visualTool.drawCurrArrangement("R0", self.points, self.poses, \
            #         root_arrangement, goalNode.arrangement, "save-to-solution")
            ### visualize the branch
            # self.visualizeLocalBranch("L0", "R0", "L,R")
            return None

        ### the subproblem is non-monotone
        ### we have to reason about the failure
        ### we look at the dependency graph from the leafs
        ### Check each leaf node
        for leaf_nodeID, leaf_object_ordering in subTree.leafs.items():
            ### we first decode the arrangement out of the leaf_node
            # print("current leafNode checking: ", leaf_nodeID, leaf_object_ordering)
            leaf_arrangement = self.decodeArrangement(leaf_nodeID, initNode.arrangement, goalNode.arrangement)
            path_selection_dict = subTree.leaf_path_selection_dict[leaf_nodeID]
            # print("leaf arrangement: " + str(leaf_arrangement))
            # print("ordering: " + str(leaf_object_ordering))
            # print("path_selection_dict: " + str(path_selection_dict))

            ### Then we use the IP solver to get the best dependency graph or the best ordering (minimum feedback arc)
            # time_computeObjectsToMove = time.clock()
            object_to_move_leaf2goal = self.computeObjectsToMove(leaf_arrangement, goalNode.arrangement)
            # print("time_computeObjectsToMove: " + str(time.clock() - time_computeObjectsToMove))
            time_objectDependencyOptsGenerate = time.clock()
            object_dependency_opts = self.object_dependency_opts_generate(
                                            leaf_arrangement, goalNode.arrangement, object_to_move_leaf2goal)
            time_objectRank = time.clock()
            object_ranking = self.rankObjectsFromDependency(object_dependency_opts, object_to_move_leaf2goal)
            print("time_objectRank: " + str(time.clock() - time_objectRank))

            # print("time_objectDependencyOptsGenerate: " + str(time.clock() - time_objectDependencyOptsGenerate))

            # time_IPsolver = time.clock()
            # IP_arc_buffers = feedback_arc_ILP_buffers(object_dependency_opts)
            # arcsrc_setSize, arcs, path_selection, object_ordering, DG, dependencyEdge_paths = IP_arc_buffers.optimum
            # print("time_IPsolver: " + str(time.clock() - time_IPsolver))
            # print("-----leaf arrangement----: " + str(leaf_arrangement))
            # print("arc_setSize: " + str(arc_setSize))
            # print("arcs: " + str(arcs))
            # print("path_selection: " + str(path_selection))
            # print("object_ordering: " + str(object_ordering))
            # print("DG: ")
            # print(DG)
            # print("-------------------------------------------")
            # print("dependencyEdge_paths: ")
            # for constr_edge, path_info in dependencyEdge_paths.items():
            #     print(str(constr_edge) + ": " + str(path_info))

            ### get the ranking for objects
            # time_objectRank = time.clock()
            # object_ranking = self.rankObjects(DG)

            # print("time_objectRank: " + str(time.clock() - time_objectRank))
            ### assign a buffer to each object (so far assign the farthest buffer per object)
            time_assignBuffers = time.clock()
            # objects2buffers = self.assignBuffers(object_ranking, leaf_arrangement, goalNode.arrangement)
            objects2buffers = self.assignBuffers_constraintSet(object_ranking, leaf_arrangement, goalNode.arrangement)
            print("time_assignBuffers: " + str(time.clock() - time_assignBuffers))

            for hh in range(len(objects2buffers)):
                objects2buffers[hh].append(initNode.node_id)
            # print("objects2buffers: " + str(objects2buffers))

            ## draw arrangement to see what happens
            # self.visualTool.drawCurrArrangement(str(leaf_nodeID), self.points, self.poses, \
            #                 leaf_arrangement, goalNode.arrangement, "save-to-tree")

            for triple_task in objects2buffers:
                # print("triple_task: " + str(triple_task))
                self.queue.insert(0, triple_task) ### initialization of the queue



    def assignBuffers(self, object_ranking, curr_arrangement, goal_arrangement):
        ### Input: object_ranking (a list of obj_idx)
        ###        curr_arrangement (a list of pose_idx)
        ### Output: objects2buffers (a dict <key: obj_idx, value: pose_idx as buffer)

        ### we currently follow the idea of selecting the fartherst buffer for each object
        objects2buffers = []
        for obj_idx in object_ranking:
            curr_obj_pose = curr_arrangement[obj_idx]
            goal_obj_pose = goal_arrangement[obj_idx]
            farthest_reachable_buffer_pose = self.findBufferForPose(
                    curr_obj_pose, curr_arrangement, goal_obj_pose, goal_arrangement)
            objects2buffers.append([obj_idx, farthest_reachable_buffer_pose])

        # print("objects2buffers: " + str(objects2buffers))

        objects2buffers_to_eliminated = []
        for [obj, buff] in objects2buffers:
            if buff == -1:
                objects2buffers_to_eliminated.append([obj, buff])

        for [obj, buff] in objects2buffers_to_eliminated:
            objects2buffers.remove([obj, buff])


        return objects2buffers


    def assignBuffers_constraintSet(self, object_ranking, curr_arrangement, goal_arrangement):
        ### This function assigns buffers for each object in the object_ranking
        ### based on the available poses in the constraint set given the curr_arrangment
        available_poses = copy.deepcopy(self.allPoses)
        collision_poses = set()
        for curr_pose in curr_arrangement:
            collision_poses = collision_poses.union(self.constraint_set[curr_pose])
            collision_poses = collision_poses.union([curr_pose])

        for pose in list(collision_poses):
            available_poses.remove(pose)

        objects2buffers = [] ### a list of [obj_idx, buff_idx]



        ### (1) given priority to the extra buffers we generated
        for obj in object_ranking:
            for buff in available_poses:
                if int(buff / 2) not in range(self.numObjs):
                    ### only extra buffers
                    objects2buffers.append([obj, buff])

        ### (2) then consider the poses of objects which has been moved at the current arrangement
        for obj in object_ranking:
            for buff in available_poses:
                if int(buff / 2) in range(self.numObjs):
                    ### they are object poses
                    if (buff % 2 == 0):
                        ### they are start poses
                        obj_idx = int(buff / 2)
                        if (curr_arrangement[obj_idx] == goal_arrangement[obj_idx]):
                            ### the objects have already been moved on the current arrangement
                            objects2buffers.append([obj, buff])

        ### (3) then consider poses which are not in available poses but can reach for certain object
        for obj in object_ranking:
            overlap_poses = self.constraint_set[curr_arrangement[obj]]
            for buff in overlap_poses:
                objects2buffers.append([obj, buff])

        # ### (4) finally consider the poses of objects which has yet to move at the current arrangement
        # for obj in object_ranking:
        #     for buff in available_poses:
        #         if int(buff / 2) in range(self.numObjs):
        #             ### they are object poses
        #             if (buff % 2 == 1):
        #                 ### they are goal poses
        #                 obj_idx = int(buff / 2)
        #                 if (curr_arrangement[obj_idx] != goal_arrangement[obj_idx]):
        #                     ### the objects have not yet moved to the goal on the current arrangement
        #                     objects2buffers.append([obj, buff])

        return objects2buffers




    def findBufferForPose(self, curr_obj_pose, curr_arrangement, goal_obj_pose, goal_arrangement):
        occupied_poses = []
        ### everythin pose on curr_arrangement is considered occupied except the curr_obj_pose
        for i in range(len(curr_arrangement)):
            if curr_arrangement[i] != curr_obj_pose:
                occupied_poses.append(curr_arrangement[i])
        ### construct available region
        Available_Regions = []
        for region in self.region_dict.keys():
            OCCUPIED = False
            for pose in region:
                if pose in occupied_poses:
                    OCCUPIED = True
                    break
            if not OCCUPIED:
                Available_Regions.append(self.region_dict[region])

        ### Search for the farthest pose using BFS
        farthest_reachable_buffer_pose = -1
        explored = {}
        for key in self.region_dict.values():
            explored[key] = False
        queue = [self.region_dict[self.pose_locations[curr_obj_pose]]]
        explored[self.region_dict[self.pose_locations[curr_obj_pose]]] = True
        while len(queue) > 0:
            ### queue(0) for BFS
            old_node = queue.pop(-1)
            ### first check if this old_node region corresponds to a pose ###
            regions = list(self.region_dict.keys())
            region_ids = list(self.region_dict.values())
            old_node_region = regions[region_ids.index(old_node)]
            if old_node_region in self.pose_locations.values():
                ### this old_node region corresponds to a pose
                pose_ids = list(self.pose_locations.keys())
                pose_regions = list(self.pose_locations.values())
                that_pose = pose_ids[pose_regions.index(old_node_region)]
                if (that_pose != curr_obj_pose) and (that_pose != goal_obj_pose):
                    farthest_reachable_buffer_pose = that_pose
            ############################################################################

            if old_node in self.linked_list:
                for region in self.linked_list[old_node]:
                    if explored[region]:
                        continue
                    if region not in Available_Regions:
                        continue
                    ### you are reaching here since it is an available and unexplored region
                    queue.append(region)
                    explored[region] = True

        ### You are reaching here since the BFS search is finished
        return farthest_reachable_buffer_pose


    def rankObjects(self, DG):
        ### get the objects inner degree and outer degree from DG
        objects_degree_dict = [] ### (obj_idx, inner_degree, outer_degree)
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
        for obj_info in objects_degree_dict[0:self.k+1]:
            object_ranking.append(obj_info[0])

        # print("object_ranking: " + str(object_ranking))

        return object_ranking


    def rankObjectsFromDependency(self, object_dependency_opts, object_to_move):
        ### This function tries to rank objects (priority to leave the current pose) given object dependency at given arrangement
        ### Input: object_depenedency_opts (key: obj_idx, value: object_path_options)
        ### Output: object_ranking: a list of obj_idx (with priority order)
        numObjs = len(object_dependency_opts)
        ### each item: [obj_idx, start_pose_degree, goal_pose_degree, indicator(0: no move, 1: move)]
        objects_degree_dict = [[0, 0, 0, obj_idx] for obj_idx in range(numObjs)]
        for obj_idx in object_to_move:
            objects_degree_dict[obj_idx][0] = 1

        for i in range(numObjs):
            for path_index in range(len(object_dependency_opts[i])):
                for constr in object_dependency_opts[i][path_index]:
                    if constr[1] == 0:
                        objects_degree_dict[constr[0]][1] += 1
                    else:
                        ### constr[1] = 1
                        objects_degree_dict[constr[0]][2] += 1

        # print("objects_degree_dict before ranking: ")
        # for item in objects_degree_dict:
        #     print(item)

        ### Given the objects_degree_dict, let's rank the object
        ### (0) the objects not moved at the current arrangement
        ### (1) start pose degree of interference 
        ### (2) goal pose degree of interference

        ### we prioritze the unmoved objects
        # objects_degree_dict = sorted(objects_degree_dict, key=itemgetter(3), reverse=True) 
        # ### we prioritize the objects whose current poses is a constraint
        # objects_degree_dict = sorted(objects_degree_dict, key=itemgetter(1), reverse=True) 
        # ### we prioritize the objects whose goal poses is a constraint
        # objects_degree_dict = sorted(objects_degree_dict, key=itemgetter(2), reverse=True)
        objects_degree_dict = sorted(objects_degree_dict, reverse=True)

        # print("objects_degree_dict after ranking: ")
        # for item in objects_degree_dict:
        #     print(item)

        object_ranking = []
        for obj_info in objects_degree_dict[0:self.k+1]:
            object_ranking.append(obj_info[3])

        # print("object_ranking: " + str(object_ranking))

        return object_ranking


    def object_dependency_opts_generate(self, query_arrangement, goal_arrangement, object_to_move):
        num_objs = len(query_arrangement)
        object_dependency_opts = {}
        for obj_idx in range(num_objs):
            if obj_idx not in object_dependency_opts.keys():
                object_dependency_opts[obj_idx] = []
            ### look into the path option for the current object's current pose and destination pose
            pose_key1 = min(query_arrangement[obj_idx], goal_arrangement[obj_idx])
            pose_key2 = max(query_arrangement[obj_idx], goal_arrangement[obj_idx])
            for path in self.dependency_dict[(pose_key1, pose_key2)]:
                path_set = set()
                for constr in path:
                    ### check the constraint based on query_arrangement and goal_arrangement
                    ### if the constraint is caused by current pose of an object
                    if (constr in query_arrangement) and (query_arrangement.index(constr) != obj_idx):
                        path_set = path_set.union({(query_arrangement.index(constr), 0)})
                    ### if the constraint is caused by final pose of an object
                    if (constr in goal_arrangement) and (goal_arrangement.index(constr) != obj_idx):
                        path_set = path_set.union({(goal_arrangement.index(constr), 1)})
                    ### Otherwise, it is a buffer and do nothing
                object_dependency_opts[obj_idx].append(path_set)

        return object_dependency_opts


    def decodeArrangement(self, node_id, init_arrangement, goal_arrangement):
        ### This function, based on number of objects in the current problem
        ### convert the node_id (int) into arrangement (a list of pose idx)
        new_arrangement = []
        for i in range(self.numObjs):
            isThatObjectInGoal = checkBitStatusAtPos(node_id, i)
            if isThatObjectInGoal:
                ### add the goal pose index
                new_arrangement.append(goal_arrangement[i])
            else:
                ### add the initial pose index
                new_arrangement.append(init_arrangement[i])

        return new_arrangement


    def decodeArrangement_withBuffer(self, node_id, init_arrangement, goal_arrangement, obj_idx, buff_idx):
        new_arrangement = []
        for i in range(self.numObjs):
            isThatObjectInGoal = checkBitStatusAtPos(node_id, i)
            if isThatObjectInGoal:
                if i == obj_idx:
                    new_arrangement.append(buff_idx)
                else:
                    ### add the goal pose index
                    new_arrangement.append(goal_arrangement[i])
            else:
                ### add the initial pose index
                new_arrangement.append(init_arrangement[i])

        ### look at the additional bit
        i = self.numObjs
        isThatObjectInGoal = checkBitStatusAtPos(node_id, i)
        if isThatObjectInGoal:
            new_arrangement[obj_idx] = goal_arrangement[obj_idx]

        return new_arrangement


    def identifyObject2buffer(self, arcs, DG):
        ### Input: arcs that are inevitably violated
        ###        DG: dependency graph
        ### Output: the object to move to buffer (obj_idx)

        ### we first use arcs to find all object candidates
        object_candidates = set()
        for arc in arcs:
            obj1 = arc[0]
            obj2 = arc[1]
            object_candidates = object_candidates.union({obj1})
            object_candidates = object_candidates.union({obj2})

        ### Then we use inner degree to take out those most constraining objects (largest inner degree)
        largest_inner_degree = -1
        for obj_idx in object_candidates:
            curr_inner_degree = sum(DG[:, obj_idx])
            if curr_inner_degree > largest_inner_degree:
                largest_inner_degree = curr_inner_degree
                most_constraining_objects = []
                most_constraining_objects.append(obj_idx)
            elif curr_inner_degree == largest_inner_degree:
                most_constraining_objects.append(obj_idx)

        ### if there are more than one most constraining objects, 
        ### pick the one with the least constrained one (smallest outer degree)
        if len(most_constraining_objects) != 1:
            smallest_outer_degree = np.inf
            for obj_idx in most_constraining_objects:
                curr_outer_degree = sum(DG[obj_idx, :])
                if curr_outer_degree < smallest_outer_degree:
                    smallest_outer_degree = curr_outer_degree
                    least_constrained_objects = []
                    least_constrained_objects.append(obj_idx)
                elif curr_outer_degree == smallest_outer_degree:
                    least_constrained_objects.append(obj_idx)
            return least_constrained_objects[0]
        else:
            return most_constraining_objects[0]


    def computeObjectsToMove(self, init_arrangement, final_arrangement):
        ### input: init_arrangement (a list of pose_idx)
        ###        final arrangement (a list of pose_idx)
        ### Output: objects_to_move (a list of obj_idx)
        objects_to_move = []
        for obj_idx in range(len(init_arrangement)):
            if init_arrangement[obj_idx] != final_arrangement[obj_idx]:
                objects_to_move.append(obj_idx)

        return objects_to_move


    def splitOrderingAndPathSelection(self, object_ordering, path_selection_dict, obj_idx):
        ### This function splits the object_ordering and path_selection_dict into two pieces 
        ### using obj_idx as the splitting factor
        order_untilBuffer = []
        order_afterBuffer_tillEnd = []
        pathSelection_untilBuffer = {}
        pathSelection_afterBuffer_tillEnd = {}

        for kk in range(len(object_ordering)):
            object_index = object_ordering[kk]
            order_untilBuffer.append(object_index)
            pathSelection_untilBuffer[object_index] = path_selection_dict[object_index]
            ### check if object_index the the object to put to the buffer
            if object_index == obj_idx:
                break
        for mm in range(kk+1, len(object_ordering)):
            object_index = object_ordering[mm]
            order_afterBuffer_tillEnd.append(object_index)
            if object_index == obj_idx: 
                pathSelection_afterBuffer_tillEnd[object_index] = path_selection_dict[self.numObjs]
            else:
                pathSelection_afterBuffer_tillEnd[object_index] = path_selection_dict[object_index]

        return order_untilBuffer, order_afterBuffer_tillEnd, pathSelection_untilBuffer, pathSelection_afterBuffer_tillEnd


    def interpolateArr(self, start_arrangement, goal_arrangement, order_untilBuffer, obj_idx, buff_idx):
        ### This function figures out an intermediate arrangement between start_arrangement and goal_arrangement
        ### by referring to order_untilBuffer, obj_idx and buff_idx
        intermediate_arrangement = copy.deepcopy(start_arrangement)
        for object_index in order_untilBuffer:
            if object_index == obj_idx:
                intermediate_arrangement[object_index] = buff_idx
            else:
                intermediate_arrangement[object_index] = goal_arrangement[object_index]

        return intermediate_arrangement


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


    def visualizeLocalBranch(self, parent_id, child_id, connectMode):
        ### Two connect modes
        ### (1) "L,L"
        ### (2) "L,R"
        if connectMode == "L,L":
            parentNode = self.treeL[parent_id]
            childNode = self.treeL[child_id]
            temp_ppaths = self.getPaths(parentNode, childNode, "Left")
        else:
            ### "L,R"
            parentNode = self.treeL[parent_id]
            childNode = self.treeR[child_id]
            temp_ppaths = self.getPaths(parentNode, childNode, "Bridge")

        self.visualTool.drawLocalMotions((parent_id, child_id), temp_ppaths,
            self.points, self.poses, parentNode.arrangement, self.final_arrangement, "save-to-tree")


    def getPaths(self, parentNode, childNode, treeMode):
        ### Input: 1. parentNode -> childNode, indicating the direction of tree node
        ###        2. treeMode: Left, or Bridge
        ### Output: point paths for all involved objects {obj_idx: [(x,x), (x,x), ... , (x, x)]}

        result_path = OrderedDict()
        
        ### loop through the objects moved based on object_ordering
        for obj_idx in childNode.object_ordering:
            ### get the key (pose1, pose2)
            start_key = parentNode.arrangement[obj_idx]
            goal_key = childNode.arrangement[obj_idx]
            path_option_curr_obj = childNode.path_option[obj_idx]


            if path_option_curr_obj == 0:
                ### it is a straight path
                start_pt = self.points[start_key]
                goal_pt = self.points[goal_key]
                nsteps = 3
                result_path[obj_idx] = []
                for step in range(nsteps+1):
                    pt_x = start_pt[0] + (goal_pt[0] - start_pt[0]) / nsteps * step
                    pt_y = start_pt[1] + (goal_pt[1] - start_pt[1]) / nsteps * step
                    result_path[obj_idx].append((pt_x, pt_y))
            else:
                ### it is a real region path (rpath)
                key = (min(start_key, goal_key), max(start_key, goal_key))
                rpath = copy.deepcopy(self.path_dict[key][path_option_curr_obj])
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

                result_path[obj_idx] = ppath

        ### reach here since we have done all objects
        return result_path


    def getSolutionStats(self):
        # print("Let's get stats!!")
        ### This function is used to get the statistics if the path is found
        self.numNodesInLeftTree = len(self.treeL)
        self.numNodesInRightTree = len(self.treeR)
        self.numLeftBranches = self.numNodesInLeftTree - 1
        self.numRightBranches = self.numNodesInRightTree - 1        
        self.simplePath = []
        ### from R0, back track to left root via parent search
        self.simplePath.insert(0, "R0")
        curr_waypoint_id = self.treeR["R0"].parent_id
        while curr_waypoint_id != "L0":
            self.simplePath.insert(0, curr_waypoint_id)
            curr_waypoint_id = self.treeL[curr_waypoint_id].parent_id
        self.simplePath.insert(0, curr_waypoint_id)
        print("path: " + str(self.simplePath))
        print("total action: " + str(self.totalActions))

        ### add ordering here
        ### add non-monotone actions here
        self.final_order = [] ### a list of obj_idx
        self.all_transitions = [] ### value: [obj_idx, [from_pose_idx, to_pose_idx]]
        for kk in range(1, len(self.simplePath)):
            pointNode_id = self.simplePath[kk]
            prev_pointNode_id = self.simplePath[kk-1]
            if pointNode_id[0] == "L" and prev_pointNode_id[0] == "L":
                curr_arrangement = self.treeL[pointNode_id].arrangement
                prev_arrangement = self.treeL[prev_pointNode_id].arrangement
                temp_object_ordering = self.treeL[pointNode_id].object_ordering
            elif pointNode_id[0] == "R" and prev_pointNode_id[0] == "L":
                curr_arrangement = self.treeR[pointNode_id].arrangement
                prev_arrangement = self.treeL[prev_pointNode_id].arrangement
                temp_object_ordering = self.treeR[pointNode_id].object_ordering                
            for obj_idx in temp_object_ordering:
                self.final_order.append(obj_idx)
                self.all_transitions.append([obj_idx, [prev_arrangement[obj_idx], curr_arrangement[obj_idx]]])

        print("object_ordering: " + str(self.final_order))
        # for [obj_idx, obj_transition] in self.all_transitions:
        #     print(str(obj_idx) + ": " + str(obj_transition))


    def constructWholePath(self):
        curr_waypoint_id = "R0"
        childNode = self.treeR[curr_waypoint_id]
        parent_waypoint_id = childNode.parent_id
        parentNode = self.treeL[parent_waypoint_id]
        
        temp_ppaths = self.getPaths(parentNode, childNode, "Bridge")
        self.whole_path.insert(0, [(parent_waypoint_id, curr_waypoint_id), temp_ppaths])
        curr_waypoint_id = self.treeR[curr_waypoint_id].parent_id

        while curr_waypoint_id != "L0":
            childNode = self.treeL[curr_waypoint_id]
            parent_waypoint_id = self.treeL[parent_waypoint_id].parent_id
            parentNode = self.treeL[parent_waypoint_id]
            temp_ppaths = self.getPaths(parentNode, childNode, "Left")
            self.whole_path.insert(0, [(parent_waypoint_id, curr_waypoint_id), temp_ppaths])
            curr_waypoint_id = self.treeL[curr_waypoint_id].parent_id




class ArrNode(object):
    def __init__(self, arrangement, node_id, cost_to_come, object_ordering, path_option, parent_id):
        self.arrangement = arrangement
        self.node_id = node_id
        self.cost_to_come = cost_to_come
        self.object_ordering = object_ordering ### a ordered list of object indices
        self.path_option = path_option ### a dict {obj_idx: path_idx}
        self.parent_id = parent_id


    def updateCostToCome(self, cost_to_come):
        self.cost_to_come = cost_to_come

    def updateObjectOrdering(self, object_ordering):
        self.object_ordering = object_ordering

    def updatePathOption(self, path_option):
        self.path_option = path_option

    def updateParent(self, parent_id):
        self.parent_id = parent_id



