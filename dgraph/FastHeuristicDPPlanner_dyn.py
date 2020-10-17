from __future__ import division

import sys
import copy
import time
import numpy as np
from itertools import product
from collections import OrderedDict
from operator import itemgetter, attrgetter

from util import *
from dspace import Circle, genBuffers
from DG_Space import linked_list_conversion
from ILPSolver import feedback_arc_ILP_buffers
from DensePathGenerator import DensePathGenerator
from DPLocalSolver import DFS_Rec_for_Monotone_General, DFS_Rec_for_Non_Monotone_General


class FastHeuristicDPPlanner(object):
    ### Input:
    ### (1) initial_arrangement (a list of pose_ids, each of which indicating the initial pose for an object)
    ### (2) final_arrangement (a list of pose_ids, each of which indicating the final pose for an object)
    ### instance
    ### (i) workspace, (ii) object centers/slots, (iii) buffer centers/slots
    ### visualTool: a visualization tool as a debugging purpose

    ### Output:
    ### the whole plan
    def __init__(self, init_arr, final_arr, space):
        self.space = space
        self.initial_arrangement = init_arr
        self.final_arrangement = final_arr
        self.numObjs = len(self.initial_arrangement)
        self.numBuffers = max(len(filter(lambda x: x[0] == 'B', self.space.poseMap.keys())), 1)
        print("Number of Buffers: ", self.numBuffers)

        ### this number decides how many leafs we will love to add each time (braching factor)
        self.k = min(1, int(self.numObjs / 2))

        # gpd = DensePathGenerator(self.space.RGAdj, self.space.pose2reg)
        # self.new_paths = {}
        # for r1, r2 in space.RG[1]:
        #     self.new_paths[(self.gpd.region_dict[r1], self.gpd.region_dict[r2])
        #                    ] = copy.deepcopy(self.regionGraph.paths[(r1, r2)])

        ### initialize dependency_dict and path_dict as empty dict
        ### since now we are going to increment these two dicts online, instead of offline
        # self.dependency_dict = copy.deepcopy(gpd.dependency_dict)
        # self.path_dict = copy.deepcopy(gpd.path_dict)
        # self.new_paths = new_paths
        # self.pose_locations = copy.deepcopy(gpd.pose_locations)
        # self.region_dict = copy.deepcopy(gpd.region_dict)
        # self.linked_list = copy.deepcopy(gpd.LL)

        self.dependency_dict = {}
        self.path_dict = {}
        self.region_dict, self.linked_list = linked_list_conversion(self.space.RGAdj)
        self.pose_locations = copy.deepcopy(self.space.pose2reg)
        self.getStraightPaths()
        # print(self.dependency_dict)
        # print(self.path_dict)

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
        self.simplePath = []  ### a list of node_ids
        self.solution_cost = np.inf

        ################## results ################
        self.isConnected = False
        self.best_solution_cost = np.inf
        ### the whole_path is a list of items and each item has the following format
        ### [("node1_id", node2_id), {2:path2, 1:path1, ...}]
        self.whole_path = []
        self.totalActions = 0  ### record the total number of actions
        self.numLeftBranches = 0  ### record the number of left branches in the solution
        self.numRightBranches = 0  ### record the number of right branches in the solution
        self.numNodesInLeftTree = 0  ### record the total number of nodes in the left tree
        self.numNodesInRightTree = 0  ### record the total number of nodes in the right tree

        ### start ruuning
        self.left_idx = 1
        self.right_idx = 1
        self.queue = []
        self.node_checked = 0

        print(self.space.poseMap.keys())
        print(self.space.regions.keys())
        self.totalTime_allowed = 30 * self.numObjs  ### allow 30s per object for the total search
        # restartTime = 5 * self.numObjs  ### allow 5s per object for the search before restarting
        start_time = time.clock()

        ### initial connection attempt
        self.monotoneConnect(self.treeL["L0"], self.treeR["R0"])

        if self.isConnected == True:
            return
        else:
            ####### conduct a BFS for the perturbation process ########
            print("\n\nstart our non-monotone journey")
            while (len(self.queue) > 0) and (self.isConnected
                                             == False) and (time.clock() - start_time < self.totalTime_allowed):
                startTime_task = time.clock()
                curr_task = self.queue.pop(-1)
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
                print("obj_buffer_dict: " + str(obj_buffer_dict))

                start_poses = {}
                goal_poses = {}
                for i in range(len(start_arrangement)):
                    start_poses[i] = start_arrangement[i]
                for i in range(len(goal_arrangement)):
                    goal_poses[i] = goal_arrangement[i]

                all_poses = set(start_arrangement + goal_arrangement + [buff_idx])
                self.space.regionGraph(lambda x: x[0] in all_poses)
                self.dependency_dict = {}
                self.path_dict = {}
                self.region_dict, self.linked_list = linked_list_conversion(self.space.RGAdj)
                self.pose_locations = copy.deepcopy(self.space.pose2reg)
                self.getStraightPaths()

                # startTime_nonmonDP = time.clock()
                subTree = DFS_Rec_for_Non_Monotone_General(
                    start_poses,
                    goal_poses,
                    self.dependency_dict,
                    self.path_dict,
                    self.pose_locations,
                    self.linked_list,
                    self.region_dict,
                    obj_buffer_dict,
                )
                # print("Time for calling non_monotone_general: " + str(time.clock() - startTime_nonmonDP))
                ### update dependency_dict and path_dict
                self.dependency_dict = subTree.dependency_dict
                self.path_dict = subTree.path_dict

                # print("The problem is solved? " + str(bool(subTree.isMonotone)))
                # print("subTree.parent: " + str(subTree.parent))
                # print("subTree.leafs: " + str(subTree.leafs))

                # startTime_harvestMonotoneAfterBuffer = time.clock()
                if subTree.isMonotone == True:
                    ### the problem is solved and we get the solution
                    root_nodeID = subTree.leafs.keys()[0]
                    root_arrangement = self.decodeArrangement_withBuffer(
                        root_nodeID, start_arrangement, goal_arrangement, obj_idx, buff_idx
                    )
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
                        start_arrangement, goal_arrangement, order_untilBuffer, obj_idx, buff_idx
                    )
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

                    self.left_idx += 1
                    # self.visualTool.drawCurrArrangement("R0", self.points, self.poses, \
                    # leaf_arrangement, goalNode.arrangement, "save-to-solution")
                    ### visualize the branch
                    # self.visualizeLocalBranch("L0", "R0", "L,R")
                    # print("Time for harvest solution after buffer introduction: " + str(time.clock() - startTime_harvestMonotoneAfterBuffer))
                    return

                ### the subproblem is non-monotone even with a help of a buffer
                ### again we have to reason about the failure
                ### we look at the dependency graph from the mutation node
                if len(subTree.mutation_nodes) == 0:
                    continue

                # startTime_addingQueueInTask = time.clock()
                ### there exists a mutation_node
                # print("subTree.mutation_nodes: " + str(subTree.mutation_nodes))
                leaf_nodeID = subTree.leafs.keys()[0]
                leaf_object_ordering = subTree.leafs.values()[0]
                leaf_arrangement = self.decodeArrangement_withBuffer(
                    leaf_nodeID, start_arrangement, goal_arrangement, obj_idx, buff_idx
                )
                leaf_path_selection_dict = subTree.leaf_path_selection_dict[leaf_nodeID]
                # print("leaf arrangement: " + str(leaf_arrangement))
                # print("ordering: " + str(leaf_object_ordering))
                # print("leaf_path_selection_dict: " + str(leaf_path_selection_dict))

                ### add it to the tree
                leaf_cost_to_come = self.treeL[start_node_id].cost_to_come + len(leaf_object_ordering)
                currTreeNode_id = "L" + str(self.left_idx)
                self.treeL[currTreeNode_id] = ArrNode(
                        leaf_arrangement, currTreeNode_id, leaf_cost_to_come, \
                        leaf_object_ordering, leaf_path_selection_dict, start_node_id)
                self.arrLeftRegistr.append(leaf_arrangement)
                self.idLeftRegistr.append(currTreeNode_id)
                self.left_idx += 1
                # print("adding a node in the tree")
                # print(str(currTreeNode_id) + ": " + str(leaf_arrangement) + \
                #     " cost_to_come: " + str(leaf_cost_to_come) + " object_ordering: " + str(leaf_object_ordering) + \
                #     " path_option: " + str(leaf_path_selection_dict) + " parent: " + str(start_node_id))

                ### (2) Now we use the IP solver to get the best dependency graph or the best ordering (minimum feedback arc)
                ### for the query of leaf_arrangement -> goal_arrangement
                object_to_move_leaf2goal = self.computeObjectsToMove(leaf_arrangement, goal_arrangement)
                object_dependency_opts = self.object_dependency_opts_generate(
                    leaf_arrangement, goal_arrangement, object_to_move_leaf2goal
                )

                IP_arc_buffers = feedback_arc_ILP_buffers(object_dependency_opts)
                arc_setSize, arcs, path_selection, object_ordering, DG, dependencyEdge_paths = IP_arc_buffers.optimum
                # print("arcs: " + str(arcs))

                ### get the ranking for objects
                object_ranking = self.rankObjects(DG)
                ### assign a buffer to each object (so far assign the farthest buffer per object)
                objects2buffers = self.assignBuffers(object_ranking, leaf_arrangement, goal_arrangement)
                for hh in range(len(objects2buffers)):
                    objects2buffers[hh].append(currTreeNode_id)
                # print("objects2buffers: " + str(objects2buffers))

                ### (3) add these new tasks to the queue
                for triple_task in objects2buffers:
                    self.queue.insert(0, triple_task)  ### update the queue

                # print("Time for adding tasks to queue during non-monotone query: " + str(time.clock() - startTime_addingQueueInTask))
                # print("Time for a task: " + str(time.clock() - startTime_task))

    def monotoneConnect(self, initNode, goalNode):
        ### construct start_poses and goal_poses
        start_poses = {}
        goal_poses = {}
        for obj_idx in range(len(initNode.arrangement)):
            start_poses[obj_idx] = initNode.arrangement[obj_idx]
        for obj_idx in range(len(goalNode.arrangement)):
            goal_poses[obj_idx] = goalNode.arrangement[obj_idx]

        subTree = DFS_Rec_for_Monotone_General(
            start_poses,
            goal_poses,
            self.dependency_dict,
            self.path_dict,
            self.pose_locations,
            self.linked_list,
            self.region_dict,
        )
        ### update dependency_dict and path_dict
        self.dependency_dict = subTree.dependency_dict
        self.path_dict = subTree.path_dict
        # print(self.dependency_dict)
        # print(self.path_dict)

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
            # root_arrangement, goalNode.arrangement, "save-to-solution")
            # ### visualize the branch
            # self.visualizeLocalBranch("L0", "R0", "L,R")
            return None

        ### the subproblem is non-monotone
        ### we have to reason about the failure
        ### we look at the dependency graph from the leafs
        ### Check each leaf node
        for leaf_nodeID, leaf_object_ordering in subTree.leafs.items():
            ### we first decode the arrangement out of the leaf_node
            leaf_arrangement = self.decodeArrangement(leaf_nodeID, initNode.arrangement, goalNode.arrangement)
            path_selection_dict = subTree.leaf_path_selection_dict[leaf_nodeID]
            # print("leaf arrangement: " + str(leaf_arrangement))
            # print("ordering: " + str(leaf_object_ordering))
            # print("path_selection_dict: " + str(path_selection_dict))

            ### Then we use the IP solver to get the best dependency graph or the best ordering (minimum feedback arc)
            object_to_move_leaf2goal = self.computeObjectsToMove(leaf_arrangement, goalNode.arrangement)
            object_dependency_opts = self.object_dependency_opts_generate(
                leaf_arrangement, goalNode.arrangement, object_to_move_leaf2goal
            )

            IP_arc_buffers = feedback_arc_ILP_buffers(object_dependency_opts)
            arcsrc_setSize, arcs, path_selection, object_ordering, DG, dependencyEdge_paths = IP_arc_buffers.optimum
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
            object_ranking = self.rankObjects(DG)
            ### assign a buffer to each object (so far assign the farthest buffer per object)
            objects2buffers = self.assignBuffers(object_ranking, leaf_arrangement, goalNode.arrangement)
            for hh in range(len(objects2buffers)):
                objects2buffers[hh].append(initNode.node_id)
            # print("objects2buffers: " + str(objects2buffers))

            ### draw arrangement to see what happens
            # self.visualTool.drawCurrArrangement(str(leaf_nodeID), self.points, self.poses, \
            # leaf_arrangement, goalNode.arrangement, "save-to-tree")

            for triple_task in objects2buffers:
                self.queue.insert(0, triple_task)  ### initialization of the queue

    def assignBuffers(self, object_ranking, curr_arrangement, goal_arrangement):
        ### Input: object_ranking (a list of obj_idx)
        ###        curr_arrangement (a list of pose_idx)
        ### Output: objects2buffers (a dict <key: obj_idx, value: pose_idx as buffer)

        ### we currently follow the idea of selecting the fartherst buffer for each object
        objects2buffers = []
        for obj_idx in object_ranking:
            # curr_obj_pose = curr_arrangement[obj_idx]
            # goal_obj_pose = goal_arrangement[obj_idx]
            # farthest_reachable_buffer_pose = self.findBufferForPose(
            #     curr_obj_pose, curr_arrangement, goal_obj_pose, goal_arrangement
            # )
            # objects2buffers.append([obj_idx, farthest_reachable_buffer_pose])
            buffer_pose = self.choose_buffer_pose(obj_idx, curr_arrangement)
            if buffer_pose:
                objects2buffers.append([obj_idx, buffer_pose])
            else:
                print("Could not generate Buffer " + pose_idx + "! Is this possible?")
                sys.exit(-1)

        # print("objects2buffers: " + str(objects2buffers))

        # objects2buffers_to_eliminated = []
        # for [obj, buff] in objects2buffers:
        #     if buff == -1:
        #         objects2buffers_to_eliminated.append([obj, buff])

        # for [obj, buff] in objects2buffers_to_eliminated:
        #     objects2buffers.remove([obj, buff])

        return objects2buffers

    def choose_buffer_pose(self, obj_idx, mutated_arrangement):
        bufs_for_obj = filter(lambda x: x[0] == 'B' and int(x.split(';')[-1][1:]) == obj_idx, self.space.poseMap.keys())

        ind = len(bufs_for_obj)
        didgen = genBuffers(
            1,
            self.space,
            filter(lambda x: x != mutated_arrangement[obj_idx], mutated_arrangement),
            method='object_feasible',
            param1=mutated_arrangement[obj_idx],
            count=ind,
            suffix=';O' + str(obj_idx),
        )
        pose_idx = 'B' + str(ind) + ';O' + str(obj_idx)

        if didgen:
            return pose_idx
        else:
            return None

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

    def object_dependency_opts_generate(self, query_arrangement, goal_arrangement, object_to_move):
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
            #     continue
            for path in self.dependency_dict[(pose_key1, pose_key2)]:
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
            object_candidates.add(obj1)
            object_candidates.add(obj2)

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
        for mm in range(kk + 1, len(object_ordering)):
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
        # nPoses = len(self.space.poseMap)
        # for i in range(nPoses):
        #     for j in range(i, nPoses):
        for i, j in product(self.space.poseMap.keys(), repeat=2):
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
        ### from R0, back track to left root via parent search
        self.simplePath.insert(0, "R0")
        curr_waypoint_id = self.treeR["R0"].parent_id
        while curr_waypoint_id != "L0":
            self.simplePath.insert(0, curr_waypoint_id)
            curr_waypoint_id = self.treeL[curr_waypoint_id].parent_id
        self.simplePath.insert(0, curr_waypoint_id)

        self.solution = []
        self.arrangements = []
        for i in range(1, len(self.simplePath) - 1):
            pid = self.simplePath[i - 1]
            nid = self.simplePath[i]
            carr = self.treeL[pid].arrangement
            farr = self.treeL[nid].arrangement
            for obj in self.treeL[nid].object_ordering:
                # print(carr)
                self.arrangements.append(carr[:])
                # print(obj)
                self.solution.append((obj, [carr[obj], farr[obj]]))
                carr[obj] = farr[obj]
            # self.arrangements.append(self.treeL[nid].arrangement)
            # self.solution.append((self.treeL[nid].objectMoved, self.treeL[nid].object_transition))

        # print(i, len(self.simplePath) - 2)
        carr = self.treeL[self.simplePath[-2]].arrangement
        farr = self.treeR[self.simplePath[-1]].arrangement
        for obj in self.treeR[self.simplePath[-1]].object_ordering:
            # print(carr)
            self.arrangements.append(carr[:])
            # print(obj)
            self.solution.append((obj, [carr[obj], farr[obj]]))
            carr[obj] = farr[obj]
        # print(farr)
        self.arrangements.append(farr)
        # self.arrangements.append(self.treeR[nid].arrangement)
        # self.solution.append((self.treeR[nid].objectMoved, self.treeR[nid].object_transition))
        # self.totalActions = len(self.simplePath) - 1
        # print("path: " + str(self.simplePath))
        # print("total action: " + str(self.totalActions))
        return self.solution


class ArrNode(object):
    def __init__(self, arrangement, node_id, cost_to_come, object_ordering, path_option, parent_id):
        self.arrangement = arrangement
        self.node_id = node_id
        self.cost_to_come = cost_to_come
        self.object_ordering = object_ordering  ### a ordered list of object indices
        self.path_option = path_option  ### a dict {obj_idx: path_idx}
        self.parent_id = parent_id

    def updateCostToCome(self, cost_to_come):
        self.cost_to_come = cost_to_come

    def updateObjectOrdering(self, object_ordering):
        self.object_ordering = object_ordering

    def updatePathOption(self, path_option):
        self.path_option = path_option

    def updateParent(self, parent_id):
        self.parent_id = parent_id
