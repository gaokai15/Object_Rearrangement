import os
import sys
import copy
from collections import OrderedDict
from cgraph import genDenseCGraph, drawLocalMotions, drawEntireAnimation
# from cgraph import genCGraph, genDenseCGraph, loadCGraph, drawMotions, animatedMotions
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
import cPickle as pickle

import IPython
from util import *
from random import sample
import time
from itertools import permutations

my_path = os.path.abspath(os.path.dirname(__file__))

class Experiments(object):

    def single_instance(self, numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, example_index):

        # print "\nFirst generate the instance and get the dense cgraph..."
        graph, paths, objects, wall_pts, color_pool, points, objectShape, object_locations = genDenseCGraph(
            numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, example_index
        )
        ### keep generating instances until it is successfully generated
        while graph == False:
            graph, paths, objects, wall_pts, color_pool, points, objectShape, object_locations = genDenseCGraph(
                numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, example_index
            )

        if graph is paths is objects is False:
            return -1

        # print "graph: " + str(graph)
        # print "\n"
        # print "paths: " + str(paths)
        # print "\n"
        # print "object_locations: " + str(object_locations)
        # print "\n"
        # print "points: " + str(points)
        # print "\n"

        initial_arrangement = [i for i in range(0, 2*numObjs, 2)]
        final_arrangement = [i for i in range(1, 2*numObjs, 2)]
        print "initial_arrangement: " + str(initial_arrangement)
        print "final_arrangement: " + str(final_arrangement)

        ## Generate path options
        start_time = time.clock()
        gpd = Dense_Path_Generation(graph, object_locations)
        print "total time for path generation: " + str(time.clock()-start_time)
        print "total path connections: " + str(len(gpd.dependency_dict))
        # print "gpd.dependency_dict: "
        # for keys, values in gpd.dependency_dict.items():
        #     print str(keys) + ": " + str(values)
        # print "\n"
        # print "gpd.path_dict: "
        # for keys, values in (gpd.path_dict).items():
        #     print str(keys) + ": " + str(values)
        # print "\n"

        
        new_paths = {}
        for r1, r2 in paths.keys():
            new_paths[(gpd.region_dict[r1], gpd.region_dict[r2])] = copy.deepcopy(paths[(r1, r2)])
        # print "new paths: "
        # for keys, values in new_paths.items():
        #     print str(keys) + ": " + str(values)
        # print "\n"

        ###########################################################################################
        ### The above generate the region graph, edge connections, 
        ### and all path options (constraints & regions)
        ### And they are done once for all
        ############################################################################################
        ### Now let's build bi-RRT incremental algorithm to
        ### find a plan from start to goal
        start_time = time.clock()
        plan = Arrangement_biRRT(initial_arrangement, final_arrangement, gpd.dependency_dict, gpd.path_dict, new_paths, points, objects, objectShape, HEIGHT, WIDTH, numObjs, RAD, color_pool)
        print "Time to perform arrangement biRRT is: " + str(time.clock()-start_time)
        print("\nFinally show the whole path.")
        if plan.isConnected:
            ### let's save all local paths to see if it makes sense
            for local_path in plan.whole_path:
                arr_pair = local_path[0]
                curr_arrangement_id = arr_pair[0]
                next_arrangement_id = arr_pair[1]
                rpaths = local_path[1]
                if curr_arrangement_id[0] == "L":
                    drawLocalMotions(
                        HEIGHT, WIDTH, numObjs, RAD, (curr_arrangement_id, next_arrangement_id), \
                        rpaths, color_pool, points, plan.treeL[curr_arrangement_id].arrangement, \
                        final_arrangement, example_index, objects, saveimage=True)
                if curr_arrangement_id[0] == "R":
                    drawLocalMotions(
                        HEIGHT, WIDTH, numObjs, RAD, (curr_arrangement_id, next_arrangement_id), \
                        rpaths, color_pool, points, plan.treeR[curr_arrangement_id].arrangement, \
                        final_arrangement, example_index, objects, saveimage=True)                    

            ### finally let's draw the animation
            drawEntireAnimation(HEIGHT, WIDTH, numObjs, RAD, plan, final_arrangement, color_pool, points, example_index, objectShape)



class Arrangement_biRRT(object):
    ### Input:
    ### (1) dependency_dict: {(pose_id1, pose_id2): [[set of pose_ids], ...], ...} (essential for IP solver)
    ### (2) path_dict: {(pose_id1, pose_id2): [[set of region_ids], ...], ...} (essential for path visualization)
    ### (3) new_paths: {(region_id1, region_id2): [[(pt1_x, pt1_y), (pt2_x, pt2_y)], ...], ...} (essential for path visualization)
    ### (4) initial_arrangement (a list of pose_ids, each of which indicating the initial pose for an object)
    ### (4) final_arrangement (a list of pose_ids, each of which indicating the final pose for an object)
    ### Output:
    ### the whole plan
    def __init__(self, initial_arrangement, final_arrangement, dependency_dict, path_dict, new_paths, points, objects, objectShape, HEIGHT, WIDTH, numObjs, RAD, color_pool):
        self.initial_arrangement = initial_arrangement
        self.final_arrangement = final_arrangement
        self.dependency_dict = dependency_dict
        # print "dependency_dict: "
        # for keys, values in self.dependency_dict.items():
        #     print str(keys) + ": "
        #     print values
        # print "\n"
        self.path_dict = path_dict
        # print "path_dict: "
        # for keys, values in self.path_dict.items():
        #     print str(keys) + ": "
        #     print values
        # print "\n"
        self.new_paths = new_paths
        # print "new_paths: "
        # for keys, values in self.path_dict.items():
        #     print str(keys) + ": " + str(values)
        # print "\n"
        self.points = points
        self.objects = objects
        self.objectShape = objectShape
        self.numObj = len(initial_arrangement)
        self.nPoses = len(self.points)
        self.allPoses = range(self.nPoses)
        self.isConnected = False
        self.treeL = {} ### left tree
        self.treeR = {} ### right tree
        self.trees = {}
        self.trees["Left"] = self.treeL
        self.trees["Right"] = self.treeR
        ### add the initial_arrangement and final_arrangement as the root node to two trees, respectively
        self.treeL["L0"] = ArrNode(self.initial_arrangement, "L0", None, num_actions=0, object_ordering=[], path_selection=[])
        self.treeR["R0"] = ArrNode(self.final_arrangement, "R0", None, num_actions=0, object_ordering=[], path_selection=[])
        self.leftKey = "L0"
        self.rightKey = "R0"
        self.bridge = [] ### [leftKey, rightKey, shortest_dist, object_ordering, path_selection]
        ################## results ################
        ### the whole_path is a list of items and each item has the following format
        ### [("node1_id", node2_id), {2:path2, 1:path1, ...}]
        self.whole_path = []
        self.totalActions = 0 ### record the total number of actions
        self.numLeftBranches = 0 ### record the number of left branches in the solution
        self.numRightBranches = 0 ### record the number of right branches in the solution
        self.numNodesInLeftTree = 0 ### record the total number of nodes in the left tree
        self.numNodesInRightTree = 0 ### record the total number of nodes in the right tree

        ### temporary variables for debugging purposes and will be deleted later
        self.HEIGHT = HEIGHT
        self.WIDTH = WIDTH
        self.numObjs = numObjs
        self.RAD = RAD
        self.color_pool = color_pool

        # self.printTree("Left")
        # self.printTree("Right")

        self.left_idx = 1
        self.right_idx = 1
        ### initial connection attempt
        self.connectToTreeR(self.treeL["L0"])

        totalTimes = 300
        timeout = totalTimes
        while (self.isConnected != True and timeout > 0):
            timeout -= 1
            ### expand the left tree
            new_arrNode = self.treeExpansionL()
            ### connect to right tree
            self.connectToTreeR(new_arrNode)
            if (self.isConnected != True and timeout > 0):
                timeout -= 1
                ### expand the right tree
                new_arrNode = self.treeExpansionR()
                ### connect to left tree
                self.connectToTreeL(new_arrNode)
        

        if (self.isConnected == True):
            print("find the solution at timeout " + str(timeout) + "/" + str(totalTimes))
            self.constructWholePath()
            self.getSolutionStats()

        ### handle failure
        if (self.isConnected == False):
            print("fail to find a valid within " + str(totalTimes))
            self.numNodesInLeftTree = len(self.treeL)
            self.numNodesInRightTree = len(self.treeR)
            print("num of nodes generated in the left tree: " + str(self.numNodesInLeftTree))
            print("num of nodes generated in the right tree: " + str(self.numNodesInRightTree))


    def treeExpansionL(self):
        new_arrangement = self.generateNewArrangement()
        # print("the new arrangement for left expansion is collision-free: " + str(new_arrangement))
        ### (1) step1: Now find the nearest node in the left tree
        shortest_dist, nearest_arrID = self.computeShortestDistance(new_arrangement, "Left")
        ### (2) step two: check the connection validity between the nearest arrangement in the left tree and new_arrangement
        # print("current arrangement: ") + str(self.treeL[nearest_arrID].arrangement)
        # print("next arrangement: ") + str(new_arrangement)
        object_dependency_opts = self.object_dependency_opts_generate(self.treeL[nearest_arrID].arrangement, new_arrangement)
        # print("object_dependency_opts: ")
        # for obj, dependency_sets in object_dependency_opts.items():
        #     print str(obj) + ": " + str(dependency_sets)
        IP_arc_buffers = feedback_arc_ILP_buffers(object_dependency_opts)
        arc_setSize, arcs, path_selection, object_ordering = IP_arc_buffers.optimum
        # print "ILP result(the smallest size of FAS with buffers):", IP_arc_buffers.optimum
        if arc_setSize == 0:
            print("left tree expanded")
            print("current arrangement: " + str(nearest_arrID) + ": " + str(self.treeL[nearest_arrID].arrangement))
            print("next arrangement: " + "L"+str(self.left_idx) + ": " + str(new_arrangement))
            print "ILP result(the smallest size of FAS with buffers):", IP_arc_buffers.optimum
            print("\n")
            ### create a new node for this node
            new_arrNode = ArrNode(new_arrangement, "L"+str(self.left_idx), nearest_arrID, shortest_dist, object_ordering, path_selection)
            self.treeL["L"+str(self.left_idx)] = new_arrNode

            # self.troubleshootLocalPath("L"+str(self.left_idx), nearest_arrID, "L,L")
            self.left_idx += 1

            return new_arrNode

        # print("\n")
        return None

    def treeExpansionR(self):
        new_arrangement = self.generateNewArrangement()
        # print("the new arrangement for right expansion is collision-free: " + str(new_arrangement))
        ### (1) step1: Now find the nearest node in the left tree
        shortest_dist, nearest_arrID = self.computeShortestDistance(new_arrangement, "Right")
        # print("current arrangement: ") + str(new_arrangement)
        # print("next arrangement: ") + str(self.treeR[nearest_arrID].arrangement)
        ### (2) step two: check the connection validity between the new_arrangement and the nearest arrangement in the right tree
        object_dependency_opts = self.object_dependency_opts_generate(new_arrangement, self.treeR[nearest_arrID].arrangement)
        # print("object_dependency_opts: ")
        # for obj, dependency_sets in object_dependency_opts.items():
        #     print str(obj) + ": " + str(dependency_sets)        
        IP_arc_buffers = feedback_arc_ILP_buffers(object_dependency_opts)
        arc_setSize, arcs, path_selection, object_ordering = IP_arc_buffers.optimum
        # print "ILP result(the smallest size of FAS with buffers):", IP_arc_buffers.optimum
        if arc_setSize == 0:
            print("right tree expanded")
            print("current arrangement: " + "R"+str(self.right_idx) + ": " + str(new_arrangement))
            print("next arrangement: " + str(nearest_arrID) + ": " + str(self.treeR[nearest_arrID].arrangement))
            print "ILP result(the smallest size of FAS with buffers):", IP_arc_buffers.optimum
            print("\n")
            ### create a new node for this node
            new_arrNode = ArrNode(new_arrangement, "R"+str(self.right_idx), nearest_arrID, shortest_dist, object_ordering, path_selection)
            self.treeR["R"+str(self.right_idx)] = new_arrNode

            # self.troubleshootLocalPath("R"+str(self.right_idx), nearest_arrID, "R,R")
            self.right_idx += 1

            return new_arrNode
        # print("\n")
        return None    


    def generateNewArrangement(self):
        isfree = False
        while (isfree != True):
            ### sample an arrangement (we currently allow duplicate node)
            new_arrangement = sample(self.allPoses, self.numObj)
            ### check if it is a collision-free arrangement
            isfree = collisionCheck([self.objects[t] for t in new_arrangement])

        return new_arrangement


    def connectToTreeR(self, query_node):

        ### in case the query node is None
        if query_node == None:
            return
        # print("Trying to connect to the right tree...")
        ### see if a left node can be connected to the right tree
        ### (1) step1: find the closest node in the tree to the query node
        shortest_dist, nearest_arrID = self.computeShortestDistance(query_node.arrangement, "Right")
        ### (2) step two: check the connection validity between query node and the nearest node in the right tree
        # print("current arrangement: ") + str(query_node.arrangement)
        # print("next arrangement: ") + str(self.treeR[nearest_arrID].arrangement)
        object_dependency_opts = self.object_dependency_opts_generate(query_node.arrangement, self.treeR[nearest_arrID].arrangement)
        # print("object_dependency_opts: ")
        # for obj, dependency_sets in object_dependency_opts.items():
        #     print str(obj) + ": " + str(dependency_sets)        
        IP_arc_buffers = feedback_arc_ILP_buffers(object_dependency_opts)
        arc_setSize, arcs, path_selection, object_ordering = IP_arc_buffers.optimum
        # print "ILP result(the smallest size of FAS with buffers):", IP_arc_buffers.optimum
        if arc_setSize == 0:
            print("A node is connecting the right tree!")
            print("current arrangement: " + str(query_node.node_id) + ": " + str(query_node.arrangement))
            print("next arrangement: " + str(nearest_arrID) + ": " + str(self.treeR[nearest_arrID].arrangement))
            print "ILP result(the smallest size of FAS with buffers):", IP_arc_buffers.optimum
            print("\n")
            ### a bridge between left tree and right tree is found
            self.isConnected = True
            self.leftKey = query_node.node_id
            self.rightKey = nearest_arrID

            ### return a bridge with all necessary information
            self.bridge = [self.leftKey, self.rightKey, shortest_dist, object_ordering, path_selection]

            # self.troubleshootLocalPath(self.leftKey, self.rightKey, "L,R")

        # print("\n")

    def connectToTreeL(self, query_node):

        if query_node == None:
            return
        # print("Trying to connect to the left tree...")
        ### see if a right node can be connected to the left tree
        ### (1) step1: find the closest node in the tree to the query node
        shortest_dist, nearest_arrID = self.computeShortestDistance(query_node.arrangement, "Left")
        ### (2) step two: check the connection validity between the nearest node in the left tree and the query node
        # print("current arrangement: ") + str(self.treeL[nearest_arrID].arrangement)
        # print("next arrangement: ") + str(query_node.arrangement)
        object_dependency_opts = self.object_dependency_opts_generate(self.treeL[nearest_arrID].arrangement, query_node.arrangement)
        # print("object_dependency_opts: ")
        # for obj, dependency_sets in object_dependency_opts.items():
        #     print str(obj) + ": " + str(dependency_sets)
        IP_arc_buffers = feedback_arc_ILP_buffers(object_dependency_opts)
        arc_setSize, arcs, path_selection, object_ordering = IP_arc_buffers.optimum
        # print "ILP result(the smallest size of FAS with buffers):", IP_arc_buffers.optimum
        if arc_setSize == 0:
            print("A node is connecting the left tree!")
            print("current arrangement: " + str(nearest_arrID) + ": " + str(self.treeL[nearest_arrID].arrangement))
            print("next arrangement: " + str(query_node.node_id) + ": " + str(query_node.arrangement))
            print "ILP result(the smallest size of FAS with buffers):", IP_arc_buffers.optimum
            print("\n")
            ### a bridge between left tree and right tree is found
            self.isConnected = True
            self.leftKey = nearest_arrID
            self.rightKey = query_node.node_id

            ### return a bridge with all necessary information
            self.bridge = [self.leftKey, self.rightKey, shortest_dist, object_ordering, path_selection]

            # self.troubleshootLocalPath(self.leftKey, self.rightKey, "L,R")

        # print("\n")      


    def compute_dist(self, arr1, arr2):
        ### compute the distance between two arrangements
        ### distance metric: the distance between two arrangement is the number of movements
        ### from one arrangement to another
        num_movements = 0
        for i in range(len(arr1)):
            if arr1[i] != arr2[i]:
                num_movements += 1
        return num_movements      

    def computeShortestDistance(self, arrangement, tree_id):
        ### compute the shortest distance from the query node to a node in the tree specified
        shortest_dist = float('inf')
        nearest_arrID = None
        for node_id, node in self.trees[tree_id].items():
            temp_dist = self.compute_dist(arrangement, node.arrangement)
            if temp_dist < shortest_dist:
                shortest_dist = temp_dist
                nearest_arrID = node_id

        return shortest_dist, nearest_arrID


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


    def object_paths_opts_generate(self, curr_arrangement, next_arrangement, treeMode):
        num_objs = len(curr_arrangement)
        object_path_opts = {}
        for obj_idx in range(num_objs):
            if obj_idx not in object_path_opts.keys():
                object_path_opts[obj_idx] = []

            if treeMode == "Left":
                if next_arrangement[obj_idx] <= curr_arrangement[obj_idx]:
                    object_path_opts[obj_idx] = self.path_dict[(next_arrangement[obj_idx], curr_arrangement[obj_idx])]
                else:
                    temp_paths = self.path_dict[(curr_arrangement[obj_idx], next_arrangement[obj_idx])]
                    for temp_path in temp_paths:
                        object_path_opts[obj_idx].append(list(reversed(temp_path)))

            if treeMode == "Right":
                if curr_arrangement[obj_idx] <= next_arrangement[obj_idx]:
                    object_path_opts[obj_idx] = self.path_dict[(curr_arrangement[obj_idx], next_arrangement[obj_idx])]
                else:
                    temp_paths = self.path_dict[(next_arrangement[obj_idx], curr_arrangement[obj_idx])]
                    for temp_path in temp_paths:
                        object_path_opts[obj_idx].append(list(reversed(temp_path)))

        return object_path_opts

    def getPaths(self, object_path_opts, curr_ArrNode, next_ArrNode, treeMode):
        # print "\ncheck getPaths() function"
        rpaths = OrderedDict() ### so as to indicate the ordering
        if treeMode == "Bridge":
            object_ordering = self.bridge[3]
            path_selection = self.bridge[4]
        else:
            object_ordering = curr_ArrNode.object_ordering
            path_selection = curr_ArrNode.path_selection

        # print "object ordering: " + str(object_ordering)
        # print "path_selection: " + str(path_selection)

        for obj in object_ordering:
            iopt = path_selection[obj]
            # print "current obj: " + str(obj) + ", and its path opts: " + str(iopt)
            dpath = object_path_opts[obj][iopt]
            # print "the chosen path: "
            # print dpath

            if treeMode == "Left":
                start = next_ArrNode.arrangement[obj]
                goal = curr_ArrNode.arrangement[obj]
            else:
                start = curr_ArrNode.arrangement[obj]
                goal = next_ArrNode.arrangement[obj]                


            rpath = [self.points[start]]
            for i in range(len(dpath) - 2):
                region1 = dpath[i]
                region2 = dpath[i + 1]
                if (region1, region2) in self.new_paths.keys():
                    rpath += self.new_paths[(region1, region2)][:-1] ### only add region1 point
                elif (region2, region1) in self.new_paths.keys():
                    rpath += list(reversed(self.new_paths[(region2, region1)]))[:-1]
                else:
                    print "invalid path 1"
                    exit()
            if len(dpath) >= 2:
                region1 = dpath[-2]
                region2 = dpath[-1]
                if (region1, region2) in self.new_paths.keys():
                    rpath += self.new_paths[(region1, region2)]
                elif (region2, region1) in self.new_paths.keys():
                    rpath += list(reversed(self.new_paths[(region2, region1)]))
                else:
                    print "invalid path 2"
                    exit()
            rpath.append(self.points[goal])
            rpaths[obj] = rpath

        return rpaths


    def constructWholePath(self):
        ### from leftKey, back track to left root via parent search (get all paths from the left tree)
        curr_waypoint = self.leftKey
        print("construct the path on the left tree")
        while curr_waypoint != "L0":
            temp_parent = self.treeL[curr_waypoint].parent_id
            object_path_opts = self.object_paths_opts_generate(self.treeL[curr_waypoint].arrangement, self.treeL[temp_parent].arrangement, "Left")
            ### get the concrete path segments for objects (with the right order)
            ### rpaths: {obj_idx: [(pts1, pts2, pts3)], ...}
            rpaths = self.getPaths(object_path_opts, self.treeL[curr_waypoint], self.treeL[temp_parent], "Left")
            ### put paths segments to the whole path
            self.whole_path.insert(0, [(temp_parent, curr_waypoint), rpaths])
            ### move to the its parent
            curr_waypoint = self.treeL[curr_waypoint].parent_id

        ### Now add the bridge to the whole path
        print("building the bridge betwen left tree and right tree")
        object_path_opts = self.object_paths_opts_generate(self.treeL[self.leftKey].arrangement, self.treeR[self.rightKey].arrangement, "Right")
        ### get the concrete path segments for objects (with the right order)
        ### rpaths: {obj_idx: [(pts1, pts2, pts3)], ...}
        rpaths = self.getPaths(object_path_opts, self.treeL[self.leftKey], self.treeR[self.rightKey], "Bridge")
        ### put paths segments to the whole path
        self.whole_path.append([(self.leftKey, self.rightKey), rpaths])

        ### from rightKey, back track to right root via parent search (get all paths from the right tree)
        curr_waypoint = self.rightKey
        print("construct the path on the right tree")
        while curr_waypoint != "R0":
            temp_parent = self.treeR[curr_waypoint].parent_id
            object_path_opts = self.object_paths_opts_generate(self.treeR[curr_waypoint].arrangement, self.treeR[temp_parent].arrangement, "Right")
            ### get the concrete path segments for objects (with the right order)
            ### rpaths: {obj_idx: [(pts1, pts2, pts3)], ...}
            rpaths = self.getPaths(object_path_opts, self.treeR[curr_waypoint], self.treeR[temp_parent], "Right")
            ### put paths segments to the whole path
            self.whole_path.append([(curr_waypoint, temp_parent), rpaths])
            ### move to the its parent
            curr_waypoint = self.treeR[curr_waypoint].parent_id

    def getSolutionStats(self):
        self.numNodesInLeftTree = len(self.treeL)
        self.numNodesInRightTree = len(self.treeR)
        ### now get total actions and num of left and right branches in the solution
        for local_path in self.whole_path:
            temp_curr_arrangement_id = local_path[0][0] ### current arrangement
            temp_next_arrangement_id = local_path[0][1] ### next arrangement
            if temp_curr_arrangement_id[0] == "L" and temp_next_arrangement_id[0] == "L":
                self.totalActions += self.treeL[temp_next_arrangement_id].num_actions
                self.numLeftBranches += 1
            if temp_curr_arrangement_id[0] == "L" and temp_next_arrangement_id[0] == "R":
                self.totalActions += self.bridge[2]
            if temp_curr_arrangement_id[0] == "R" and temp_next_arrangement_id[0] == "R":
                self.totalActions += self.treeR[temp_curr_arrangement_id].num_actions
                self.numRightBranches += 1
        ### optional: just for printing purposes (can be commented when perform large experiment)
        for local_path in self.whole_path:
            print(local_path[0])
        print("num of nodes generated in the left tree: " + str(self.numNodesInLeftTree))
        print("num of nodes generated in the right tree: " + str(self.numNodesInRightTree))
        print("num of left branches in the solution: " + str(self.numLeftBranches))
        print("num of right branches in the solution: " + str(self.numRightBranches))
        print("total actions: " + str(self.totalActions))



    ### This function is only called when you want to troubleshoot what's going wrong with the local path
    def troubleshootLocalPath(self, child_id, parent_id, connectMode):
        ### Three connect modes
        ### (1) "L,L"
        ### (2) "R,R"
        ### (3) "L,R"
        print("Now let's performing the troubleshooting")
        if connectMode == "L,L":
            currNode = self.treeL[child_id]
            parentNode = self.treeL[parent_id]
            print "current arrangement: " + str(child_id) + ", " + str(currNode.arrangement)
            print "parent arrangement: " + str(parent_id) + ", " + str(parentNode.arrangement)
            object_path_opts = self.object_paths_opts_generate(currNode.arrangement, parentNode.arrangement, "Left")
            ### check if the path_opts are correct and reversed if necessary
            print "object_path_opts: "
            for obj, paths in object_path_opts.items():
                print str(obj) + ": " + str(paths)
            rpaths = self.getPaths(object_path_opts, currNode, parentNode, "Left")
            print "rpaths: "
            for obj, path in rpaths.items():
                print str(obj) + ": " + str(path)
            ### show the local solution here
            curr_arrangement = parentNode.arrangement
            drawLocalMotions(
                self.HEIGHT, self.WIDTH, self.numObjs, self.RAD, (parent_id, child_id), rpaths, self.color_pool, self.points, curr_arrangement, self.final_arrangement, example_index, self.objects, saveimage=True)
            # IPython.embed()

        if connectMode == "L,R":
            currNode = self.treeL[child_id]
            parentNode = self.treeR[parent_id]
            print "current arrangement: " + str(child_id) + ", " + str(currNode.arrangement)
            print "parent arrangement: " + str(parent_id) + ", " + str(parentNode.arrangement)
            object_path_opts = self.object_paths_opts_generate(currNode.arrangement, parentNode.arrangement, "Right")
            ### check if the path_opts are correct and reversed if necessary
            print "object_path_opts: "
            for obj, paths in object_path_opts.items():
                print str(obj) + ": " + str(paths)
            rpaths = self.getPaths(object_path_opts, currNode, parentNode, "Bridge")
            print "rpaths: "
            for obj, path in rpaths.items():
                print str(obj) + ": " + str(path)
            ### show the local solution here
            curr_arrangement = currNode.arrangement
            drawLocalMotions(
                self.HEIGHT, self.WIDTH, self.numObjs, self.RAD, (child_id, parent_id), rpaths, self.color_pool, self.points, curr_arrangement, self.final_arrangement, example_index, self.objects, saveimage=True)
            # IPython.embed()

        if connectMode == "R,R":
            currNode = self.treeR[child_id]
            parentNode = self.treeR[parent_id]
            print "current arrangement: " + str(child_id) + ", " + str(currNode.arrangement)
            print "parent arrangement: " + str(parent_id) + ", " + str(parentNode.arrangement)
            object_path_opts = self.object_paths_opts_generate(currNode.arrangement, parentNode.arrangement, "Right")
            ### check if the path_opts are correct and reversed if necessary
            print "object_path_opts: "
            for obj, paths in object_path_opts.items():
                print str(obj) + ": " + str(paths)
            rpaths = self.getPaths(object_path_opts, currNode, parentNode, "Right")
            print "rpaths: "
            for obj, path in rpaths.items():
                print str(obj) + ": " + str(path)
            ### show the local solution here
            curr_arrangement = currNode.arrangement
            drawLocalMotions(
                self.HEIGHT, self.WIDTH, self.numObjs, self.RAD, (child_id, parent_id), rpaths, self.color_pool, self.points, curr_arrangement, self.final_arrangement, example_index, self.objects, saveimage=True)
            # IPython.embed()

    def printTree(self, tree_id):
        ### Usually we will not call this function during our search process
        ### This function is mostly used for debugging
        print "\n" + str(tree_id) + " tree"
        for node_id, node in self.trees[tree_id].items():
            print "node " + str(node_id) + ": " + str(node.arrangement)
            print "parent_id: " + str(node.parent_id)
            print "num_actions: " + str(node.num_actions)
            print "object_ordering: " + str(node.object_ordering)
            print "path_selection: " + str(node.path_selection)


class ArrNode(object):
    def __init__(self, arrangement, node_id, parent_id, num_actions, object_ordering, path_selection):
        self.arrangement = arrangement
        self.node_id = node_id
        self.parent_id = parent_id
        self.num_actions = num_actions
        self.object_ordering = object_ordering
        self.path_selection = path_selection


class Dense_Path_Generation(object):
    # Input: Danniel's region connectivity graph
    # Output: the dependency dictionary
    def __init__(self, graph, obj_locations):
        self.obj_locations = obj_locations
        self.path_dict = {}
        self.dependency_dict = {}
        self.n = len(obj_locations)
        self.poses = range(0, self.n)
        print "the number of poses:", self.n
        print "poses: " + str(self.poses)
        self.linked_list_conversion(graph)
        self.construct_path_dict()
        self.dependency_dict_conversion()
        # dependency_dict: key:(5,8) value: [set((2,1), (1,0), ..., (4,0)), set(...)]
        ### Rui: In my setup, the dependency_dict should look like
        ### key: (pose1, pose2)
        ### value: [ set1([.....]), set2([.....]), ... ]

    def dependency_dict_conversion(self):
        for key in self.dependency_dict.keys():
            number_set_list = self.dependency_dict[key]
            pose_set_list = []
            for number_set in number_set_list:
                pose_set = set()
                for number in number_set:
                    # pose_set = pose_set.union({(number // 2, number % 2)})
                    # if number != key[0] and number != key[1]:
                    pose_set = pose_set.union({number})
                pose_set_list.append(pose_set)
            self.dependency_dict[key] = pose_set_list

    def linked_list_conversion(self, graph):
        # print "graph"
        # print graph
        self.region_dict = {}  # (1,2,'a'): 0
        self.LL = {}  # 0:[1,2,3]
        for key in graph:
            index = len(self.region_dict.keys())
            self.region_dict[key] = index
            self.LL[index] = []
        for key in graph:
            for v in graph[key]:
                self.LL[self.region_dict[key]].append(self.region_dict[v])
        # print "LL"
        # print self.LL
        # print "region dict"
        # print self.region_dict

    def construct_path_dict(self):
        for i in range(len(self.poses)):
            key1 = self.poses[i]
            for j in range(i,len(self.poses)):
                if i == j:
                    # if j%2:
                    #     self.dependency_dict[(key1, key1)] = [self.get_dependency_set_from_index(self.region_dict[self.obj_locations[key1]])]
                    #     self.path_dict[(key1, key1)] = [[self.region_dict[self.obj_locations[key1]], self.region_dict[self.obj_locations[key1]]]]
                    self.dependency_dict[(key1, key1)] = [self.get_dependency_set_from_index(self.region_dict[self.obj_locations[key1]])]
                    self.path_dict[(key1, key1)] = [[self.region_dict[self.obj_locations[key1]]]]
                    continue
                key2 = self.poses[j]
                key = (key1, key2) ### key1 <= key2
                self.dependency_set_pruning_search(key)

    def dependency_set_pruning_search(self, key):
        self.path_dict[key] = []
        self.dependency_dict[key] = []  # key:obj,obj, value: a list of dependency set
        vertex2node_dict = {}
        for region in self.region_dict.values():
            vertex2node_dict[region] = []
        node_dependency_set_dict = {}  # the dictionary for the dependency set of each node in the search tree
        nodes = {}
        parents = {}
        nodes[1] = self.region_dict[self.obj_locations[key[0]]]
        node_dependency_set_dict[1] = self.get_dependency_set_from_index(nodes[1])
        vertex2node_dict[self.region_dict[self.obj_locations[key[0]]]].append(1)
        queue = [1]
        while len(queue) > 0:
            old_node = queue.pop()
            if nodes[old_node] in self.LL:  # if it has neighbors
                for pose in self.LL[nodes[old_node]]:
                    current_dependency_set = node_dependency_set_dict[old_node].union(
                        self.get_dependency_set_from_index(pose)
                    )
                    Abandoned = False
                    remove_list = []
                    for n in vertex2node_dict[pose]:
                        if current_dependency_set.issuperset(node_dependency_set_dict[n]):
                            Abandoned = True
                            break
                        if node_dependency_set_dict[n].issuperset(current_dependency_set):
                            remove_list.append(n)
                    for n in remove_list:
                        vertex2node_dict[pose].remove(n)
                    if not Abandoned:
                        node = len(nodes) + 1
                        nodes[node] = pose
                        parents[node] = old_node
                        vertex2node_dict[pose].append(node)
                        queue.append(node)
                        node_dependency_set_dict[node] = current_dependency_set

        goal_nodes = vertex2node_dict[self.region_dict[self.obj_locations[key[1]]]]

        for node in goal_nodes:
            current_node = node
            path = []
            while current_node in parents:  # while it is not the root(start pose).
                path.append(nodes[current_node])
                current_node = parents[current_node]
            path.append(nodes[current_node])
            # In this model we should keep the start and goal poses in the dependency graph, because they could be occupied
            # node_dependency_set_dict[node] = node_dependency_set_dict[node].difference({key[0], key[1]})
            self.path_dict[key].append(list(reversed(path)))
            self.dependency_dict[key].append(node_dependency_set_dict[node])

        # print "parents", parents
        # print "goal", goal_nodes
        # print "nodes", nodes
        # print "node_dependency_set_dict"
        # print node_dependency_set_dict
        # print "vertex2node_dict"
        # print vertex2node_dict

    def get_path_dependency_set(self, node, parents, nodes, key):
        current_node = node
        path = []
        dependency_set = set()
        while current_node in parents:  # while it is not the root(start pose).
            path.append(nodes[current_node])
            dependency_set = dependency_set.union(self.get_dependency_set_from_index(nodes[current_node]))
            current_node = parents[current_node]
        path.append(current_node)
        dependency_set = dependency_set.union(self.get_dependency_set_from_index(nodes[current_node]))
        dependency_set = dependency_set.difference({2 * key, 2 * key + 1})
        self.path_dict[key].append(list(reversed(path)))
        self.dependency_dict[key].append(dependency_set)

    def get_dependency_set_from_index(self, index):
        for key, value in self.region_dict.items():
            if value == index:
                region_tuple = key
                break
        dependency_set = set()
        for i in region_tuple:
            value = -1
            try:
                value = int(i)
            except ValueError:
                pass  # it was a string, not an int.
            if value >= -0.5:
                dependency_set = dependency_set.union({value})
        return dependency_set


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

        # for keys, values in dependency_dict.items():
        #     print str(keys) + ": " + str(values)

        
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


class feedback_arc_ILP(object):
    def __init__(self, path_dict):
        self.path_dict = path_dict
        self.optimum = self.run_arc_ILP()

    def run_arc_ILP(self):
        MOST_PATH = 0
        for paths in self.path_dict.values():
            MOST_PATH = max(MOST_PATH, len(paths))
        # print "most paths", MOST_PATH
        n = len(self.path_dict)
        print "n: " + str(n)
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
            for j in range(len(self.path_dict[objs[i]]), MOST_PATH):
                m.addConstr(x[i, j] == 0)
        for i in range(n):
            m.addConstr(sum(x[i, u] for u in range(MOST_PATH)) == 1)
        m.addConstrs(
            C[j, k] <= sum(x[objs.index(i), u]
                            for (i, u) in dependency_dict[(j, k)])
            for (j, k) in dependency_dict.keys()
        )
        for (j, k) in dependency_dict.keys():
            for (i, u) in dependency_dict[(j, k)]:
                m.addConstr(C[j, k] >= x[objs.index(i), u])

        m.addConstrs(
            C[i, i] == 0 for i in range(n)
        )

        m.optimize()
        obj = m.getObjective()

        print("\nFAS output: ")

        ### figure out what path options are chosen for each object
        print "path_selection: "
        path_selection = []
        for i in range(n):
            for j in range(MOST_PATH):
                if x[i, j].x > 0.5:
                    path_selection.append(j)
                    print "Object " + str(i) + ": " + str(self.path_dict[i][j])
        print("path_selection: ", path_selection)


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
        print("The dependecy graph is: ")
        print(DG)

        ### print final ordering
        Y = np.zeros([n, n])
        for i in range(n):
            for j in range(n):
                Y[i, j] = y[i, j].x
        obj_indexes = [i for i in range(n)]
        order_count = np.sum(Y, axis=1)
        final_order = [obj_index for _, obj_index in sorted(zip(order_count, obj_indexes), reverse=True)]
        print("Final objects order: ")
        print(final_order)


        return obj.getValue(), edges_to_be_removed, tuple(path_selection), final_order

################################################################################################

class feedback_vertex_ILP(object):
    def __init__(self, path_dict):
        self.path_dict = path_dict
        self.optimum = self.run_vertex_ILP()

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

        print("\nFVS output: ")

        ### figure out what path options are chosen for each object
        print "path_selection: "
        path_selection = []
        for i in range(n):
            for j in range(MOST_PATH):
                if x[i, j].x > 0.5:
                    path_selection.append(j)
                    print "Object " + str(i) + ": " + str(self.path_dict[i][j])
        print("path_selection: ", path_selection)


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
        final_order = [obj_index for _, obj_index in sorted(zip(order_count, obj_indexes), reverse=True)]
        print("Final objects order: ")
        print(final_order)


        return obj.getValue(), vertices_to_be_removed, tuple(path_selection), final_order

################################################################################################

if __name__ == "__main__":
    if (len(sys.argv) < 5):
        print(
            '''Error: deptree.py: <# objects> <height> <width> <radius>
            [display?: (y/n)] [display more?: (y/n)] [save file] [load file?:(y/n)]'''
        )
        exit()

    try:
        numObjs = int(sys.argv[1])
        RAD = float(sys.argv[2])
        HEIGHT = int(sys.argv[3])
        WIDTH = int(sys.argv[4])
    except ValueError:
        print(
            '''Error: deptree.py: <# objects> <height> <width> <radius>
            [display?: (y/n)] [display more?: (y/n)] [save file] [load file?:(y/n)]'''
        )
        exit()

    display = False
    if (len(sys.argv) > 5):
        display = sys.argv[5] not in ('n', 'N')

    displayMore = False
    if (len(sys.argv) > 6):
        displayMore = sys.argv[6] not in ('n', 'N')

    savefile = False
    if (len(sys.argv) > 7):
        savefile = sys.argv[7]

    loadfile = False
    if (len(sys.argv) > 8):
        loadfile = sys.argv[8] not in ('n', 'N')
        if sys.argv[8] in ('x', 'X'):
            savefile = False
            loadfile = False

    saveimage = False
    if (len(sys.argv) > 9):
        saveimage = sys.argv[9] not in ('n', 'N')

    example_index = int(sys.argv[10])

    EXP = Experiments()
    print "Number of objects: ", numObjs
    print "Radius: ", RAD
    print "Environment size: ", HEIGHT, WIDTH

    if loadfile:
        EXP.load_instance(savefile, True, display, displayMore)
    else:
        EXP.single_instance(numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, example_index)

        