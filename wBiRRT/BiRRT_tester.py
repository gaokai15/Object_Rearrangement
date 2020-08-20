from __future__ import division

from LocalSolver import feedback_arc_ILP_buffers
from util import *
import IPython
from random import sample
from collections import OrderedDict

class BiRRT_tester(object):
    ### Input:
    ### (1) initial_arrangement (a list of pose_ids, each of which indicating the initial pose for an object)
    ### (2) final_arrangement (a list of pose_ids, each of which indicating the final pose for an object)
    ### gpd
    ### (1) dependency_dict: {(pose_id1, pose_id2): [[set of pose_ids], ...], ...} (essential for IP solver)
    ### (2) path_dict: {(pose_id1, pose_id2): [[set of region_ids], ...], ...} (essential for path visualization)
    ### visualTool: a visualization tool as a debugging purpose
    ### Output:
    ### the whole plan
    def __init__(self, initial_arrangement, final_arrangement, sample_arrangements, gpd, instance, new_paths, visualTool):
        self.initial_arrangement = initial_arrangement
        self.final_arrangement = final_arrangement
        self.dependency_dict = gpd.dependency_dict
        self.path_dict = gpd.path_dict
        self.new_paths = new_paths
        self.points = instance.points + instance.buffer_points
        self.objects = instance.objects + instance.buffers
        self.numObjs = len(self.initial_arrangement)
        self.nPoses = len(self.points)
        self.allPoses = range(self.nPoses)
        self.isConnected = False
        self.treeL = {}
        self.treeR = {}
        self.trees = {}
        self.trees["Left"] = self.treeL
        self.trees["Right"] = self.treeR
        ### add the initial_arrangement and final_arrangement as the root node to two trees, respectively
        self.treeL["L0"] = Arrtest_node(
            self.initial_arrangement, "L0", None, num_actions=0, object_ordering=[], path_selection=[])
        self.treeR["R0"] = Arrtest_node(
            self.final_arrangement, "R0", None, num_actions=0, object_ordering=[], path_selection=[])
        self.leftKey = "L0"
        self.rightKey = "R0"
        self.bridge = [] ### [leftKey, rightKey, shortest_dist, object_ordering, path_selection]
        self.threshold = 2 ### every time a tree can at most expand 2 distances (2 actions)
        ################## results ################
        ### the whole_path is a list of items and each item has the following format
        ### [("node1_id", node2_id), {2:path2, 1:path1, ...}]
        self.whole_path = []
        self.totalActions = 0 ### record the total number of actions
        self.numLeftBranches = 0 ### record the number of left branches in the solution
        self.numRightBranches = 0 ### record the number of right branches in the solution
        self.numNodesInLeftTree = 0 ### record the total number of nodes in the left tree
        self.numNodesInRightTree = 0 ### record the total number of nodes in the right tree

        ### start running
        self.left_idx = 1
        self.right_idx = 1

        
        ### initial connection attempt
        self.connectToTreeR(self.treeL["L0"])

        self.sample_arrangements = sample_arrangements

        totalTimes = len(sample_arrangements)
        timeout = totalTimes
        while (self.isConnected != True and timeout > 0):
            timeout -= 1
            ### expand the left tree
            new_Arrtest_node = self.treeExpansionL(self.sample_arrangements[timeout])
            ### connect to right tree
            self.connectToTreeR(new_Arrtest_node)
            if (self.isConnected != True and timeout > 0):
                timeout -= 1
                ### expand the right tree
                new_Arrtest_node = self.treeExpansionR(self.sample_arrangements[timeout])
                ### connect to left tree
                self.connectToTreeL(new_Arrtest_node)

        if (self.isConnected == True):
            print("find the solution at timeout " + str(timeout) + "/" + str(totalTimes))

        ### handle failure
        if (self.isConnected == False):
            print("fail to find a valid within " + str(totalTimes))
            # self.numNodesInLeftTree = len(self.treeL)
            # self.numNodesInRightTree = len(self.treeR)
            # print("num of nodes generated in the left tree: " + str(self.numNodesInLeftTree))
            # print("num of nodes generated in the right tree: " + str(self.numNodesInRightTree))
        


    def treeExpansionL(self, sample_arrangement):
        # new_arrangement = self.generateNewArrangement()
        new_arrangement = sample_arrangement
        ### (1) step1: Now find the nearest node in the left tree
        shortest_dist, nearest_arrID = self.computeShortestDistance(new_arrangement, "Left")
        ### (2) step two: check the connection validity between the nearest arrangement in the left tree and new_arrangement
        object_dependency_opts = self.object_dependency_opts_generate(
                                        self.treeL[nearest_arrID].arrangement, new_arrangement)
        IP_arc_buffers = feedback_arc_ILP_buffers(object_dependency_opts)
        arc_setSize, arcs, path_selection, object_ordering = IP_arc_buffers.optimum
        if arc_setSize == 0:
            print("left tree can be expanded")
            print("current arrangement: " + str(nearest_arrID) + ": " + str(self.treeL[nearest_arrID].arrangement))
            print("next arrangement: " + "L"+str(self.left_idx) + ": " + str(new_arrangement))
            print "ILP result(the smallest size of FAS with buffers):", IP_arc_buffers.optimum
            ### now check if the distance exceeds threshold
            ### if it does, we only want to expand the tree to a small extent
            if shortest_dist > self.threshold:
                partial_arrangement, partial_path_selection, partial_object_ordering = \
                    self.partialExtension(self.treeL[nearest_arrID].arrangement, new_arrangement, 
                                    path_selection, object_ordering, shortest_dist, "Left")
                new_Arrtest_node = Arrtest_node(partial_arrangement, "L"+str(self.left_idx), 
                                nearest_arrID, self.threshold, partial_object_ordering, partial_path_selection)
                print("partial arrangement: " + "L"+str(self.left_idx) + ": " + str(partial_arrangement))
                print("path selection: " + str(partial_path_selection))
                print("object ordering: " + str(partial_object_ordering))
            else:
                new_Arrtest_node = Arrtest_node(new_arrangement, "L"+str(self.left_idx), 
                                    nearest_arrID, shortest_dist, object_ordering, path_selection)
            
            self.treeL["L"+str(self.left_idx)] = new_Arrtest_node

            # self.troubleshootLocalPath("L"+str(self.left_idx), nearest_arrID, "L,L")
            self.left_idx += 1
            print("\n")

            return new_Arrtest_node

        return None

    def treeExpansionR(self, sample_arrangement):
        # new_arrangement = self.generateNewArrangement()
        new_arrangement = sample_arrangement
        ### (1) step1: Now find the nearest node in the left tree
        shortest_dist, nearest_arrID = self.computeShortestDistance(new_arrangement, "Right")
        ### (2) step two: check the connection validity between the new_arrangement and the nearest arrangement in the right tree
        object_dependency_opts = self.object_dependency_opts_generate(
                                        new_arrangement, self.treeR[nearest_arrID].arrangement)  
        IP_arc_buffers = feedback_arc_ILP_buffers(object_dependency_opts)
        arc_setSize, arcs, path_selection, object_ordering = IP_arc_buffers.optimum
        if arc_setSize == 0:
            print("right tree can be expanded")
            print("current arrangement: " + "R"+str(self.right_idx) + ": " + str(new_arrangement))
            print("next arrangement: " + str(nearest_arrID) + ": " + str(self.treeR[nearest_arrID].arrangement))
            print "ILP result(the smallest size of FAS with buffers):", IP_arc_buffers.optimum
            ### now check if the distance exceeds threshold
            ### if it does, we only want to expand the tree to a small extent
            if shortest_dist > self.threshold:
                partial_arrangement, partial_path_selection, partial_object_ordering = \
                    self.partialExtension(self.treeR[nearest_arrID].arrangement, new_arrangement, 
                                    path_selection, object_ordering, shortest_dist, "Right")
                new_Arrtest_node = Arrtest_node(partial_arrangement, "R"+str(self.right_idx), 
                                nearest_arrID, self.threshold, partial_object_ordering, partial_path_selection)    
                print("partial arrangement: " + "R"+str(self.right_idx) + ": " + str(partial_arrangement))
                print("path selection: " + str(partial_path_selection))
                print("object ordering: " + str(partial_object_ordering))
            else:
                new_Arrtest_node = Arrtest_node(new_arrangement, "R"+str(self.right_idx), 
                                    nearest_arrID, shortest_dist, object_ordering, path_selection)
            
            self.treeR["R"+str(self.right_idx)] = new_Arrtest_node

            # self.troubleshootLocalPath("R"+str(self.right_idx), nearest_arrID, "R,R")
            self.right_idx += 1
            print("\n")

            return new_Arrtest_node

        return None

    def generateNewArrangement(self):
        isfree = False
        while (isfree != True):
            ### sample an arrangement (we currently allow duplicate node)
            new_arrangement = sample(self.allPoses, self.numObjs)
            ### check if it is a collision-free arrangement
            isfree = collisionCheck([self.objects[t] for t in new_arrangement])

        return new_arrangement

    def connectToTreeR(self, query_node):
        ### in case the query node is None
        if query_node == None:
            return
        ### see if a left node can be connected to the right tree
        ### (1) step one: find the closest node in the tree to the query node
        shortest_dist, nearest_arrID = self.computeShortestDistance(query_node.arrangement, "Right")
        ### (2) step two: check the connection validity between query node and the nearest node in the right tree
        object_dependency_opts = self.object_dependency_opts_generate(
                                                query_node.arrangement, self.treeR[nearest_arrID].arrangement)
        IP_arc_buffers = feedback_arc_ILP_buffers(object_dependency_opts)
        arc_setSize, arcs, path_selection, object_ordering = IP_arc_buffers.optimum
        # print "ILP result(the smallest size of FAS with buffers):", IP_arc_buffers.optimum
        if arc_setSize == 0:
            print("A node is connecting the right tree!")
            print("current arrangement: " + str(query_node.node_id) + ": " + str(query_node.arrangement))
            print("next arrangement: " + str(nearest_arrID) + ": " + str(self.treeR[nearest_arrID].arrangement))
            print "ILP result(the smallest size of FAS with buffers):", IP_arc_buffers.optimum
            
            ### a short bridge between left tree and right tree is found
            self.isConnected = True
            self.leftKey = query_node.node_id
            self.rightKey = nearest_arrID
            ### return a bridge with all necessary information
            self.bridge = [self.leftKey, self.rightKey, shortest_dist, object_ordering, path_selection]

            # self.troubleshootLocalPath(self.leftKey, self.rightKey, "L,R")
            print("\n")

    def connectToTreeL(self, query_node):
        ### in case the query node is None
        if query_node == None:
            return
        ### see if a right node can be connected to the left tree
        ### (1) step one: find the closest node in the tree to the query node
        shortest_dist, nearest_arrID = self.computeShortestDistance(query_node.arrangement, "Left")
        ### (2) step two: check the connection validity between the nearest node in the left tree and the query node
        object_dependency_opts = self.object_dependency_opts_generate(
                                                self.treeL[nearest_arrID].arrangement, query_node.arrangement)
        IP_arc_buffers = feedback_arc_ILP_buffers(object_dependency_opts)
        arc_setSize, arcs, path_selection, object_ordering = IP_arc_buffers.optimum
        # print "ILP result(the smallest size of FAS with buffers):", IP_arc_buffers.optimum
        if arc_setSize == 0:
            print("A node can be connecting the left tree!")
            print("current arrangement: " + str(nearest_arrID) + ": " + str(self.treeL[nearest_arrID].arrangement))
            print("next arrangement: " + str(query_node.node_id) + ": " + str(query_node.arrangement))
            print "ILP result(the smallest size of FAS with buffers):", IP_arc_buffers.optimum

            ### a short bridge between left tree and right tree is found
            self.isConnected = True
            self.leftKey = nearest_arrID
            self.rightKey = query_node.node_id
            ### return a bridge with all necessary information
            self.bridge = [self.leftKey, self.rightKey, shortest_dist, object_ordering, path_selection]

            # self.troubleshootLocalPath(self.leftKey, self.rightKey, "L,R")
            print("\n")

    def partialExtension(self, nearest_arrangement, new_arrangement, path_selection, object_ordering, 
                                                                            total_actions, expansionMode):
        num_moves = 0
        partial_arrangement = [-1]*self.numObjs
        partial_path_selection = [-1]*self.numObjs
        partial_object_ordering = object_ordering

        for obj_idx in object_ordering:
            if expansionMode == "Left": 
                if num_moves >= self.threshold: 
                    break
                partial_arrangement[obj_idx] = new_arrangement[obj_idx]
                partial_path_selection[obj_idx] = path_selection[obj_idx]
            else: 
                if num_moves >= (total_actions - self.threshold): 
                    break
                partial_arrangement[obj_idx] = nearest_arrangement[obj_idx]
                partial_path_selection[obj_idx] = 0 ### There is only one path option for a path from pose to itself
            
            if nearest_arrangement[obj_idx] != new_arrangement[obj_idx]:
                ### this is a move
                num_moves += 1            

        ### reach here since num_moves == 2 and we don't allow more moves
        ### for the rest of them, we let them stand still
        for i in range(object_ordering.index(obj_idx), len(object_ordering)):
            obj_idx = object_ordering[i]
            if expansionMode == "Left": 
                partial_arrangement[obj_idx] = nearest_arrangement[obj_idx]
                partial_path_selection[obj_idx] = 0
            else: 
                partial_arrangement[obj_idx] = new_arrangement[obj_idx]
                partial_path_selection[obj_idx] = path_selection[obj_idx]
             

        return partial_arrangement, partial_path_selection, partial_object_ordering

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

    def getPaths(self, object_path_opts, curr_Arrtest_node, next_Arrtest_node, treeMode):
        # print "\ncheck getPaths() function"
        rpaths = OrderedDict() ### so as to indicate the ordering
        if treeMode == "Bridge":
            object_ordering = self.bridge[3]
            path_selection = self.bridge[4]
        else:
            object_ordering = curr_Arrtest_node.object_ordering
            path_selection = curr_Arrtest_node.path_selection

        # print "object ordering: " + str(object_ordering)
        # print "path_selection: " + str(path_selection)

        for obj in object_ordering:
            iopt = path_selection[obj]
            # print "current obj: " + str(obj) + ", and its path opts: " + str(iopt)
            dpath = object_path_opts[obj][iopt]
            # print "the chosen path: "
            # print dpath

            if treeMode == "Left":
                start = next_Arrtest_node.arrangement[obj]
                goal = curr_Arrtest_node.arrangement[obj]
            else:
                start = curr_Arrtest_node.arrangement[obj]
                goal = next_Arrtest_node.arrangement[obj]                


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
            object_path_opts = self.object_paths_opts_generate(self.treeL[curr_waypoint].arrangement, 
                                                                    self.treeL[temp_parent].arrangement, "Left")
            ### get the concrete path segments for objects (with the right order)
            ### rpaths: {obj_idx: [(pts1, pts2, pts3)], ...}
            rpaths = self.getPaths(object_path_opts, self.treeL[curr_waypoint], self.treeL[temp_parent], "Left")
            ### put paths segments to the whole path
            self.whole_path.insert(0, [(temp_parent, curr_waypoint), rpaths])
            ### move to the its parent
            curr_waypoint = self.treeL[curr_waypoint].parent_id

        ### Now add the bridge to the whole path
        print("building the bridge betwen left tree and right tree")
        object_path_opts = self.object_paths_opts_generate(self.treeL[self.leftKey].arrangement, 
                                                                self.treeR[self.rightKey].arrangement, "Right")
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
            object_path_opts = self.object_paths_opts_generate(self.treeR[curr_waypoint].arrangement, 
                                                                    self.treeR[temp_parent].arrangement, "Right")
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
        # print("num of nodes generated in the left tree: " + str(self.numNodesInLeftTree))
        # print("num of nodes generated in the right tree: " + str(self.numNodesInRightTree))
        # print("num of left branches in the solution: " + str(self.numLeftBranches))
        # print("num of right branches in the solution: " + str(self.numRightBranches))
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
            self.visualTool.drawLocalMotions((parent_id, child_id), rpaths, 
                                self.points, self.objects, curr_arrangement, self.final_arrangement, debug=True)
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
            self.visualTool.drawLocalMotions((child_id, parent_id), rpaths, 
                                self.points, self.objects, curr_arrangement, self.final_arrangement, debug=True)
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
            self.visualTool.drawLocalMotions((child_id, parent_id), rpaths, 
                                self.points, self.objects, curr_arrangement, self.final_arrangement, debug=True)
            # IPython.embed()

class Arrtest_node(object):
    def __init__(self, arrangement, node_id, parent_id, num_actions, object_ordering, path_selection):
        self.arrangement = arrangement
        self.node_id = node_id
        self.parent_id = parent_id
        self.num_actions = num_actions
        self.object_ordering = object_ordering
        self.path_selection = path_selection
