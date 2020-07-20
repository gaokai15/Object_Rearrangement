from __future__ import division


from DPLocalSolver import DFS_Rec_for_Monotone
from util import *
import copy
import IPython
import random
from random import sample
from collections import OrderedDict
from itertools import combinations, product
import math

class Non_Monotone_Solver_General(object):
    def __init__(self, graph, obj_locations, start_poses, goal_poses):
        self.obj_locations = obj_locations
        self.path_dict = {}
        self.dependency_dict = {}
        self.object_ordering = []
        self.start_poses = copy.deepcopy(start_poses)
        self.goal_poses = copy.deepcopy(goal_poses)
        self.n = len(self.start_poses)
        self.linked_list_conversion(graph)
        self.enumerate_cases()
        self.dependency_dict_conversion()
        self.totalActions = len(self.object_ordering)
        
    def enumerate_cases(self):
        # enumerate possible cases
        FOUND = False
        for obj_num in range(self.n+1): # num of objects that need buffers
            print "number of objects that use buffers", obj_num
            for obj_set in combinations(self.start_poses.keys(), obj_num): # which objs need buffers
                for buffer_set in product(sorted(self.obj_locations.keys(), reverse=True), repeat=obj_num): # which poses are buffers
                    obj_buffer_dict = {}
                    Degrade = False # when an object uses its own start or goal pose as a buffer, Degrade = True.
                    for index in xrange(len(obj_set)):
                        obj = obj_set[index]
                        buffer = buffer_set[index]
                        if (buffer == self.start_poses[obj]) or (buffer == self.goal_poses[obj]):
                            Degrade = True
                            break
                        obj_buffer_dict[obj] = (self.n+index, buffer)
                    if Degrade:
                        continue
                    # monotone solver input path_dict, dependency_dict, obj_locations, LL, region_dict, obj_buffer_dict
                    DFS = DFS_Rec_for_Non_Monotone_General(self.start_poses, self.goal_poses, self.dependency_dict, self.path_dict, self.obj_locations, self.LL, self.region_dict, obj_buffer_dict)
                    # DFS = DFS_Rec_for_Non_Monotone_General(self.start_poses, self.goal_poses, self.dependency_dict, self.path_dict, self.obj_locations, self.LL, self.region_dict, obj_buffer_dict)
                    self.dependency_dict = copy.deepcopy(DFS.dependency_dict)
                    self.path_dict = copy.deepcopy(DFS.path_dict)
                    if len(DFS.object_ordering)>0:
                        print "Find a solution!"
                        FOUND = True
                        print "obj_buffer_dict", obj_buffer_dict
                        print "DFS.object_ordering", DFS.object_ordering
                        self.object_ordering = DFS.object_ordering
                        break
                if FOUND:
                    break
            if FOUND:
                break
        
        

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

    def dependency_dict_conversion(self):
        for key in self.dependency_dict.keys():
            number_set_list = self.dependency_dict[key]
            pose_set_list = []
            for number_set in number_set_list:
                pose_set = set()
                for number in number_set:
                    pose_set = pose_set.union({(number // 2, number % 2)})
                pose_set_list.append(pose_set)
            self.dependency_dict[key] = pose_set_list

class DFS_Rec_for_Non_Monotone_General(object):
    def __init__(self, start_poses, goal_poses, dependency_dict, path_dict, object_locations, linked_list, region_dict, obj_buffer_dict):
        self.object_ordering = []
        self.path_selection_dict = {}
        self.obstacle_lst = []
        self.start_poses = {}
        self.goal_poses = {}
        for i in start_poses.keys():
            if start_poses[i] == goal_poses[i]:
                self.obstacle_lst.append(start_poses[i])
            else:
                self.start_poses[i] = start_poses[i]
                self.goal_poses[i] = goal_poses[i]
        self.n = len(self.start_poses)
        self.b = len(obj_buffer_dict)
        self.dependency_dict = copy.deepcopy(dependency_dict)
        self.path_dict = copy.deepcopy(path_dict)
        self.obj_locations = copy.deepcopy(object_locations)
        self.linked_list = copy.deepcopy(linked_list)
        self.region_dict = copy.deepcopy(region_dict)
        self.obj_buffer_dict = copy.deepcopy(obj_buffer_dict)
        self.buffer_objects = []
        for value in self.obj_buffer_dict.values():
            self.buffer_objects.append(value[0])
        self.dynamic_programming()
        

    def dynamic_programming(self):
        complete_index = 0
        for i in self.start_poses.keys():
            complete_index += (1<<i)
        for value in self.obj_buffer_dict.values():
            complete_index += (1<<value[0])
        self.parent = {}
        self.path_option = {}
        self.explored = {}
        self.explored[0] = True
        # Recursion
        self.DFS_rec(0)


        task_index = complete_index
        if task_index in self.path_option:
            current_task = task_index
            object_ordering = []
            while current_task in self.parent:
                parent_task = self.parent[current_task]
                last_object = int(math.log(current_task - parent_task, 2))
                self.path_selection_dict[last_object] = self.path_option[current_task]
                if last_object in self.buffer_objects:
                    for key in self.obj_buffer_dict.keys():
                        if self.obj_buffer_dict[key][0] == last_object:
                            real_object = key
                            break
                    object_ordering.append( real_object)
                else:
                    object_ordering.append( last_object)
                
                
                current_task = parent_task
            self.object_ordering = list(reversed(object_ordering))
            return True
        else:
            # print "Non-monotone"
            # exit(0)
            return False
            # print MISTAKE

    def DFS_rec(self, old_node):
        FLAG = False
        for next_object in self.next_object(old_node):
            new_node = old_node + (1<<next_object)
            if new_node in self.explored:
                continue
            
            # Detect which poses are occupied
            occupied_poses = copy.deepcopy(self.obstacle_lst)
            for i in self.start_poses.keys():
                if i == next_object:
                    continue
                elif i in self.obj_buffer_dict.keys(): # objects using buffers
                    if self.obj_buffer_dict[i][0] == next_object:
                        continue
                    elif ((old_node>>(self.obj_buffer_dict[i][0]))%2):# has been at the goal pose
                        occupied_poses.append(self.goal_poses[i])
                    elif ((old_node>>(i))%2):# at the buffer
                        occupied_poses.append(self.obj_buffer_dict[i][1])
                    else: # at the start pose
                        occupied_poses.append(self.start_poses[i])
                else:
                    if ((old_node>>i)%2):
                        occupied_poses.append(self.goal_poses[i])
                    else:
                        occupied_poses.append(self.start_poses[i])

            path_index = self.transformation(occupied_poses, next_object)
            if path_index >= 0:
                self.path_option[new_node] = path_index
                self.parent[new_node] = old_node
                self.explored[new_node] = True
                if new_node == 2**(self.n+self.b) - 1:
                    return True
                FLAG = self.DFS_rec(new_node)
                if FLAG:
                    break
        return FLAG


    def next_object(self, index):
        for i in self.start_poses.keys():
            if ((index >> i)%2): # it has moved
                if (i in self.obj_buffer_dict) and (not ((index >> (self.obj_buffer_dict[i][0]))%2)): # it is at the buffer
                    yield self.obj_buffer_dict[i][0]
            else: # it is at the start pose
                yield i

    def generate_task_index(self, obj_set):
        task_index = 0
        for obj in obj_set:
            task_index += 2**obj
        return task_index

    def transformation(self,occupied_poses, obj):
        
        if obj < self.n:
            start = 2*obj
            if obj in self.obj_buffer_dict:
                goal = self.obj_buffer_dict[obj][1]
            else:
                goal = 2*obj+1
        else:
            for key in self.obj_buffer_dict.keys():
                if self.obj_buffer_dict[key][0] == obj:
                    real_object = key
                    break
            start = self.obj_buffer_dict[real_object][1]
            goal = 2*real_object+1
        dependency_dict_key = (min(start, goal), max(start, goal))
        if dependency_dict_key not in self.dependency_dict:
            self.dependency_dict[dependency_dict_key] = []
            self.path_dict[dependency_dict_key] = []
        for path_index in range(len(self.dependency_dict[dependency_dict_key])):
            path = self.dependency_dict[dependency_dict_key][path_index]
            OCCUPIED = False
            for pose in path:
                if pose in occupied_poses:
                    OCCUPIED = True
                    break
            if not OCCUPIED:
                return path_index
        Available_Regions = []
        for region in self.region_dict.keys():
            OCCUPIED = False
            for pose in region:
                if pose in occupied_poses:
                    OCCUPIED = True
                    break
            if not OCCUPIED:
                Available_Regions.append(self.region_dict[region])
        if (self.region_dict[self.obj_locations[goal]] not in Available_Regions):
            # print "Not accessable"
            return -1
        if (self.region_dict[self.obj_locations[start]] not in Available_Regions):
            # print "Not accessable"
            return -1
        if (self.region_dict[self.obj_locations[start]] == self.region_dict[self.obj_locations[goal]]):
            path = [self.region_dict[self.obj_locations[start]]]
            dep_set = set(self.get_dependency_set_from_index(self.region_dict[self.obj_locations[start]]))
            self.path_dict[dependency_dict_key].append(list(reversed(path)))
            self.dependency_dict[dependency_dict_key].append(dep_set)
            return len(self.dependency_dict[dependency_dict_key]) - 1
            
        Found = False
        parents = {}
        explored = {}
        for key in self.region_dict.values():
            explored[key] = False
        queue = [self.region_dict[self.obj_locations[start]]]
        explored[self.region_dict[self.obj_locations[start]]] = True
        while (len(queue) >0) and (not Found):
            # stack(-1) for DFS and queue(0) for BFS
            old_node = queue.pop(-1)
            if old_node in self.linked_list:
                for region in self.linked_list[old_node]:
                    if explored[region]:
                        continue
                    if region not in Available_Regions:
                        continue
                    parents[region] = old_node
                    if region == self.region_dict[self.obj_locations[goal]]:
                        Found = True
                        break
                    queue.append(region)
                    explored[region] = True
            else:
                print("Linked list error")
        
        if Found:
            path = []
            dep_set = set()
            current_node = self.region_dict[self.obj_locations[goal]]
            while current_node in parents:
                path.append(current_node)
                dep_set = dep_set.union(self.get_dependency_set_from_index(current_node))
                current_node = parents[current_node]
            path.append(current_node)
            dep_set = dep_set.union(self.get_dependency_set_from_index(current_node))
            if dependency_dict_key[0]==start:
                self.path_dict[dependency_dict_key].append(list(reversed(path)))
            else:
                self.path_dict[dependency_dict_key].append(list(path))
            self.dependency_dict[dependency_dict_key].append(dep_set)
            return len(self.dependency_dict[dependency_dict_key]) - 1
        else:
            return -1
                    
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
