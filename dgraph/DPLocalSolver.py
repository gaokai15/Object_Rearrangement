from __future__ import division

import copy
import math

# import time


### DP local solver (purely monotone)
# DFS_rec not considering buffers where the start/goal poses are not necessarily 2*i/2*i+1
class DFS_Rec_for_Monotone_General(object):
    def __init__(self, start_poses, goal_poses, dependency_dict, path_dict, object_locations, linked_list, region_dict):
        self.object_ordering = []  ### a list of object indices when the problem is solved
        self.path_selection_dict = {}  ### key: object index, value: path index
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

        self.dependency_dict = copy.deepcopy(dependency_dict)
        self.path_dict = copy.deepcopy(path_dict)
        self.obj_locations = copy.deepcopy(object_locations)
        self.linked_list = copy.deepcopy(linked_list)
        self.region_dict = copy.deepcopy(region_dict)
        # startTime_dynamicProgramming = time.clock()
        self.dynamic_programming()
        # print("Time for core function dynamic programmming: " + str(time.clock() - startTime_dynamicProgramming))

    def dynamic_programming(self):
        self.falseLeafs = []
        self.leafs = {}  ### key: leaf id, value: object ordering (this is for the case of isMonotone == False)
        self.leaf_path_selection_dict = {}  ### key: leaf id, value: dict {obj: path_option}
        self.parent = {}
        self.path_option = {}  ### key: leaf id, values: path option for a single move
        self.explored = {}
        self.queue = [0]
        self.explored[0] = True
        # it is a stack when pop(-1)
        old_node = self.queue.pop(-1)
        # Recursion
        self.DFS_rec(old_node)

        task_index = 0
        for i in self.start_poses.keys():
            task_index += (1 << i)
        if task_index in self.path_option:
            current_task = task_index
            object_ordering = []
            while current_task in self.parent:
                parent_task = self.parent[current_task]
                last_object = int(math.log(current_task - parent_task, 2))
                self.path_selection_dict[last_object] = self.path_option[current_task]
                object_ordering.append(last_object)
                current_task = parent_task
            self.object_ordering = list(reversed(object_ordering))
            # print "DFS_REC_MONOTONE", self.object_ordering
            self.leafs[task_index] = self.object_ordering
            self.isMonotone = True
            return True
        else:
            # print "Non-monotone"
            # exit(0)
            self.getLeafsAndOrderings()
            self.isMonotone = False
            return False
            # print MISTAKE

    def getLeafsAndOrderings(self):
        child_dict = {}
        for child_id, parent_id in self.parent.items():
            if parent_id not in child_dict.keys():
                child_dict[parent_id] = []
            if child_id not in child_dict.keys():
                child_dict[child_id] = []
            child_dict[parent_id].append(child_id)
        ### Now find the leafs, excluding falseLeafs
        for leaf_candidate, children_list in child_dict.items():
            if child_dict[leaf_candidate] == [] and leaf_candidate not in self.falseLeafs:
                self.leafs[leaf_candidate] = []
        ### for those leafs, get the curresponding object ordering
        for leaf in self.leafs.keys():
            temp_object_ordering = []
            temp_path_selection_dict = {}
            current_task = leaf
            while current_task in self.parent:
                parent_task = self.parent[current_task]
                last_object = int(math.log(current_task - parent_task, 2))
                temp_path_selection_dict[last_object] = self.path_option[current_task]
                temp_object_ordering.append(last_object)
                current_task = parent_task
            temp_object_ordering = list(reversed(temp_object_ordering))
            self.leafs[leaf] = temp_object_ordering
            self.leaf_path_selection_dict[leaf] = temp_path_selection_dict
        ### As we get all the ordering, check if some of the ordering is a subset of the others
        redundant_leafs = []
        for leaf_id, temp_order in self.leafs.items():
            if self.checkSubset(temp_order):
                redundant_leafs.append(leaf_id)
        for redundant_leaf in redundant_leafs:
            self.leafs.pop(redundant_leaf)
            self.leaf_path_selection_dict.pop(redundant_leaf)

        if self.leafs == {}:
            self.leafs[0] = []
            self.leaf_path_selection_dict[0] = []

    def checkSubset(self, temp_order):
        isSubset = False
        temp_order = set(temp_order)
        for ordering in self.leafs.values():
            ordering = set(ordering)
            if temp_order == ordering:
                continue
            elif temp_order.issubset(ordering):
                isSubset = True
                break

        return isSubset

    def DFS_rec(self, old_node):
        FLAG = False  # Flag = True iff we find a solution to the rearrangement problem
        for next_object in self.next_object(old_node):
            new_node = old_node + (1 << next_object)
            if new_node in self.explored:
                if old_node not in self.falseLeafs:
                    self.falseLeafs.append(old_node)
                continue

            # Detect which poses are occupied
            occupied_poses = copy.deepcopy(self.obstacle_lst)
            for i in self.start_poses.keys():
                if i == next_object:
                    continue
                elif ((old_node >> i) % 2):
                    occupied_poses.append(self.goal_poses[i])
                else:
                    occupied_poses.append(self.start_poses[i])

            path_index = self.transformation(occupied_poses, next_object)
            if path_index >= 0:
                self.path_option[new_node] = path_index
                self.parent[new_node] = old_node
                self.queue.append(new_node)
                self.explored[new_node] = True
                complete_node = 0
                for i in self.start_poses.keys():
                    complete_node += (1 << i)
                if new_node == complete_node:
                    return True
                # if new_node == (1<<(self.n)) - 1:
                #     return True
                FLAG = self.DFS_rec(new_node)
                if FLAG:
                    break
        return FLAG

    def next_object(self, index):
        for i in self.start_poses.keys():
            if ((index >> i) % 2):  # it has moved
                pass
            else:  # it is at the start pose
                yield i

    def generate_task_index(self, obj_set):
        task_index = 0
        for obj in obj_set:
            task_index += 2**obj
        return task_index

    def transformation(self, occupied_poses, obj):
        start = self.start_poses[obj]
        goal = self.goal_poses[obj]
        # dependency_dict_key = (min(start, goal), max(start, goal))
        dependency_dict_key = tuple(sorted([start, goal]))
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
        while (len(queue) > 0) and (not Found):
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
            if dependency_dict_key[0] == start:
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
        for pose in region_tuple:
            try:
                value = int(pose)
            except ValueError:
                if (pose[0] in "SGB"):  # it was a string, not an int.
                    dependency_set.add(pose)
        return dependency_set


### DP local solver (with one buffer)
# DFS_rec non monotone where the start/goal poses are not necessarily 2*i/2*i+1
class DFS_Rec_for_Non_Monotone_General(object):
    def __init__(
        self, start_poses, goal_poses, dependency_dict, path_dict, object_locations, linked_list, region_dict,
        obj_buffer_dict
    ):
        ###### output   ############
        self.mutation_nodes = []
        ######
        self.object_ordering = []  ### a list of object indices when the problem is solved
        self.path_selection_dict = {}  ### key: object index, value: path_index
        self.obstacle_lst = []
        self.start_poses = {}
        self.goal_poses = {}
        for i in start_poses.keys():
            if start_poses[i] == goal_poses[i]:
                self.obstacle_lst.append(start_poses[i])
            else:
                self.start_poses[i] = start_poses[i]
                self.goal_poses[i] = goal_poses[i]
        self.n = len(start_poses)
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

        # startTime_dynamicProgramming_nonMonotone = time.clock()
        self.dynamic_programming()
        # print("Time for core function dynamic programmming for non monotone: " + str(time.clock() - startTime_dynamicProgramming_nonMonotone))

    def dynamic_programming(self):
        self.falseLeafs = []
        self.leafs = {}  ### key: leaf id, value: object ordering (this is for the case of isMonotone == False)
        self.leaf_path_selection_dict = {}  ### key: leaf id, value: dict {obj: path_option}
        self.parent = {}
        self.path_option = {}  ### key: leaf id, values: path option for a single move
        self.explored = {}
        self.explored[0] = True
        # Recursion
        self.DFS_rec(0)

        complete_index = 0
        for i in self.start_poses.keys():
            complete_index += (1 << i)
        for value in self.obj_buffer_dict.values():
            complete_index += (1 << value[0])

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
                    object_ordering.append(real_object)
                else:
                    object_ordering.append(last_object)

                current_task = parent_task
            self.object_ordering = list(reversed(object_ordering))
            self.leafs[task_index] = self.object_ordering
            self.isMonotone = True
            return True
        else:
            if (len(self.obj_buffer_dict) > 0):
                ###### pick out nodes with mutations ######
                Greatest_Progress = -1
                mutation_obj = self.obj_buffer_dict.keys()[0]
                for node in self.parent.keys():
                    if (((node >> mutation_obj) % 2) and (not ((node >> self.obj_buffer_dict[mutation_obj][0]) % 2))):
                        if (not ((self.parent[node] >> mutation_obj) % 2)):  # first mutation node in the branch
                            progress = 0
                            for i in self.start_poses.keys():
                                if ((node >> mutation_obj) % 2):
                                    progress += 1
                            if progress > Greatest_Progress:
                                Greatest_Progress = progress
                                self.mutation_nodes = [node]
                # print "mutation"
                # print self.mutation_nodes
            # print "Non-monotone"
            # exit(0)
            # startTime_filterLeafs_nonMonotone = time.clock()
            self.getLeafsAndOrdering_nonMonotone()
            # print("Time for getting leafs and orderings nonMonotone: " + str(time.clock() - startTime_filterLeafs_nonMonotone))
            self.isMonotone = False
            return False
            # print MISTAKE

    def getLeafsAndOrdering_nonMonotone(self):
        ### in non-monotone solver, so far we can trust that
        ### the mutation_node is the only leaf
        ### so let's directly get the ordering and path_selection
        if self.mutation_nodes == []:
            self.leafs[0] = []
            self.leaf_path_selection_dict[0] = []
        else:
            temp_object_ordering = []
            temp_path_selection_dict = {}
            leaf = self.mutation_nodes[0]  ### it should be only a single id (int)
            current_task = leaf
            while current_task in self.parent:
                parent_task = self.parent[current_task]
                last_object = int(math.log(current_task - parent_task, 2))
                temp_path_selection_dict[last_object] = self.path_option[current_task]
                temp_object_ordering.append(last_object)
                current_task = parent_task
            temp_object_ordering = list(reversed(temp_object_ordering))
            self.leafs[leaf] = temp_object_ordering
            self.leaf_path_selection_dict[leaf] = temp_path_selection_dict

    def DFS_rec(self, old_node):
        FLAG = False
        for next_object in self.next_object(old_node):
            new_node = old_node + (1 << next_object)
            if new_node in self.explored:
                if old_node not in self.falseLeafs:
                    self.falseLeafs.append(old_node)
                continue

            # Detect which poses are occupied
            occupied_poses = copy.deepcopy(self.obstacle_lst)
            for i in self.start_poses.keys():
                if i == next_object:
                    continue
                elif i in self.obj_buffer_dict.keys():  # objects using buffers
                    if self.obj_buffer_dict[i][0] == next_object:
                        continue
                    elif ((old_node >> (self.obj_buffer_dict[i][0])) % 2):  # has been at the goal pose
                        occupied_poses.append(self.goal_poses[i])
                    elif ((old_node >> (i)) % 2):  # at the buffer
                        occupied_poses.append(self.obj_buffer_dict[i][1])
                    else:  # at the start pose
                        occupied_poses.append(self.start_poses[i])
                else:
                    if ((old_node >> i) % 2):
                        occupied_poses.append(self.goal_poses[i])
                    else:
                        occupied_poses.append(self.start_poses[i])

            path_index = self.transformation(occupied_poses, next_object)
            if path_index >= 0:
                self.path_option[new_node] = path_index
                self.parent[new_node] = old_node
                self.explored[new_node] = True
                complete_node = 0
                for i in self.start_poses.keys():
                    complete_node += (1 << i)
                for value in self.obj_buffer_dict.values():
                    complete_node += (1 << value[0])
                if new_node == complete_node:
                    return True
                # if new_node == 2**(self.n+self.b) - 1:
                #     return True
                FLAG = self.DFS_rec(new_node)
                if FLAG:
                    break
        return FLAG

    def next_object(self, index):
        for i in self.start_poses.keys():
            if ((index >> i) % 2):  # it has moved
                # it is at the buffer
                if (i in self.obj_buffer_dict) and (not ((index >> (self.obj_buffer_dict[i][0])) % 2)):
                    yield self.obj_buffer_dict[i][0]
            else:  # it is at the start pose
                yield i

    def generate_task_index(self, obj_set):
        task_index = 0
        for obj in obj_set:
            task_index += 2**obj
        return task_index

    def transformation(self, occupied_poses, obj):

        if obj < self.n:
            start = self.start_poses[obj]
            if obj in self.obj_buffer_dict:
                goal = self.obj_buffer_dict[obj][1]
            else:
                goal = self.goal_poses[obj]
        else:
            for key in self.obj_buffer_dict.keys():
                if self.obj_buffer_dict[key][0] == obj:
                    real_object = key
                    break
            start = self.obj_buffer_dict[real_object][1]
            goal = self.goal_poses[real_object]
        # dependency_dict_key = (min(start, goal), max(start, goal))
        dependency_dict_key = tuple(sorted([start, goal]))
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
        while (len(queue) > 0) and (not Found):
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
            if dependency_dict_key[0] == start:
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
        for pose in region_tuple:
            try:
                value = int(pose)
            except ValueError:
                if (pose[0] in "SGB"):  # it was a string, not an int.
                    dependency_set.add(pose)
        return dependency_set
