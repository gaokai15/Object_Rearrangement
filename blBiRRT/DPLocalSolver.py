from __future__ import division

import IPython
import copy
import math
from itertools import combinations, product


# DFS_rec not considering buffers where the start/goal poses are not necessarily 2*i/2*i+1
class DFS_Rec_for_Monotone_General(object):
    def __init__(self, start_poses, goal_poses, dependency_dict, path_dict, object_locations, linked_list, region_dict):
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

        self.dependency_dict = copy.deepcopy(dependency_dict)
        self.path_dict = copy.deepcopy(path_dict)
        self.obj_locations = copy.deepcopy(object_locations)
        self.linked_list = copy.deepcopy(linked_list)
        self.region_dict = copy.deepcopy(region_dict)
        self.dynamic_programming()

    def dynamic_programming(self):
        self.parent = {}
        self.path_option = {}
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
            self.isMonotone = True
            return True
        else:
            # print "Non-monotone"
            # exit(0)
            self.isMonotone = False
            return False
            # print MISTAKE

    def DFS_rec(self, old_node):
        FLAG = False  # Flag = True iff we find a solution to the rearrangement problem
        for next_object in self.next_object(old_node):
            new_node = old_node + (1 << next_object)
            if new_node in self.explored:
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
                if new_node == 1 << (self.n) - 1:
                    return True
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
        for i in region_tuple:
            value = -1
            try:
                value = int(i)
            except ValueError:
                pass  # it was a string, not an int.
            if value >= -0.5:
                dependency_set = dependency_set.union({value})
        return dependency_set


class DFS_Rec_for_Monotone(object):
    def __init__(self, start_poses, goal_poses, dependency_dict, path_dict, object_locations, linked_list, region_dict):
        self.isMonotone = False
        self.n = len(start_poses)
        self.start_poses = start_poses
        self.goal_poses = goal_poses
        self.dependency_dict = copy.deepcopy(dependency_dict)
        self.path_dict = copy.deepcopy(path_dict)
        self.obj_locations = copy.deepcopy(object_locations)
        self.linked_list = copy.deepcopy(linked_list)
        self.region_dict = copy.deepcopy(region_dict)
        self.dynamic_programming()
        # print("finish DP")

    def dynamic_programming(self):
        self.parent = {}
        self.path_option = {}
        self.object_ordering = []
        self.explored = {}
        self.queue = [0]
        self.explored[0] = 0
        # it is a stack when pop(-1)
        old_node = self.queue.pop(-1)
        # Recursion
        self.DFS_rec(old_node)

        task_index = 2**(self.n) - 1
        if task_index in self.path_option:
            current_task = task_index
            path_selection_dict = {}
            object_ordering = []
            while current_task in self.parent:
                parent_task = self.parent[current_task]
                last_object = int(math.log(current_task - parent_task, 2))
                path_selection_dict[last_object] = self.path_option[current_task]
                object_ordering.append(last_object)

                current_task = parent_task
            path_selection_list = []
            for i in range(self.n):
                path_selection_list.append(path_selection_dict[i])
            self.path_selection = tuple(path_selection_list)
            self.object_ordering = list(reversed(object_ordering))
            self.isMonotone = True
            return True
        else:
            # print "Non-monotone"
            # exit(0)
            self.isMonotone = False
            return False
            # print MISTAKE

    def DFS_rec(self, old_node):
        FLAG = False  # Flag = True iff we find a solution to the rearrangement problem
        for next_object in self.next_object(old_node):
            new_node = old_node + (1 << next_object)
            if new_node in self.explored:
                continue

            # Detect which poses are occupied
            occupied_poses = []
            for i in range(self.n):
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
                self.explored[new_node] = 0
                if new_node == 1 << (self.n) - 1:
                    return True
                FLAG = self.DFS_rec(new_node)
                if FLAG:
                    break
        return FLAG

    def next_object(self, index):
        for i in range(self.n):
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
            explored[key] = 0
        queue = [self.region_dict[self.obj_locations[start]]]
        explored[self.region_dict[self.obj_locations[start]]] = 1
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
                    explored[region] = 1
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
        for i in region_tuple:
            value = -1
            try:
                value = int(i)
            except ValueError:
                pass  # it was a string, not an int.
            if value >= -0.5:
                dependency_set = dependency_set.union({value})
        return dependency_set


# brute force non monotone solver
class Non_Monotone_Solver(object):
    def __init__(self, graph, obj_locations, num_obj, obj):
        self.obj_id = obj
        self.obj_locations = obj_locations
        self.path_dict = {}
        self.path_option = {}
        self.dependency_dict = {}
        self.object_ordering = []
        self.n = num_obj
        self.num_extra_buffer = len(self.obj_locations) - 2 * self.n
        self.start_pose = range(0, self.n)
        self.linked_list_conversion(graph)
        self.enumerate_cases()
        self.dependency_dict_conversion()
        self.FOUND = False

    def enumerate_cases(self):
        # enumerate possible cases
        self.FOUND = False
        # for obj_num in range(self.n + 1):  # num of objects that need buffers
        for obj_num in range(2):  # num of objects that need buffers
            print "number of objects that use buffers", obj_num
            # for obj_set in combinations(range(self.n), obj_num):
            # print("Obj num: ", obj_num, list(combinations(range(self.n), obj_num)))
            obj_set = []
            if obj_num > 0:
                obj_set.append(self.obj_id // 2)
            for buffer_set in product(range(2 * self.n + self.num_extra_buffer - 1, -1, -1), repeat=obj_num):
                obj_buffer_dict = {}
                Degrade = False
                for index in xrange(len(obj_set)):
                    obj = obj_set[index]
                    buffer = buffer_set[index]
                    if (buffer == 2 * obj) or (buffer == 2 * obj + 1):
                        Degrade = True
                        break
                    obj_buffer_dict[obj] = (self.n + index, buffer)
                if Degrade:
                    continue
                # monotone solver input path_dict, dependency_dict, obj_locations, LL, region_dict, obj_buffer_dict
                DFS = DFS_for_Non_Monotone(
                    self.n, self.dependency_dict, self.path_dict, self.obj_locations, self.LL, self.region_dict,
                    obj_buffer_dict
                )
                # DFS = DFS_Rec_for_Non_Monotone(self.n, self.dependency_dict, self.path_dict, self.obj_locations, self.LL, self.region_dict, obj_buffer_dict)
                self.dependency_dict = copy.deepcopy(DFS.dependency_dict)
                self.path_dict = copy.deepcopy(DFS.path_dict)
                self.path_option = copy.deepcopy(DFS.path_option)
                if len(DFS.object_ordering) > 0:
                    print "Find a solution!"
                    self.FOUND = True
                    print "obj_buffer_dict", obj_buffer_dict
                    print "DFS.object_ordering", DFS.object_ordering
                    self.object_ordering = DFS.object_ordering
                    break
            if self.FOUND:
                break
        # if self.FOUND:
        #     break

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


# DFS for the non monotone solvers considering buffers
class DFS_for_Non_Monotone(object):
    def __init__(
        self, num_obj, dependency_dict, path_dict, object_locations, linked_list, region_dict, obj_buffer_dict
    ):
        self.n = num_obj
        self.b = len(obj_buffer_dict)  # number of objects that uses buffers
        self.dependency_dict = copy.deepcopy(dependency_dict)
        self.path_dict = copy.deepcopy(path_dict)
        self.obj_locations = copy.deepcopy(object_locations)
        self.linked_list = copy.deepcopy(linked_list)
        self.region_dict = copy.deepcopy(region_dict)
        self.obj_buffer_dict = copy.deepcopy(obj_buffer_dict)
        self.dynamic_programming()

    def dynamic_programming(self):
        parent = {}
        self.path_option = {}
        self.object_ordering = []
        explored = {}
        queue = [0]
        explored[0] = True
        FOUND = False
        while (len(queue) > 0) & (not FOUND):
            old_node = queue.pop(-1)
            for next_object in self.next_object(old_node):
                new_node = old_node + (1 << next_object)
                if new_node in explored:
                    continue

                # Detect which poses are occupied
                occupied_poses = []
                for i in range(self.n):
                    if i == next_object:
                        continue
                    elif i in self.obj_buffer_dict.keys():  # objects using buffers
                        if self.obj_buffer_dict[i][0] == next_object:
                            continue
                        elif ((old_node >> (self.obj_buffer_dict[i][0])) % 2):  # has been at the goal pose
                            occupied_poses.append(2 * i + 1)
                        elif ((old_node >> (i)) % 2):  # at the buffer
                            occupied_poses.append(self.obj_buffer_dict[i][1])
                        else:  # at the start pose
                            occupied_poses.append(2 * i)
                    else:
                        if ((old_node >> i) % 2):
                            occupied_poses.append(2 * i + 1)
                        else:
                            occupied_poses.append(2 * i)

                path_index = self.transformation(occupied_poses, next_object)
                if path_index >= 0:
                    self.path_option[new_node] = path_index
                    parent[new_node] = old_node
                    queue.append(new_node)
                    explored[new_node] = True
                    if new_node == 2**(self.n + self.b) - 1:
                        FOUND = True
                        break

        task_index = 2**(self.n + self.b) - 1
        if task_index in self.path_option:
            current_task = task_index
            path_selection_dict = {}
            object_ordering = []
            while current_task in parent:
                parent_task = parent[current_task]
                last_object = int(math.log(current_task - parent_task, 2))
                path_selection_dict[last_object] = self.path_option[current_task]
                if last_object > self.n:
                    for key in self.obj_buffer_dict.keys():
                        if self.obj_buffer_dict[key][0] == last_object:
                            real_object = key
                            break
                    object_ordering.append(real_object)
                else:
                    object_ordering.append(last_object)

                current_task = parent_task
            path_selection_list = []
            for i in range(self.n + self.b):
                path_selection_list.append(path_selection_dict[i])
            self.path_selection = tuple(path_selection_list)
            self.object_ordering = list(reversed(object_ordering))
            return True
        else:
            # print "Non-monotone"
            # exit(0)
            return False
            # print MISTAKE

    def next_object(self, index):
        for i in range(self.n):
            if ((index >> i) % 2):  # it has moved
                if (i in self.obj_buffer_dict) and (not ((index >>
                                                          (self.obj_buffer_dict[i][0])) % 2)):  # it is at the buffer
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
            start = 2 * obj
            if obj in self.obj_buffer_dict:
                goal = self.obj_buffer_dict[obj][1]
            else:
                goal = 2 * obj + 1
        else:
            for key in self.obj_buffer_dict.keys():
                if self.obj_buffer_dict[key][0] == obj:
                    real_object = key
                    break
            start = self.obj_buffer_dict[real_object][1]
            goal = 2 * real_object + 1
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
        for i in region_tuple:
            value = -1
            try:
                value = int(i)
            except ValueError:
                pass  # it was a string, not an int.
            if value >= -0.5:
                dependency_set = dependency_set.union({value})
        return dependency_set


# DFS non monotone where the start/goal poses are not necessarily 2*i/2*i+1
class DFS_for_Non_Monotone_General(object):
    def __init__(
        self, start_poses, goal_poses, dependency_dict, path_dict, object_locations, linked_list, region_dict,
        obj_buffer_dict
    ):
        self.object_ordering = []
        self.path_selection_dict = {}
        self.obstacle_lst = []
        self.start_poses = {}
        self.goal_poses = {}
        for i in start_poses.keys():
            # if start_poses[i] == goal_poses[i]:
            #     self.obstacle_lst.append(start_poses[i])
            # else:
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
        print(self.FOUND)

    def dynamic_programming(self):
        complete_index = 0
        for i in self.start_poses.keys():
            complete_index += (1 << i)
        for value in self.obj_buffer_dict.values():
            complete_index += (1 << value[0])
        self.parent = {}
        self.path_option = {}
        self.object_ordering = []
        explored = {}
        queue = [0]
        explored[0] = True
        self.FOUND = False
        while (len(queue) > 0) & (not self.FOUND):
            old_node = queue.pop(-1)
            for next_object in self.next_object(old_node):
                new_node = old_node + (1 << next_object)
                if new_node in explored:
                    continue

                # Detect which poses are occupied
                occupied_poses = copy.deepcopy(self.obstacle_lst)
                for i in self.start_poses.keys():
                    if i == next_object:
                        continue
                    elif i in self.obj_buffer_dict:  # objects using buffers
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
                    queue.append(new_node)
                    explored[new_node] = True
                    if new_node == complete_index:
                        self.FOUND = True
                        break

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
            return True
        else:
            # print "Non-monotone"
            # exit(0)
            return False
            # print MISTAKE

    def next_object(self, index):
        for i in self.start_poses.keys():
            if ((index >> i) % 2):  # it has moved
                if (i in self.obj_buffer_dict) and (not ((index >>
                                                          (self.obj_buffer_dict[i][0])) % 2)):  # it is at the buffer
                    yield self.obj_buffer_dict[i][0]
            else:  # it is at the start pose
                yield i

    def generate_task_index(self, obj_set):
        task_index = 0
        for obj in obj_set:
            task_index += 2**obj
        return task_index

    def transformation(self, occupied_poses, obj):
        # print(obj)
        if obj < self.n:
            start = 2 * obj
            if obj in self.obj_buffer_dict:
                goal = self.obj_buffer_dict[obj][1]
            else:
                goal = 2 * obj + 1
        else:
            for key in self.obj_buffer_dict.keys():
                if self.obj_buffer_dict[key][0] == obj:
                    real_object = key
                    break
            # print(obj, self.n)
            start = self.obj_buffer_dict[real_object][1]
            goal = 2 * real_object + 1
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
        for i in region_tuple:
            value = -1
            try:
                value = int(i)
            except ValueError:
                pass  # it was a string, not an int.
            if value >= -0.5:
                dependency_set = dependency_set.union({value})
        return dependency_set
