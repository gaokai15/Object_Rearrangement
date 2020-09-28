from __future__ import division
import IPython


class DensePathGenerator(object):
    # Input: Danniel's region connectivity graph
    # Output: the dependency dictionary
    def __init__(self, graph, pose_locations):
        self.pose_locations = pose_locations
        self.path_dict = {} ### is empty at the beginning, and we will enlarge it dynamically
        self.dependency_dict = {} ### is empty at the beginning, and we will enlarge it dynamically
        self.n = len(pose_locations)
        self.poses = range(0, self.n)
        print "the number of poses:", self.n
        print "poses: " + str(self.poses)
        self.linked_list_conversion(graph)
        # self.construct_path_dict()
        # self.dependency_dict_conversion()
        ### Rui: In my setup, the dependency_dict should look like
        ### key: (pose1, pose2)
        ### value: [ set1([.....]), set2([.....]), ... ]


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


    def dependency_dict_conversion_one_key(self, key):
        ### This function does some post-processing about the dependecy_list
        ### we just get from querying key = (key1, key2) to make sure for each 
        ### dependency set in this key, there are no duplicate elements
        number_set_list = self.dependency_dict[key]
        pose_set_list = []
        for number_set in number_set_list:
            pose_set = set()
            for number in number_set:
                pose_set = pose_set.union({number})
            pose_set_list.append(pose_set)
        self.dependency_dict[key] = pose_set_list




    def update_dependency_path_dict(self, key1, key2, numObjs):
        ### This function updates the dependency_dict and path_dict
        ### by adding the dependency set and path set for transition (key1, key2)
        if key1 == key2:
            key = (key1, key1)
            self.dependency_dict[(key1, key1)] = [self.get_dependency_set_from_regionID(self.region_dict[self.pose_locations[key1]])]
            self.path_dict[(key1, key1)] = [[self.region_dict[self.pose_locations[key1]]]]

        else:
            ### key1 < key2
            key = (key1, key2)
            self.dependency_set_pruning_search(key, numObjs)

        ### does the post-processing
        self.dependency_dict_conversion_one_key(key)


    def dependency_set_pruning_search(self, key, numObjs):
        print("key: " + str(key))
        self.path_dict[key] = []
        self.dependency_dict[key] = []  # key:obj,obj, value: a list of dependency set
        regionID2nodes_dict = {}
        for regionID in self.region_dict.values():
            regionID2nodes_dict[regionID] = []
        node_dependency_set_dict = {}  # the dictionary for the dependency set of each node in the search tree
        nodes = {}
        parents = {}
        nodes[1] = self.region_dict[self.pose_locations[key[0]]]
        node_dependency_set_dict[1] = self.get_dependency_set_from_regionID(nodes[1])
        regionID2nodes_dict[self.region_dict[self.pose_locations[key[0]]]].append(1)
        queue = [1]
        n_path_found = 0
        while (len(queue) > 0) and (n_path_found < numObjs):
            curr_node = queue.pop()
            if nodes[curr_node] in self.LL:  # if it has neighbors
                for neighbor_regionID in self.LL[nodes[curr_node]]:
                    neighbor_dependency_set = node_dependency_set_dict[curr_node].union(
                        self.get_dependency_set_from_regionID(neighbor_regionID)
                    )
                    ### check if this set is a superset or not
                    Abandoned = False
                    remove_list = []
                    for visitor_node in regionID2nodes_dict[neighbor_regionID]:
                        if neighbor_dependency_set.issuperset(node_dependency_set_dict[visitor_node]):
                            Abandoned = True
                            break
                        if node_dependency_set_dict[visitor_node].issuperset(neighbor_dependency_set):
                            remove_list.append(visitor_node)
                    for visitor_node in remove_list:
                        regionID2nodes_dict[neighbor_regionID].remove(visitor_node)
                        n_path_found -= 1
                    if not Abandoned:
                        nodeID = len(nodes) + 1
                        nodes[nodeID] = neighbor_regionID
                        parents[nodeID] = curr_node
                        regionID2nodes_dict[neighbor_regionID].append(nodeID)
                        queue.append(nodeID)
                        node_dependency_set_dict[nodeID] = neighbor_dependency_set

                        ### if the neighbor region is the goal region, a path is found
                        if neighbor_regionID == self.region_dict[self.pose_locations[key[1]]]:
                            ### increment the number of paths found
                            n_path_found += 1
                            if n_path_found >= numObjs:
                                break

        # print(node_dependency_set_dict[nodeID])

        goal_nodes = regionID2nodes_dict[self.region_dict[self.pose_locations[key[1]]]]

        for goal_node in goal_nodes:
            current_node = goal_node
            path = []
            while current_node in parents:  # while it is not the root(start pose).
                path.append(nodes[current_node])
                current_node = parents[current_node]
            path.append(nodes[current_node])
            # In this model we should keep the start and goal poses in the dependency graph, because they could be occupied
            # node_dependency_set_dict[goal_node] = node_dependency_set_dict[goal_node].difference({key[0], key[1]})
            self.path_dict[key].append(list(reversed(path)))
            self.dependency_dict[key].append(node_dependency_set_dict[goal_node])
            print(node_dependency_set_dict[goal_node])





    def get_dependency_set_from_regionID(self, regionID):
        for key, value in self.region_dict.items():
            if value == regionID:
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




######################################################### old codes kept for legacy ################################################################
    # def construct_path_dict(self):
    #     for i in range(len(self.poses)):
    #         key1 = self.poses[i]
    #         for j in range(i,len(self.poses)):
    #             if i == j:
    #                 # if j%2:
    #                 #     self.dependency_dict[(key1, key1)] = [self.get_dependency_set_from_regionID(self.region_dict[self.pose_locations[key1]])]
    #                 #     self.path_dict[(key1, key1)] = [[self.region_dict[self.pose_locations[key1]], self.region_dict[self.pose_locations[key1]]]]
    #                 self.dependency_dict[(key1, key1)] = [self.get_dependency_set_from_regionID(self.region_dict[self.pose_locations[key1]])]
    #                 self.path_dict[(key1, key1)] = [[self.region_dict[self.pose_locations[key1]]]]
    #                 continue
    #             key2 = self.poses[j]
    #             key = (key1, key2) ### key1 <= key2
    #             self.dependency_set_pruning_search(key)


    # def dependency_dict_conversion(self):
    #     ### This function does some post-processing about the dependency list
    #     ### to make sure for each dependency set of any (key1, key2)
    #     ### there are no duplicate elements
    #     for key in self.dependency_dict.keys():
    #         number_set_list = self.dependency_dict[key]
    #         pose_set_list = []
    #         for number_set in number_set_list:
    #             pose_set = set()
    #             for number in number_set:
    #                 # pose_set = pose_set.union({(number // 2, number % 2)})
    #                 # if number != key[0] and number != key[1]:
    #                 pose_set = pose_set.union({number})
    #             pose_set_list.append(pose_set)
    #         self.dependency_dict[key] = pose_set_list


    # def get_path_dependency_set(self, node, parents, nodes, key):
    #     current_node = node
    #     path = []
    #     dependency_set = set()
    #     while current_node in parents:  # while it is not the root(start pose).
    #         path.append(nodes[current_node])
    #         dependency_set = dependency_set.union(self.get_dependency_set_from_regionID(nodes[current_node]))
    #         current_node = parents[current_node]
    #     path.append(current_node)
    #     dependency_set = dependency_set.union(self.get_dependency_set_from_regionID(nodes[current_node]))
    #     dependency_set = dependency_set.difference({2 * key, 2 * key + 1})
    #     self.path_dict[key].append(list(reversed(path)))
    #     self.dependency_dict[key].append(dependency_set)