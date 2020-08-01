from __future__ import division


class DensePathGenerator(object):
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