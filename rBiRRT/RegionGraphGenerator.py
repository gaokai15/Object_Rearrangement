from __future__ import division
from rgraph import *


class RegionGraphGenerator(object):
    def __init__(self, instance, visualTool, wall_mink):
        self.epsilon = 2**-8
        self.wall_mink = wall_mink
        self.graph, self.regions, self.obj2reg = self.genRegionGraph(instance)
        if visualTool.display:
            self.paths = self.getDisplayPaths(instance)
        else:
            self.paths = {}

        ### After cgraph generation, let's see if we want to display or save it
        if visualTool.display:
            #     regLists = []
            #     for i, x in self.regions.items():
            #         regLists.append(x.to_list())
            #     ### region graph without connections
            visualTool.drawRegionGraph({}, [x.to_list() for x in self.regions.values()], label=False)
        #     ### region graph with connections
        # visualTool.drawRegionGraph(self.paths, regLists, label=False)
        #     ### connectivity graph
        #     visualTool.drawConGraph(self.paths, instance.points, instance.objects, instance.buffers)

    def genRegionGraph(self, instance):
        ### This function generates a graph with different regions, specifying neighboring regions
        all_points = instance.points + instance.buffer_points
        all_minkowskis = instance.minkowski_objs + instance.minkowski_buffers

        less_regions, polysum = objects2regions(all_minkowskis, self.wall_mink)
        graph, regions, point2regs = regions2graph(less_regions, self.wall_mink, polysum, all_points, 0)
        return graph, regions, point2regs

    def getDisplayPaths(self, instance):
        ### This function figures out the connectivity between different regions
        paths = {}

        for rind1, r1adj in self.graph.items():
            for rind2 in r1adj:
                r1 = self.regions[rind1]
                r2 = self.regions[rind2]
                path = regionPath(r1, r2)
                paths[(rind1, rind2)] = path

        return paths
