from __future__ import division, print_function

import os
import sys
import time
import copy
import numpy as np
from random import sample

from BiDirDPPlanner import BiDirDPPlanner
from DG_Space import linked_list_conversion


class Experiments(object):
    def single_instance(self, space):
        self.space = space
        self.graph = space.RGAdj
        self.object_locations = space.pose2reg

        start_poses = {}
        goal_poses = {}
        self.initial_arrangement = []
        self.final_arrangement = []
        for pid in space.poseMap:
            dd = str(pid).strip('ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz')
            sd = str(pid).strip('0123456789')
            if 'S' in sd:
                start_poses[int(dd)] = pid
            elif 'G' in sd:
                goal_poses[int(dd)] = pid

        assert (len(start_poses) == len(goal_poses))

        ### index the arrangement
        self.initial_arrangement = [start_poses[i] for i in range(len(start_poses))]
        self.final_arrangement = [goal_poses[i] for i in range(len(goal_poses))]
        print("initial_arrangement: " + str(self.initial_arrangement))
        print("final_arrangement: " + str(self.final_arrangement))

        region_dict, linked_list = linked_list_conversion(graph)

        start_time = time.clock()
        self.plan_DP_local = BiDirDPPlanner(
            self.initial_arrangement,
            self.final_arrangement,
            self.space,
            region_dict,
            linked_list,
        )
        self.comTime_DP_local = time.clock() - start_time
        print("Time to perform BiDirectional search with DP local solver: " + str(self.comTime_DP_local))

        if self.plan_DP_local.isConnected == False:
            ### the solution is not found
            self.totalActions_DP_local = -1
        else:
            self.totalActions_DP_local = self.plan_DP_local.totalActions


if __name__ == "__main__":
    space = None
    numObjs = 5
    rad = 50
    height = 1000
    width = 1000
    if len(sys.argv) > 1:
        if sys.argv[1].isdigit():
            numObjs = int(sys.argv[1])
        else:
            space = loadEnv(sys.argv[1])

    if len(sys.argv) > 2:
        if space is None:
            rad = int(sys.argv[2])
        else:
            numObjs = int(sys.argv[2])

    if len(sys.argv) > 3:
        height = int(sys.argv[3])

    if len(sys.argv) > 4:
        width = int(sys.argv[4])

    if space is None:
        space = DiskCSpace(rad, {}, [], height, width)

    if len(space.poseMap) == 0:
        genPoses(numObjs, space)

    space.regionGraph()
    genBuffers(5, space, 4)
    space.regionGraph()
    # print(space.RGAdj.keys())

    # DGs = DG_Space(path_dict)
    # print(DGs.DGs)
    # IP = feekback_arc_ILP(path_dict)
    EXP = Experiments()
    set_max_memory(1.3 * 2**(34))  #2**34=16G

    EXP.single_instance(space)
