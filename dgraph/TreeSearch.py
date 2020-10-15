from __future__ import division, print_function

import os
import sys
import copy
# import numpy as np
from time import clock
from random import sample

from dspace import *
from DG_Space import set_max_memory

# from BiDirDPPlanner import BiDirDPPlanner as Planner
# from BiDirDPPlanner_dyn import BiDirDPPlanner as Planner
from BiDirDPPlanner_dyn_rand import BiDirDPPlanner as Planner

# from FastHeuristicDPPlanner import FastHeuristicDPPlanner as Planner
# from FastHeuristicDPPlanner_dyn import FastHeuristicDPPlanner as Planner

# VISUALIZE = False
VISUALIZE = True
num_buffers = 5


class Experiments(object):
    def single_instance(self, space, visualize):
        self.space = space

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

        start_time = clock()
        self.plan_DP_local = Planner(self.initial_arrangement, self.final_arrangement, self.space)
        self.comTime_DP_local = clock() - start_time
        print("Time to perform BiDirectional search with DP local solver: " + str(self.comTime_DP_local))

        self.solution = None
        self.arrangements = None
        self.totalActions = -1
        self.best_solution_cost = -1
        if self.plan_DP_local.isConnected:
            self.plan_DP_local.getTheStat()
            self.solution = self.plan_DP_local.solution
            self.arrangements = self.plan_DP_local.arrangements
            self.totalActions = self.plan_DP_local.totalActions
            self.best_solution_cost = self.plan_DP_local.best_solution_cost

            print("\nsolution path: " + str(self.solution))
            print("arrangements: " + str(self.arrangements))
            print("total action: " + str(self.totalActions))
            print("solution cost: " + str(self.best_solution_cost))
        else:
            print("failed to find a solution within " + str(self.plan_DP_local.totalTime_allowed) + " seconds...")

        if visualize:
            program = DiskCSpaceProgram(self.space, self.solution, self.arrangements)
            program.view.w = program.view.h = 1080
            program.name = "Motion planning test"
            program.run()
        return self.totalActions, self.comTime_DP_local


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
            rad = space.robot.radius
            height = space.bound[1][1]
            width = space.bound[0][1]

    if len(sys.argv) > 2:
        if space is None:
            rad = int(sys.argv[2])
        else:
            numObjs = int(sys.argv[2])

    if len(sys.argv) > 3:
        if space is None:
            height = int(sys.argv[3])
        else:
            rad = int(sys.argv[3])

    if len(sys.argv) > 4:
        width = int(sys.argv[4])

    if space is None:
        space = DiskCSpace(rad, {}, [], height, width)
    else:
        space.setRobotRad(rad)

    if len(space.poseMap) == 0:
        genPoses(numObjs, space)

    space.regionGraph()
    if num_buffers > 0:
        genBuffers(num_buffers, space, space.poseMap.keys(), 'random', 50)
        # genBuffers(num_buffers, space, space.poseMap.keys(), 'greedy_free')
        # genBuffers(num_buffers, space, space.poseMap.keys(), 'boundary_random')
        # genBuffers(num_buffers, space, filter(lambda x: x[0] == 'S', space.poseMap.keys()), 'object_feasible', 0, [1, 2, 0, 3, 4])
        space.regionGraph()

    EXP = Experiments()
    set_max_memory(1.3 * 2**(34))  #2**34=16G

    EXP.single_instance(space, VISUALIZE)

    outfile = sys.stderr
    if len(sys.argv) > 5:
        outfile = open(sys.argv[5], 'w')

    print(
        """DiskCSpace(
    rad={},
    height={},
    width={},""".format(
            rad,
            height,
            width,
        ),
        file=outfile,
    )
    print('    obstacles=[', file=outfile)
    for x in space.obstacles:
        print('        ', x, ',', sep='', file=outfile)
    print('    ],', file=outfile)

    print('    poseMap={', file=outfile)
    for k, v in space.poseMap.items():
        print("        '", k, "': ", v, ',', sep='', file=outfile)
    print('    },\n)', file=outfile)

    if outfile is not sys.stderr:
        outfile.close()
