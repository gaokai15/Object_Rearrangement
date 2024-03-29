from __future__ import division, print_function

import os
import sys
from glob import glob
from random import seed

from dspace import *
# from DG_Space import Experiments
from DG_Space import set_max_memory, DFS_Rec_for_Monotone_General, linked_list_conversion
from TreeSearch import Experiments

num_buffers = 100


def isMonotone(space):
    space.regionGraph()
    start_poses = {}
    goal_poses = {}
    for pid in space.poseMap:
        dd = str(pid).strip('ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz')
        sd = str(pid).strip('0123456789')
        if 'S' in sd:
            start_poses[int(dd)] = pid
        elif 'G' in sd:
            goal_poses[int(dd)] = pid

    region_dict, linked_list = linked_list_conversion(space.RGAdj)
    object_locations = space.pose2reg
    subTree = DFS_Rec_for_Monotone_General(
        start_poses,
        goal_poses,
        {},
        {},
        object_locations,
        linked_list,
        region_dict,
    )
    return subTree.isMonotone


if __name__ == "__main__":

    env_folder = None
    try:
        nExperiments = int(sys.argv[1])
        assert (nExperiments > 0)
        env_folder = sys.argv[2]
        out_folder = sys.argv[3]
        print(
            "Creating ",
            nExperiments,
            " experiments in folder ",
            out_folder,
            " per environment in folder ",
            env_folder,
        )

    except Exception:
        try:
            tests_glob = sys.argv[1]
            out_file = sys.argv[2]
            print("Running tests matching: ", sys.argv[1], " and dumping stats in file ", out_file)
        except Exception:
            print(
                sys.argv[0], ' <#-experiments /or/ "tests_glob"> <environments_folder /or/ output_file> [output_folder]'
            )
            sys.exit(-1)

    if env_folder:
        # num_objs_rad = [
        #     [5, 99],
        #     [5, 127],
        #     [10, 70],
        #     [10, 90],
        #     [15, 57],
        #     [15, 73],
        #     [20, 49],
        #     [20, 63],
        # ]
        num_objs_dense = [
            [5, 0.35],
            [5, 0.45],
            [10, 0.35],
            [10, 0.45],
            [15, 0.35],
            [15, 0.45],
            [20, 0.35],
            [20, 0.45],
        ]
        base_envs = {'Empty': DiskCSpace()}
        for env_file in glob(env_folder + '/*'):
            name = env_file.split('/')[-1].split('.')[0]
            base_envs[name] = loadEnv(env_file)

        for name, space in base_envs.items():
            for params in num_objs_dense:
                if name == 'shelf5':
                    rad = 1000
                    space.setRobotRad(rad)
                    space.computeMinkObs()
                else:
                    space.setRobotRad(1)
                    space.computeMinkObs()
                if space.mink_obs.type == 'S_Poly':
                    area = pc.Area(space.mink_obs.points)
                elif space.mink_obs.type == 'C_Poly':
                    area = sum([pc.Area(x) for x in space.mink_obs.points])
                else:
                    print("WTF?")
                    sys.exit(-1)

                if name == 'shelf5':
                    nobj = int(ceil(area * params[1] / (2 * pi * rad**2)))
                else:
                    nobj = params[0]
                    rad = int(ceil(sqrt((area * params[1]) / (2 * nobj * pi))))
                    space.setRobotRad(rad)
                print(area, nobj, rad)
                for i in range(nExperiments):
                    space.clearPoses()
                    num_fail = 10
                    while num_fail > 0:
                        if genPoses(nobj, space):
                            if not isMonotone(space):
                                break
                        else:
                            num_fail -= 1
                    with open("{}/{}_{}_{}_{}.py".format(out_folder, name, nobj, rad, i), 'w') as outfile:
                        print(
                            'DiskCSpace(\n    rad={},\n    height={},\n    width={},\n'.format(
                                space.robot.radius,
                                space.bound[1][1],
                                space.bound[0][1],
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
    else:
        EXP = Experiments()
        set_max_memory(1.3 * 2**(34))  #2**34=16G
        with open(out_file, 'w') as outfile:
            print('name, num_objs, radius, trial, actions, time, iterations', file=outfile)
            for env_file in glob(tests_glob):
                name = env_file.split('/')[-1].split('.')[0]
                space = loadEnv(env_file)
                space.regionGraph()
                if num_buffers > 0:
                    seed(env_file)
                    genBuffers(num_buffers, space, space.poseMap.keys(), 'random', 1)
                    # genBuffers(num_buffers, space, space.poseMap.keys(), 'greedy_free')
                    # genBuffers(num_buffers, space, space.poseMap.keys(), 'greedy_boundary')
                    # genBuffers(num_buffers, space, [], 'greedy_free')
                    # genBuffers(num_buffers, space, [], 'greedy_boundary')
                    # genBuffers(num_buffers, space, [], 'boundary_random', 50)
                    # genBuffers(num_buffers, space, space.poseMap.keys(), 'boundary_random')
                    space.regionGraph()
                try:
                    actions, runtime, iters = EXP.single_instance(space, False)
                except Exception as e:
                    actions = -1
                    runtime = -1
                    iters = repr(e).replace(',', '.')

                print(','.join(name.split('_') + [str(actions), str(runtime), str(iters)]), file=outfile)
