from __future__ import print_function

import sys
import glob
import time
import json
import signal
import timeout_decorator

from random import seed
from collections import OrderedDict
from multiprocessing import Pool, cpu_count, TimeoutError

from DG_Space import linked_list_conversion
from dspace import DiskCSpace, DiskCSpaceProgram, genBuffers
from DPLocalSolver import DFS_Rec_for_Monotone_General, DFS_Rec_for_Non_Monotone_General

Visualize = False
TIMEOUT = 600  # Timeout in seconds


@timeout_decorator.timeout(TIMEOUT)
def isMonotone(space, ignored=set()):
    try:
        space.regionGraph()
        start_poses = {}
        goal_poses = {}
        for pid in space.poseMap:
            dd = int(str(pid).strip('ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz'))
            sd = str(pid).strip('0123456789')
            if dd not in ignored:
                if 'S' in sd:
                    start_poses[dd] = pid
                elif 'G' in sd:
                    goal_poses[dd] = pid

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
    except timeout_decorator.timeout_decorator.TimeoutError:
        return 'Timeout'


@timeout_decorator.timeout(TIMEOUT)
def isSolution(space, obj, buff):
    try:
        space.regionGraph(lambda x: x[0] != 'B' or x == buff)  # ignore buffers except buff
        start_poses = {}
        goal_poses = {}
        for pid in space.poseMap:
            dd = int(str(pid).strip('ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz'))
            sd = str(pid).strip('0123456789')
            if 'S' in sd:
                start_poses[dd] = pid
            elif 'G' in sd:
                goal_poses[dd] = pid

        region_dict, linked_list = linked_list_conversion(space.RGAdj)
        object_locations = space.pose2reg
        subTree = DFS_Rec_for_Non_Monotone_General(
            start_poses,
            goal_poses,
            {},
            {},
            object_locations,
            linked_list,
            region_dict,
            {obj: (len(start_poses), buff)},
        )
        return subTree.isMonotone
    except timeout_decorator.timeout_decorator.TimeoutError:
        return 'Timeout'


def updateJson(filename, jsonData):
    with open(filename) as f:
        data = json.load(f, object_pairs_hook=OrderedDict)
    # print(data, jsonData, filename)
    data.update(jsonData)
    # print(data)
    with open(filename, 'w') as f:
        json.dump(data, f, indent=2)


# @timeout_decorator.timeout(TIMEOUT)
def label_isMonotone(filename):
    space = DiskCSpace.from_json(filename)
    t0 = time.clock()
    t1 = time.time()
    try:
        is_monotone = isMonotone(space)
    except Exception:
        is_monotone = "Error"
    comp_time = time.clock() - t0
    comp_time1 = time.time() - t1
    data = (
        ('is_monotone', is_monotone),
        ('computation_time', comp_time),
        # ('computation_time_wall', comp_time1),
    )
    print(filename, comp_time)
    return filename, data


# @timeout_decorator.timeout(TIMEOUT)
def label_perturbable(filename):
    space = DiskCSpace.from_json(filename)
    objIsPert = {}
    comp_time = {}
    comp_time1 = {}
    for pobj in filter(lambda x: x[0] == 'S', space.poseMap.keys()):
        obj = int(str(pobj).strip('ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz'))
        t0 = time.clock()
        t1 = time.time()
        try:
            is_monotone = isMonotone(space, set([obj]))
        except Exception:
            is_monotone = "Error"
        objIsPert[obj] = is_monotone
        comp_time[obj] = time.clock() - t0
        comp_time1[obj] = time.time() - t1
    data = (
        ('is_perturbable', objIsPert),
        ('computation_time', comp_time),
        # ('computation_time_wall', comp_time1),
    )
    print(filename, comp_time)
    return filename, data


# @timeout_decorator.timeout(TIMEOUT)
def label_buffers(filename):
    space = DiskCSpace.from_json(filename)
    num_buffers = 10
    space.computeMinkObs()
    seed(88)
    num_generated = genBuffers(num_buffers, space, space.poseMap.keys(), 'greedy_free')
    num_generated = genBuffers(
        num_buffers - num_generated,
        space,
        space.poseMap.keys(),
        'random',
        len(space.poseMap.keys()),
        count=num_generated
    )
    try:
        with open(filename) as f:
            # perturbed = sorted([int(x) for x, y in json.load(f)['is_perturbable'].items() if y])
            obj = int(json.load(f)['object_selected'])
    except:
        data = (('is_valid_buffer', 'Bad instance'), ('computation_time', -1))
        print(filename, -1)
        return filename, data
    bufferIsValid = {}
    buffer_coords = {}
    t0 = time.clock()
    t1 = time.time()
    for buff in filter(lambda x: x[0] == 'B', space.poseMap.keys()):
        buffer_coords[buff] = space.poseMap[buff].center
        try:
            is_buffer_valid = isSolution(space, obj, buff)
        except Exception:
            is_buffer_valid = "Error"
        # print(buff, obj, is_buffer_valid)
        bufferIsValid[buff] = is_buffer_valid
    comp_time = time.clock() - t0
    comp_time1 = time.time() - t1
    data = (
        ('buffers', buffer_coords),
        ('is_valid_buffer', bufferIsValid),
        ('computation_time', comp_time),
        # ('computation_time_wall', comp_time1),
    )
    print(filename, comp_time)
    return filename, data


def label_challenge(directory, function, on_timeout, start_index=None, end_index=None):
    print(directory + '/*/*/*.json')
    original_sigint_handler = signal.signal(signal.SIGINT, signal.SIG_IGN)
    pool = Pool(cpu_count())
    signal.signal(signal.SIGINT, original_sigint_handler)
    results = []
    for filename in sorted(glob.glob(directory + '/*/*/*.json'))[start_index:end_index]:

        def fileUpdate(filename_and_data):
            updateJson(*filename_and_data)

        results.append((filename, pool.apply_async(function, (filename, ), callback=fileUpdate)))
    try:
        for filename, result in results:
            try:
                result.get(TIMEOUT * 60)
            except TimeoutError:
                print('\nTimeout: ', filename, '\n')
                updateJson(filename, on_timeout)
            except Exception:
                print("Error?: ", filename)
    except KeyboardInterrupt:
        print('\nQuitting!')
        pool.terminate()
        sys.exit(0)
    print('Done!')
    pool.close()
    pool.join()


def clean(directory):
    for filename in sorted(glob.glob(directory + '/*/*/*/*.json')):
        print(filename)
        with open(filename) as f:
            data = json.load(f)
        data = OrderedDict(
            [
                ("n", data['n']),
                ("radius", data['radius']),
                ("height", data['height']),
                ("width", data['width']),
                ("starts", data['starts']),
                ("goals", data['goals']),
                ("obstacles", data['obstacles']),
            ]
        )
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)


def split_challenge2(directory):
    for filename in sorted(glob.glob(directory + '/*/*/*.json')):
        with open(filename) as f:
            data = json.load(f, object_pairs_hook=OrderedDict)

        c = 0
        if 'is_perturbable' not in data:
            continue
        if data['is_perturbable'] is None:
            continue
        perturb = data['is_perturbable']
        comp_times = data['computation_time']
        # comp_time1s = data['computation_time_wall']
        if type(perturb) is not OrderedDict:
            continue

        for obj, is_pert in perturb.items():
            if type(is_pert) == bool:
                newfile = filename[:-5] + '-' + obj + '.json'
                data['object_selected'] = int(obj)
                data['is_perturbable'] = is_pert
                data['computation_time'] = comp_times[obj] if type(comp_times) != float else comp_times
                # data['computation_time_wall'] = comp_time1s[obj] if type(comp_time1s) != float else comp_time1s
                with open(newfile, 'w') as f:
                    json.dump(data, f, indent=2)
                print(newfile, is_pert)


if __name__ == "__main__":

    ### Misc Processing ###
    # split_challenge2(sys.argv[1])
    # clean(sys.argv[1])
    # sys.exit(0)

    si = None
    ei = None
    print(len(sys.argv))
    if len(sys.argv) > 2:
        si = int(sys.argv[2])
    if len(sys.argv) > 3:
        ei = int(sys.argv[3])

    if sys.argv[1][-1] == '1':
        ### Challenge 1 ###
        # print(label_isMonotone(sys.argv[1]))
        label_challenge(
            sys.argv[1], label_isMonotone, (
                ('is_monotone', "Timeout"),
                ('computation_time', TIMEOUT * 60),
            ), si, ei
        )
    elif sys.argv[1][-1] == '2':
        ### Challenge 2 ###
        # print(label_perturbable(sys.argv[1]))
        label_challenge(
            sys.argv[1], label_perturbable, (
                ('is_perturbable', "Timeout"),
                ('computation_time', TIMEOUT * 60),
            ), si, ei
        )
    elif sys.argv[1][-1] == '3':
        ### Challenge 3 ###
        # print(label_buffers(sys.argv[1]))
        label_challenge(
            sys.argv[1], label_buffers, (
                ('is_valid_buffer', "Timeout"),
                ('computation_time', TIMEOUT * 60),
            ), si, ei
        )

    # sys.exit(0)

    ### Compute On One ###
    # space = DiskCSpace.from_json(sys.argv[1])
    # space.regionGraph()
    # print(isMonotone(space))

    # if Visualize:
    #     program = DiskCSpaceProgram(space)
    #     program.view.w = program.view.h = 1080
    #     program.name = "Motion planning test"
    #     program.run()
