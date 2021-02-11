from __future__ import print_function

import sys
import glob
import time
import json
import signal
from collections import OrderedDict
from multiprocessing import Pool, cpu_count, TimeoutError

from dspace import DiskCSpace, DiskCSpaceProgram
from DG_Space import DFS_Rec_for_Monotone_General, linked_list_conversion

Visualize = False
TIMEOUT = 3600  # Timeout in seconds


def isMonotone(space, ignored=set()):
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


def updateJson(filename, jsonData):
    with open(filename) as f:
        data = json.load(f, object_pairs_hook=OrderedDict)
    # print(data, jsonData, filename)
    data.update(jsonData)
    # print(data)
    with open(filename, 'w') as f:
        json.dump(data, f, indent=2)


class KeyboardInterruptError(Exception):
    pass


def label_isMonotone(filename):
    space = DiskCSpace.from_json(filename)
    t0 = time.clock()
    try:
        is_monotone = isMonotone(space)
    except Exception:
        is_monotone = "Error"
    comp_time = time.clock() - t0
    data = (('is_monotone', is_monotone), ('computation_time', comp_time))
    print(filename, comp_time)
    return filename, data


def label_perturbable(filename):
    space = DiskCSpace.from_json(filename)
    objIsPert = {}
    t0 = time.clock()
    for pobj in filter(lambda x: x[0] == 'S', space.poseMap.keys()):
        obj = int(str(pobj).strip('ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz'))
        try:
            is_monotone = isMonotone(space, set([obj]))
        except Exception:
            is_monotone = "Error"
        objIsPert[obj] = is_monotone
    comp_time = time.clock() - t0
    data = (('is_perturbable', objIsPert), ('computation_time', comp_time))
    print(filename, comp_time)
    return filename, data


def label_challenge(directory, function, on_timeout, start_index=None, end_index=None):
    print(directory + '/*/*/*.json')
    # for filename in sorted(glob.glob(directory + '/*/*/*.json')):
    #     D, n, trial = filename.split('/')[2:]
    #     D = float(D.split('=')[-1])
    #     n = int(n.split('=')[-1])
    #     trial = int(trial.split('.')[0])
    #     print(D, n, trial)
    #     label_isMonotone(filename)
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
                result.get(TIMEOUT)
            except TimeoutError:
                print('\nTimeout!')
                updateJson(filename, on_timeout)
                print(filename, TIMEOUT)
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


if __name__ == "__main__":
    # clean(sys.argv[1])
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
                ('computation_time', TIMEOUT),
            ), si, ei
        )
    elif sys.argv[1][-1] == '2':
        ### Challenge 2 ###
        # print(label_perturbable(sys.argv[1]))
        label_challenge(
            sys.argv[1], label_perturbable, (
                ('is_perturbable', "Timeout"),
                ('computation_time', TIMEOUT),
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