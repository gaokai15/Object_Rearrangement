import sys
import glob
import time
import json
import signal
from multiprocessing import Pool, cpu_count
from collections import OrderedDict

from dspace import DiskCSpace, DiskCSpaceProgram
from DG_Space import DFS_Rec_for_Monotone_General, linked_list_conversion

Visualize = True


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


def updateJson(filename, jsonData):
    with open(filename) as f:
        data = json.load(f, object_pairs_hook=OrderedDict)
    data.update(jsonData)
    with open(filename, 'w') as f:
        json.dump(data, f, indent=2)


def label_isMonotone(filename):
    print(filename)
    space = DiskCSpace.from_json(filename)
    t0 = time.time()
    try:
        is_monotone = isMonotone(space)
    except Exception:
        is_monotone = "Error"
    comp_time = time.time() - t0
    data = OrderedDict([('is_monotone', is_monotone), ('computation_time', comp_time)])
    updateJson(filename, data)


def label_challenge1(directory):
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
    try:
        pool.map_async(label_isMonotone, glob.glob(directory + '/*/*/*.json')).get(60)
    except KeyboardInterrupt:
        print('\nQuitting!')
        pool.terminate()
        sys.exit(0)
    else:
        print('Done!')
        pool.close()
    pool.join()


def prettyfy(directory):
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
    label_challenge1(sys.argv[1])
    # label_isMonotone(sys.argv[1])
    # prettyfy(sys.argv[1])
    sys.exit(0)

    space = DiskCSpace.from_json(sys.argv[1])

    space.regionGraph()
    # print(space.poseMap)
    print(isMonotone(space))

    if Visualize:
        program = DiskCSpaceProgram(space)
        program.view.w = program.view.h = 1080
        program.name = "Motion planning test"
        program.run()
