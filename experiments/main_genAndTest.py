from __future__ import division
### This file is used for generate an instance and quickly test on it

import os
import sys
import IPython
import shutil
import math
from Experiment_genAndTest import Experiment_genAndTest

if __name__ == "__main__":
    if (len(sys.argv) < 3):
        print("Please enter at least 3 essential inputs in order: \
                <numObjs> <RAD> <workspace_density>")
        exit()
    try:
        numObjs = int(sys.argv[1])
        RAD = int(sys.argv[2])
        workspace_density = float(sys.argv[3])

    except ValueError:
        print("Please enter at least 3 essential inputs in order: \
                <numObjs> <RAD> <workspace_density>")
        exit()

    HEIGHT = int(math.sqrt(2*numObjs*math.pi*math.pow(RAD, 2)/workspace_density))
    WIDTH = int(math.sqrt(2*numObjs*math.pi*math.pow(RAD, 2)/workspace_density))    

    display = False
    if (len(sys.argv) > 4):
        display = sys.argv[4] not in ('n', 'N')

    displayMore = False
    if (len(sys.argv) > 5):
        displayMore = sys.argv[5] not in ('n', 'N')

    displayAnimation = False
    if (len(sys.argv) > 6):
        displayAnimation = sys.argv[6] not in ('n', 'N')

    savefile = False
    if (len(sys.argv) > 7):
        savefile = sys.argv[7] not in ('n', 'N')

    saveimage = False
    if (len(sys.argv) > 8):
        saveimage = sys.argv[8] not in ('n', 'N')

    savestat = False
    if (len(sys.argv) > 9):
        savestat = sys.argv[9] not in ('n', 'N')

    flag = str(numObjs) + "_" + str(RAD) + "_" + str(HEIGHT) + "_" + str(WIDTH)
    instance_dir = os.path.join(os.getcwd(), "instances", flag)

    EXP = Experiment_genAndTest(
        numObjs, RAD, HEIGHT, WIDTH, \
        display, displayMore, displayAnimation, savefile, saveimage, savestat,\
        instance_dir)