from __future__ import division
### This file is used for test an existing instance generated before
### for other methods


import os
import sys
import IPython
import shutil
import math
from Experiment_singleTest import Experiment_singleTest

if __name__ == "__main__":
    ### specify what instance do you want to use for test
    if (len(sys.argv) < 6):
        print("Please enter at least 6 essential inputs in order: \
                <numObjs> <RAD> <HEIGHT> <WIDTH> <nBuffers> <instance_index>")
        exit()
    try:
        numObjs = int(sys.argv[1])
        RAD = int(sys.argv[2])
        HEIGHT = int(sys.argv[3])
        WIDTH = int(sys.argv[4])
        nBuffers = int(sys.argv[5])
        assert(nBuffers >= 0)
        assert(nBuffers <= 3)
        instance_id = int(sys.argv[6])
    except ValueError:
        print("Please enter at least 6 essential inputs in order: \
                <numObjs> <RAD> <HEIGHT> <WIDTH> <nBuffers> <instance_index>")
        exit()

    flag = str(numObjs) + "_" + str(RAD) + "_" + str(HEIGHT) + "_" + str(WIDTH)

    if nBuffers == 0:
        instance_dir = os.path.join(os.getcwd(), "test", flag, "zero_buffer", str(instance_id))
    if nBuffers == 1:
        instance_dir = os.path.join(os.getcwd(), "test", flag, "one_buffer", str(instance_id))
    if nBuffers == 2:
        instance_dir = os.path.join(os.getcwd(), "test", flag, "two_buffer", str(instance_id))
    if nBuffers == 3:
        instance_dir = os.path.join(os.getcwd(), "test", flag, "unsolvable", str(instance_id))

    ### Now deal with other binary switch variables for file management or visualization (by default they are disabled)
    display = False
    if (len(sys.argv) > 7):
        display = sys.argv[7] not in ('n', 'N')

    displayMore = False
    if (len(sys.argv) > 8):
        displayMore = sys.argv[8] not in ('n', 'N')

    displayAnimation = False
    if (len(sys.argv) > 9):
        displayAnimation = sys.argv[9] not in ('n', 'N')

    savefile = False
    if (len(sys.argv) > 10):
        savefile = sys.argv[10] not in ('n', 'N')

    saveimage = False
    if (len(sys.argv) > 11):
        saveimage = sys.argv[11] not in ('n', 'N')

    savestat = False
    if (len(sys.argv) > 12):
        savestat = sys.argv[12] not in ('n', 'N')

    EXP = Experiment_singleTest(
            numObjs, RAD, HEIGHT, WIDTH, \
            display, displayMore, displayAnimation, savefile, saveimage, savestat, \
            instance_dir)