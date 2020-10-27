from __future__ import division
### This file is used for generating instances
### save instances and optimal solutions from brute force methods

import os
import sys
import IPython
import shutil
import math
from Experiment_genInstances import Experiment_genInstances

#######################################################################################################
def initializeStat(exp_path):
    ### this function initialize the statistics for a given experimental setting
    ### Input: exp_path (experiment setting)
    ### Ouput: (instances_id, )

    zeroBuffer_instances_path = os.path.join(exp_path, "zero_buffer")
    oneBuffer_instances_path = os.path.join(exp_path, "one_buffer")
    twoBuffer_instances_path = os.path.join(exp_path, "two_buffer")
    unsolvable_instances_path = os.path.join(exp_path, "unsolvable")

    if not os.path.exists(exp_path):
        ### If this experiment setting is not used before, create a new directory for it
        os.mkdir(exp_path)
        os.mkdir(zeroBuffer_instances_path)
        os.mkdir(oneBuffer_instances_path)
        os.mkdir(twoBuffer_instances_path)
        os.mkdir(unsolvable_instances_path)
        f_launchstat = open(zeroBuffer_instances_path + "/BF_stat.txt", "w")
        f_launchstat.write("0" + "\n" + "0" + "\n" + "0" + "\n" + "0" + "\n" + "0")
        f_launchstat.close()
        f_launchstat = open(oneBuffer_instances_path + "/BF_stat.txt", "w")
        f_launchstat.write("0" + "\n" + "0" + "\n" + "0" + "\n" + "0" + "\n" + "0")
        f_launchstat.close()
        f_launchstat = open(twoBuffer_instances_path + "/BF_stat.txt", "w")
        f_launchstat.write("0" + "\n" + "0" + "\n" + "0" + "\n" + "0" + "\n" + "0")
        f_launchstat.close()
        f_launchstat = open(unsolvable_instances_path + "/BF_stat.txt", "w")
        f_launchstat.write("0" + "\n" + "0" + "\n" + "0" + "\n" + "0" + "\n" + "0")
        f_launchstat.close()
        instances_zeroBuffer_id = 0
        instances_oneBuffer_id = 0
        instances_twoBuffer_id = 0
        instances_unsolvable_id = 0
        instances_zeroBuffer_additionalActions = 0
        instances_oneBuffer_additionalActions = 0
        instances_twoBuffer_additionalActions = 0
        instances_unsolvable_additionalActions = 0
        instances_zeroBuffer_totalTime = 0
        instances_oneBuffer_totalTime = 0
        instances_twoBuffer_totalTime = 0
        instances_unsolvable_totalTime = 0
    else:
        ### If this experiment setting is used before, create a new directory for it
        ### Then read in the parameters
        f_launchstat = open(zeroBuffer_instances_path + "/BF_stat.txt", "r")
        counter = 1
        for line in f_launchstat:
            line = line.split()
            if (counter == 1):
                instances_zeroBuffer_id = int(line[0])
            if (counter == 2):
                instances_zeroBuffer_additionalActions = int(line[0])
            if (counter == 3):
                instances_zeroBuffer_totalTime = float(line[0])
            if (counter > 3):
                break
            counter += 1
        f_launchstat.close()

        f_launchstat = open(oneBuffer_instances_path + "/BF_stat.txt", "r")
        counter = 1
        for line in f_launchstat:
            line = line.split()
            if (counter == 1):
                instances_oneBuffer_id = int(line[0])
            if (counter == 2):
                instances_oneBuffer_additionalActions = int(line[0])
            if (counter == 3):
                instances_oneBuffer_totalTime = float(line[0])
            if (counter > 3):
                break
            counter += 1
        f_launchstat.close()

        f_launchstat = open(twoBuffer_instances_path + "/BF_stat.txt", "r")
        counter = 1
        for line in f_launchstat:
            line = line.split()
            if (counter == 1):
                instances_twoBuffer_id = int(line[0])
            if (counter == 2):
                instances_twoBuffer_additionalActions = int(line[0])
            if (counter == 3):
                instances_twoBuffer_totalTime = float(line[0])
            if (counter > 3):
                break
            counter += 1
        f_launchstat.close()


        f_launchstat = open(unsolvable_instances_path + "/BF_stat.txt", "r")
        counter = 1
        for line in f_launchstat: 
            line = line.split()
            if (counter == 1):
                instances_unsolvable_id = int(line[0])
            if (counter == 2):
                instances_unsolvable_additionalActions = int(line[0])
            if (counter == 3):
                instances_unsolvable_totalTime = float(line[0])
            if (counter > 3):
                break
            counter += 1
        f_launchstat.close()


    return instances_zeroBuffer_id, instances_oneBuffer_id, instances_twoBuffer_id, instances_unsolvable_id, \
          instances_zeroBuffer_additionalActions, instances_oneBuffer_additionalActions, instances_twoBuffer_additionalActions, instances_unsolvable_additionalActions, \
          instances_zeroBuffer_totalTime, instances_oneBuffer_totalTime, instances_twoBuffer_totalTime, instances_unsolvable_totalTime


def updateStat(exp_path, instances_zeroBuffer_id, instances_oneBuffer_id, instances_twoBuffer_id, instances_unsolvable_id, \
        instances_zeroBuffer_additionalActions, instances_oneBuffer_additionalActions, instances_twoBuffer_additionalActions, instances_unsolvable_additionalActions, \
        instances_zeroBuffer_totalTime, instances_oneBuffer_totalTime, instances_twoBuffer_totalTime, instances_unsolvable_totalTime):

    zeroBuffer_instances_path = os.path.join(exp_path, "zero_buffer")
    f_launchstat = open(zeroBuffer_instances_path + "/BF_stat.txt", "w")
    f_launchstat.write(str(instances_zeroBuffer_id) + "\n")
    f_launchstat.write(str(instances_zeroBuffer_additionalActions) + "\n")
    f_launchstat.write(str(instances_zeroBuffer_totalTime) + "\n")
    if (instances_zeroBuffer_id == 0):
        f_launchstat.write("0" + "\n" + "0" + "\n")
    else:
        f_launchstat.write(str(instances_zeroBuffer_additionalActions / instances_zeroBuffer_id) + "\n")
        f_launchstat.write(str(instances_zeroBuffer_totalTime / instances_zeroBuffer_id) + "\n")
    f_launchstat.close()

    oneBuffer_instances_path = os.path.join(exp_path, "one_buffer")
    f_launchstat = open(oneBuffer_instances_path + "/BF_stat.txt", "w")
    f_launchstat.write(str(instances_oneBuffer_id) + "\n")
    f_launchstat.write(str(instances_oneBuffer_additionalActions) + "\n")
    f_launchstat.write(str(instances_oneBuffer_totalTime) + "\n")
    if (instances_oneBuffer_id == 0):
        f_launchstat.write("0" + "\n" + "0" + "\n")
    else:
        f_launchstat.write(str(instances_oneBuffer_additionalActions / instances_oneBuffer_id) + "\n")
        f_launchstat.write(str(instances_oneBuffer_totalTime / instances_oneBuffer_id) + "\n")
    f_launchstat.close()

    twoBuffer_instances_path = os.path.join(exp_path, "two_buffer")
    f_launchstat = open(twoBuffer_instances_path + "/BF_stat.txt", "w")
    f_launchstat.write(str(instances_twoBuffer_id) + "\n")
    f_launchstat.write(str(instances_twoBuffer_additionalActions) + "\n")
    f_launchstat.write(str(instances_twoBuffer_totalTime) + "\n")
    if (instances_twoBuffer_id == 0):
        f_launchstat.write("0" + "\n" + "0" + "\n")
    else:
        f_launchstat.write(str(instances_twoBuffer_additionalActions / instances_twoBuffer_id) + "\n")
        f_launchstat.write(str(instances_twoBuffer_totalTime / instances_twoBuffer_id) + "\n")
    f_launchstat.close()

    unsolvable_instances_path = os.path.join(exp_path, "unsolvable")
    f_launchstat = open(unsolvable_instances_path + "/BF_stat.txt", "w")
    f_launchstat.write(str(instances_unsolvable_id) + "\n")
    f_launchstat.write(str(instances_unsolvable_additionalActions) + "\n")
    f_launchstat.write(str(instances_unsolvable_totalTime) + "\n")
    if (instances_unsolvable_id == 0):
        f_launchstat.write("0" + "\n" + "0" + "\n")
    else:
        f_launchstat.write(str(instances_unsolvable_additionalActions / instances_unsolvable_id) + "\n")
        f_launchstat.write(str(instances_unsolvable_totalTime / instances_unsolvable_id) + "\n")
    f_launchstat.close()

################################################################################################################

if __name__ == "__main__":
    
    ### specify other variables
    display = False
    displayMore = False
    displayAnimation = False
    savefile = True
    saveimage = True
    savestat = True

    # nobjs_set = [5, 8, 10, 13, 15, 18, 20]
    # nobjs_set = [10, 13]
    nobjs_set = [10, 13, 15, 18, 20]

    ### load the argument inputs (radius, density)
    if (len(sys.argv) < 3):
        print("Please enter at least two essential inputs in order: <radius> <density> <#experiments>")
        exit()
    try:
        RAD = int(sys.argv[1])
        workspace_density = float(sys.argv[2])
        nExperiments = int(sys.argv[3])
        assert(nExperiments > 0)
    except ValueError:
        print("Please enter at least two essential inputs in order: <radius> <density> <#experiments>")
        exit()

    ### given the nobjs_set, argument inputs (radius, density), 
    ### get the combinations of parameters used in the mega experiments
    parameters_combination = []
    for nobjs in nobjs_set:
        temp_parameters = []
        temp_parameters.append(nobjs)
        temp_parameters.append(RAD)
        HEIGHT = int(math.sqrt(2*nobjs*math.pi*math.pow(RAD, 2)/workspace_density))
        WIDTH = int(math.sqrt(2*nobjs*math.pi*math.pow(RAD, 2)/workspace_density))
        temp_parameters.append(HEIGHT)
        temp_parameters.append(WIDTH)
        parameters_combination.append(temp_parameters)

    ###################################################################################################################
    for paras in parameters_combination:
        numObjs = paras[0]
        RAD = paras[1]
        HEIGHT = paras[2]
        WIDTH = paras[3]
        print("\nnumObjs: " + str(numObjs) + ", RAD: " + str(RAD) + ", HEIGHT: " + str(HEIGHT) + ", WIDTH: " + str(WIDTH))
        flag = str(numObjs) + "_" + str(int(RAD)) + "_" + str(HEIGHT) + "_" + str(WIDTH)
        ### directory generation
        exp_path = os.path.join(os.getcwd(), "instances", flag)
        instances_zeroBuffer_id, instances_oneBuffer_id, instances_twoBuffer_id, instances_unsolvable_id, \
        instances_zeroBuffer_additionalActions, instances_oneBuffer_additionalActions, instances_twoBuffer_additionalActions, instances_unsolvable_additionalActions, \
        instances_zeroBuffer_totalTime, instances_oneBuffer_totalTime, instances_twoBuffer_totalTime, instances_unsolvable_totalTime = initializeStat(exp_path)

        ### Now start experiments
        exp_id = 0
        while (exp_id < nExperiments):
            print("\nstart experiment " + str(exp_id) + ": " + str(numObjs) + " " + \
                        str(int(RAD)) + " " + str(HEIGHT) + " " + str(WIDTH))
            EXP = Experiment_genInstances(
                    numObjs, RAD, HEIGHT, WIDTH, \
                    display, displayMore, displayAnimation, savefile, saveimage, savestat, \
                    instances_zeroBuffer_id, instances_oneBuffer_id, instances_twoBuffer_id, instances_unsolvable_id, \
                    instances_zeroBuffer_additionalActions, instances_oneBuffer_additionalActions, instances_twoBuffer_additionalActions, instances_unsolvable_additionalActions, \
                    instances_zeroBuffer_totalTime, instances_oneBuffer_totalTime, instances_twoBuffer_totalTime, instances_unsolvable_totalTime, \
                    exp_path)

            if EXP.genInstanceFailure:
                continue

            else:
                exp_id += 1
                instances_zeroBuffer_id = EXP.instances_zeroBuffer_id
                instances_oneBuffer_id = EXP.instances_oneBuffer_id
                instances_twoBuffer_id = EXP.instances_twoBuffer_id
                instances_unsolvable_id = EXP.instances_unsolvable_id
                instances_zeroBuffer_additionalActions = EXP.instances_zeroBuffer_additionalActions
                instances_oneBuffer_additionalActions = EXP.instances_oneBuffer_additionalActions
                instances_twoBuffer_additionalActions = EXP.instances_twoBuffer_additionalActions
                instances_unsolvable_additionalActions = EXP.instances_unsolvable_additionalActions
                instances_zeroBuffer_totalTime = EXP.instances_zeroBuffer_totalTime
                instances_oneBuffer_totalTime = EXP.instances_oneBuffer_totalTime
                instances_twoBuffer_totalTime = EXP.instances_twoBuffer_totalTime
                instances_unsolvable_totalTime = EXP.instances_unsolvable_totalTime

        ### at the end of experiments, update the statistic file for the current experimental setting
        updateStat(exp_path, instances_zeroBuffer_id, instances_oneBuffer_id, instances_twoBuffer_id, instances_unsolvable_id, \
            instances_zeroBuffer_additionalActions, instances_oneBuffer_additionalActions, instances_twoBuffer_additionalActions, instances_unsolvable_additionalActions, \
            instances_zeroBuffer_totalTime, instances_oneBuffer_totalTime, instances_twoBuffer_totalTime, instances_unsolvable_totalTime)


