from __future__ import division

import os
import sys
import IPython
import shutil
from Experiment import Experiment

if __name__ == "__main__":
    ### make sure it loads in the essential parameters (numObjs, RAD, HEIGHT, WIDTH)
    if (len(sys.argv) < 4):
        print("Please enter the 4 essential inputs in order: <#objects> <radius> <height> <width>")
        exit()
    try:
        numObjs = int(sys.argv[1])
        RAD = float(sys.argv[2])
        HEIGHT = int(sys.argv[3])
        WIDTH = int(sys.argv[4])
    except ValueError:
        print("Please enter the 4 essential inputs in order: <#objects> <radius> <height> <width>")
        exit()

    ### Now deal with other binary switch variables for file management or visualization (by default they are disabled)
    display = False
    if (len(sys.argv) > 5):
        display = sys.argv[5] not in ('n', 'N')

    displayMore = False
    if (len(sys.argv) > 6):
        displayMore = sys.argv[6] not in ('n', 'N')

    displayAnimation = False
    if (len(sys.argv) > 7):
        displayAnimation = sys.argv[7] not in ('n', 'N')

    savefile = False
    if (len(sys.argv) > 8):
        savefile = sys.argv[8] not in ('n', 'N')

    saveimage = False
    if (len(sys.argv) > 9):
        saveimage = sys.argv[9] not in ('n', 'N')

    savestat = False
    if (len(sys.argv) > 10):
        savestat = sys.argv[10] not in ('n', 'N')

    ###################################################################################################################
    nExperiments = int(sys.argv[11])
    assert(nExperiments > 0)
    print "Number of objects: ", numObjs
    print "Radius: ", RAD
    print "Environment size: ", HEIGHT, WIDTH
    print "\n"
    ### Once we have the parameters, we can create a folder to store the data with respect to this parameter set
    flag = str(numObjs) + "_" + str(int(RAD)) + "_" + str(HEIGHT) + "_" + str(WIDTH)
    data_path = os.path.join(os.getcwd(), "figures", flag)
    if os.path.exists(data_path):
        shutil.rmtree(data_path)
    os.mkdir(data_path)

    ### prepare the statistics
    total_successGenTimes = 0 ### record the number of times you successfully generate an instance

    total_successSolTimes_BF = 0 ### record the number of times you successfully find a solution
    average_comTime_BF = 0
    average_numActions_BF = 0

    total_successSolTimes_DP = 0 ### record the number of times you successfully find a solution
    average_comTime_DP = 0
    average_numActions_DP = 0

    # total_successSolTimes_DP_heuristic = 0 ### record the number of times you successfully find a solution
    # average_comTime_DP_heuristic = 0
    # average_numActions_DP_heuristic = 0

    total_successSolTimes_Fast_heuristic = 0 ### record the number of times you successfully find a solution
    average_comTime_Fast_heuristic = 0
    average_numActions_Fast_heuristic = 0

    for exp_id in range(1, nExperiments+1):
        print("\nstart experiment " + str(exp_id) + ": " + str(numObjs) + " " + str(int(RAD)) + " " + str(HEIGHT) + " " + str(WIDTH))
        EXP = Experiment(numObjs, RAD, HEIGHT, WIDTH, display, displayMore, displayAnimation, savefile, saveimage, savestat, exp_id, data_path)
        ### first check if the instance is correctly generated
        if EXP.genInstanceFailure == True: 
            continue
        else:
            total_successGenTimes += 1


        ########### method 0: brute force ############
        if EXP.genSolutionFailure_BF == True:
            pass
        else:
            ### record the stats
            total_successSolTimes_BF += 1
            average_comTime_BF += EXP.comTime_BF
            average_numActions_BF += EXP.totalActions_BF


        ########### method 1: DP ################
        if EXP.genSolutionFailure_DP == True:
            pass
        else:
            ### record the stats
            total_successSolTimes_DP += 1
            average_comTime_DP += EXP.comTime_DP
            average_numActions_DP += EXP.totalActions_DP


        # ######### method 2: DP with heuristic ##########
        # if EXP.genSolutionFailure_DP_heuristic == True:
        #     pass
        # else:
        #     ### record the stats
        #     total_successSolTimes_DP_heuristic += 1
        #     average_comTime_DP_heuristic += EXP.comTime_DP_heuristic
        #     average_numActions_DP_heuristic += EXP.totalActions_DP_heuristic


        ######### method 3: DP with fast heuristic ##########
        if EXP.genSolutionFailure_Fast_heuristic == True:
            pass
        else:
            ### record the stats
            total_successSolTimes_Fast_heuristic += 1
            average_comTime_Fast_heuristic += EXP.comTime_Fast_heuristic
            average_numActions_Fast_heuristic += EXP.totalActions_Fast_heuristic


    ### get the average
    average_comTime_BF = average_comTime_BF / total_successSolTimes_BF
    average_numActions_BF = average_numActions_BF / total_successSolTimes_BF
    average_comTime_DP = average_comTime_DP / total_successSolTimes_DP
    average_numActions_DP = average_numActions_DP / total_successSolTimes_DP
    # average_comTime_DP_heuristic = average_comTime_DP_heuristic / total_successSolTimes_DP_heuristic
    # average_numActions_DP_heuristic = average_numActions_DP_heuristic / total_successSolTimes_DP_heuristic
    average_comTime_Fast_heuristic = average_comTime_Fast_heuristic / total_successSolTimes_Fast_heuristic
    average_numActions_Fast_heuristic = average_numActions_Fast_heuristic / total_successSolTimes_Fast_heuristic
    print("______________________________________________________")
    print("success times for generating instances: " + str(total_successGenTimes) + "/" + str(nExperiments) + "\n")

    print("success times for having a solution for BF: " + str(total_successSolTimes_BF) + "/" + str(nExperiments))
    if (total_successSolTimes_BF != 0):
        print("average computation time for successful cases: " + str(average_comTime_BF))
        print("average number of actions: " + str(average_numActions_BF))

    print("success times for having a solution for DP: " + str(total_successSolTimes_DP) + "/" + str(nExperiments))
    if (total_successSolTimes_DP != 0):
        print("average computation time for successful cases: " + str(average_comTime_DP))
        print("average number of actions: " + str(average_numActions_DP))

    # print("success times for having a solution for DP with heuristic: " + str(total_successSolTimes_DP_heuristic) + "/" + str(nExperiments))
    # if (total_successSolTimes_DP_heuristic != 0):
    #     print("average computation time for successful cases: " + str(average_comTime_DP_heuristic))
    #     print("average number of actions: " + str(average_numActions_DP_heuristic))

    print("success times for having a solution for DP with fast heuristic: " + str(total_successSolTimes_Fast_heuristic) + "/" + str(nExperiments))
    if (total_successSolTimes_Fast_heuristic != 0):
        print("average computation time for successful cases: " + str(average_comTime_Fast_heuristic))
        print("average number of actions: " + str(average_numActions_Fast_heuristic))

