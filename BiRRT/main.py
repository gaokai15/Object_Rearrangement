from __future__ import division

import os
import sys
import IPython
import shutil
from Experiment import Experiment




if __name__ == "__main__":
    ### make sure it loads in the essential parameters (numObjs, RAD, HEIGHT, WIDTH)
    if (len(sys.argv) < 4):
        print("Please enter the 4 essential inputs in order: <#objects> <radius> <width> <radius>")
        exit()
    try:
        numObjs = int(sys.argv[1])
        RAD = float(sys.argv[2])
        HEIGHT = int(sys.argv[3])
        WIDTH = int(sys.argv[4])
    except ValueError:
        print("Please enter the 4 essential inputs in order: <#objects> <radius> <width> <radius>")
        exit()

    ### Now deal with other binary switch variables for file management or visualization (by default they are disabled)
    display = False
    if (len(sys.argv) > 5):
        display = sys.argv[5] not in ('n', 'N')
    displayMore = False
    if (len(sys.argv) > 6):
        displayMore = sys.argv[6] not in ('n', 'N')
    savefile = False
    if (len(sys.argv) > 7):
        savefile = sys.argv[7] not in ('n', 'N')
    saveimage = False
    if (len(sys.argv) > 8):
        saveimage = sys.argv[8] not in ('n', 'N')
    savestat = False
    if (len(sys.argv) > 9):
        savestat = sys.argv[9] not in ('n', 'N')
    ###################################################################################################################
    nExperiments = int(sys.argv[10])
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

    successGenTimes = 0 ### record the number of times you successfully generate an instance
    successSolTimes = 0 ### record the number of times you successfully find a solution
    comTime = 0
    numActions = 0
    successSolTimes_star = 0 ### record the number of times you successfully find a solution
    comTime_star = 0
    numActions_star = 0


    for exp_id in range(1, nExperiments+1):
        print("\nstart experiment " + str(exp_id) + ": " + str(numObjs) + " " + str(int(RAD)) + " " + str(HEIGHT) + " " + str(WIDTH))
        EXP = Experiment(numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, savestat, exp_id, data_path)
        if EXP.genInstanceFailure == True: 
            continue
        else:
            successGenTimes += 1

        if EXP.genSolutionFailure == True:
            pass
        else:
            ### record the stats
            successSolTimes += 1
            comTime += EXP.comTime
            numActions += EXP.totalActions

        if EXP.genSolutionFailure_star == True:
            pass
        else:
            ### record the stats
            successSolTimes_star += 1
            comTime_star += EXP.comTime_star
            numActions_star += EXP.totalActions_star


    print("success times for generating instances: " + str(successGenTimes) + "/" + str(nExperiments))
    print("success times for having a solution for biRRT: " + str(successSolTimes) + "/" + str(nExperiments))
    if (successSolTimes != 0):
        print("average computation time for successful cases: " + str(comTime / successSolTimes))
        print("average number of actions: " + str(numActions / successSolTimes))
    print("success times for having a solution for biRRT*: " + str(successSolTimes_star) + "/" + str(nExperiments))
    if (successSolTimes_star != 0):
        print("average computation time for successful cases: " + str(comTime_star / successSolTimes_star))
        print("average number of actions: " + str(numActions_star / successSolTimes_star))
















