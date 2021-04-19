from __future__ import division

import os
import sys
import IPython
import shutil
from Experiment import Experiment
import math
import resource

def set_max_memory(MAX):
    soft, hard = resource.getrlimit(resource.RLIMIT_AS)
    resource.setrlimit(resource.RLIMIT_AS, (MAX, hard))

# Disable
def blockPrint():
    sys.stdout = open(os.devnull, 'w')

# Restore
def enablePrint():
    sys.stdout = sys.__stdout__


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

    radius = 80
    para_combinations = []
    for D in [0.45 for i in range(1)]:
        for num in [5]:
            H = math.sqrt(2*num*math.pi*radius*radius/D)
            para_combinations.append([num, radius, H, H])

    f_stat = open("stat.txt", "w")

    for para_comb in para_combinations:
        numObjs = para_comb[0]
        RAD = para_comb[1]
        HEIGHT = para_comb[2]
        WIDTH = para_comb[3]
        f_stat.write(str(numObjs) + " " + str(RAD) + " " + str(HEIGHT) + " " + str(WIDTH) + "\n")

        successGenTimes = 0 ### record the number of times you successfully generate an instance

        successSolTimes_fmRS = 0 ### record the number of times you successfully find a solution
        comTime_fmRS = 0
        numActions_fmRS = 0
        successSolTimes_mRS = 0 ### record the number of times you successfully find a solution
        comTime_mRS = 0
        numActions_mRS = 0
        successSolTimes_Fast_heuristic = 0 ### record the number of times you successfully find a solution
        comTime_Fast_heuristic = 0
        numActions_Fast_heuristic = 0
        successSolTimes_DP_local_leaf_root = 0 ### record the number of times you successfully find a solution
        comTime_DP_local_leaf_root = 0
        numActions_DP_local_leaf_root = 0
        successSolTimes_DP_BruteForce = 0 ### record the number of times you successfully find a solution
        comTime_DP_BruteForce = 0
        numActions_DP_BruteForce = 0
        successSolTimes_Generalized_BruteForce = 0 ### record the number of times you successfully find a solution
        comTime_Generalized_BruteForce = 0
        numActions_Generalized_BruteForce = 0
        successSolTimes_GPD = 0
        comTime_PathOptions = 0
        comTime_SampleArrangements = 0
        numActions_FVS = 0

        for exp_id in range(1, nExperiments+1):
            enablePrint()
            print("\nstart experiment " + str(exp_id) + ": " + str(numObjs) + " " + str(int(RAD)) + " " + str(HEIGHT) + " " + str(WIDTH))
            EXP = Experiment(numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, savestat, exp_id, data_path)
            # if (EXP.genInstanceFailure == True) or (EXP.genSolutionFailure_DP_local == True) or (EXP.genSolutionFailure_DP_BruteForce == True) or (EXP.genSolutionFailure_biRRT == True) or (EXP.genSolutionFailure_biRRTstar == True) or (EXP.genSolutionFailure_GPD == True): 
            if (EXP.genInstanceFailure == True): 
                continue
            else:
                successGenTimes += 1

            blockPrint()

            ### method -2: fmRS
            if EXP.genSolutionFailure_fmRS == True:
                pass
            else:
                ### record the stats
                successSolTimes_fmRS += 1
                comTime_fmRS += EXP.comTime_fmRS
                numActions_fmRS += EXP.totalActions_fmRS

            ### method -1: mRS
            if EXP.genSolutionFailure_mRS == True:
                pass
            else:
                ### record the stats
                successSolTimes_mRS += 1
                comTime_mRS += EXP.comTime_mRS
                numActions_mRS += EXP.totalActions_mRS

            ### method 0: Fast_heuristics
            if EXP.genSolutionFailure_Fast_heuristic == True:
                pass
            else:
                ### record the stats
                successSolTimes_Fast_heuristic += 1
                comTime_Fast_heuristic += EXP.comTime_Fast_heuristic
                numActions_Fast_heuristic += EXP.totalActions_Fast_heuristic

            ### method 1.1: DP_local_leaf_root
            if EXP.genSolutionFailure_DP_local_leaf_root == True:
                pass
            else:
                ### record the stats
                successSolTimes_DP_local_leaf_root += 1
                comTime_DP_local_leaf_root += EXP.comTime_DP_local_leaf_root
                numActions_DP_local_leaf_root += EXP.totalActions_DP_local_leaf_root

            ### method 1.5: DP_BruteForce
            if EXP.genSolutionFailure_DP_BruteForce == True:
                pass
            else:
                ### record the stats
                successSolTimes_DP_BruteForce += 1
                comTime_DP_BruteForce += EXP.comTime_DP_BruteForce
                numActions_DP_BruteForce += EXP.totalActions_DP_BruteForce

            ### method 1.51: DP_BruteForce
            if EXP.genSolutionFailure_Generalized_BruteForce == True:
                pass
            else:
                ### record the stats
                successSolTimes_Generalized_BruteForce += 1
                comTime_Generalized_BruteForce += EXP.comTime_Generalized_BruteForce
                numActions_Generalized_BruteForce += EXP.totalActions_Generalized_BruteForce

        
        enablePrint()
        print("______________________________________________________")
        print("success times for generating instances: " + str(successGenTimes) + "/" + str(nExperiments))

        print("success times for having a solution for fmRS: " + str(successSolTimes_fmRS) + "/" + str(nExperiments))
        if (successSolTimes_fmRS != 0):
            print("average computation time for successful cases: " + str(comTime_fmRS / successSolTimes_fmRS))
            print("average number of actions: " + str(numActions_fmRS / successSolTimes_fmRS))

        print("success times for having a solution for mRS: " + str(successSolTimes_mRS) + "/" + str(nExperiments))
        if (successSolTimes_mRS != 0):
            print("average computation time for successful cases: " + str(comTime_mRS / successSolTimes_mRS))
            print("average number of actions: " + str(numActions_mRS / successSolTimes_mRS))


        
        print("success times for having a solution for Fast_heuristic: " + str(successSolTimes_Fast_heuristic) + "/" + str(nExperiments))
        if (successSolTimes_Fast_heuristic != 0):
            print("average computation time for successful cases: " + str(comTime_Fast_heuristic / successSolTimes_Fast_heuristic))
            print("average number of actions: " + str(numActions_Fast_heuristic / successSolTimes_Fast_heuristic))

        
        print("success times for having a solution for biRRT_DP_local_leaf_root: " + str(successSolTimes_DP_local_leaf_root) + "/" + str(nExperiments))
        if (successSolTimes_DP_local_leaf_root != 0):
            print("average computatirutgers roboticson time for successful cases: " + str(comTime_DP_local_leaf_root / successSolTimes_DP_local_leaf_root))
            print("average number of actions: " + str(numActions_DP_local_leaf_root / successSolTimes_DP_local_leaf_root))

        print("success times for having a solution for biRRT_DP_BruteForce: " + str(successSolTimes_DP_BruteForce) + "/" + str(nExperiments))
        if (successSolTimes_DP_BruteForce != 0):
            print("average computation time for successful cases: " + str(comTime_DP_BruteForce / successSolTimes_DP_BruteForce))
            print("average number of actions: " + str(numActions_DP_BruteForce / successSolTimes_DP_BruteForce))

        print("success times for having a solution for biRRT_Generalized_BruteForce: " + str(successSolTimes_Generalized_BruteForce) + "/" + str(nExperiments))
        if (successSolTimes_Generalized_BruteForce != 0):
            print("average computation time for successful cases: " + str(comTime_Generalized_BruteForce / successSolTimes_Generalized_BruteForce))
            print("average number of actions: " + str(numActions_Generalized_BruteForce / successSolTimes_Generalized_BruteForce))


        f_stat.write(str(nExperiments) + " " + str(successSolTimes_DP_local_leaf_root) + " " + str(successSolTimes_DP_local_leaf_root) + " " + str(successSolTimes_DP_BruteForce) + " " + str(successSolTimes_Generalized_BruteForce) + "\n")
        
        if successSolTimes_Fast_heuristic < 1:
            successSolTimes_Fast_heuristic = -1
        if successSolTimes_DP_BruteForce < 1:
            successSolTimes_DP_BruteForce = -1
        if successSolTimes_Generalized_BruteForce < 1:
            successSolTimes_Generalized_BruteForce = -1
        if successSolTimes_DP_local_leaf_root < 1:
            successSolTimes_DP_local_leaf_root = -1
        if successSolTimes_GPD <1:
            successSolTimes_GPD = -1

        f_stat.write( str(float(numActions_Fast_heuristic) / successSolTimes_Fast_heuristic) + " " + 
            str(float(numActions_DP_local_leaf_root) / successSolTimes_DP_local_leaf_root) + " " + 
            str(float(numActions_DP_BruteForce) / successSolTimes_DP_BruteForce) + " " + 
            str(float(numActions_Generalized_BruteForce) / successSolTimes_Generalized_BruteForce) + "\n")
        f_stat.write( str(comTime_Fast_heuristic / successSolTimes_Fast_heuristic) +" " + 
            str(comTime_DP_local_leaf_root / successSolTimes_DP_local_leaf_root) +" " + 
            str(comTime_DP_BruteForce / successSolTimes_DP_BruteForce) +" " + 
            str(comTime_Generalized_BruteForce / successSolTimes_Generalized_BruteForce) + "\n")


    f_stat.close()



    ############################## previous experiment ####################################################

    # successGenTimes = 0 ### record the number of times you successfully generate an instance

    # successSolTimes = 0 ### record the number of times you successfully find a solution
    # comTime = 0
    # numActions = 0
    # successSolTimes_star = 0 ### record the number of times you successfully find a solution
    # comTime_star = 0
    # numActions_star = 0


    # for exp_id in range(1, nExperiments+1):
    #     print("\nstart experiment " + str(exp_id) + ": " + str(numObjs) + " " + str(int(RAD)) + " " + str(HEIGHT) + " " + str(WIDTH))
    #     EXP = Experiment(numObjs, RAD, HEIGHT, WIDTH, display, displayMore, savefile, saveimage, savestat, exp_id, data_path)
    #     if EXP.genInstanceFailure == True: 
    #         continue
    #     else:
    #         successGenTimes += 1

    #     if EXP.genSolutionFailure == True:
    #         pass
    #     else:
    #         ### record the stats
    #         successSolTimes += 1
    #         comTime += EXP.comTime
    #         numActions += EXP.totalActions

    #     if EXP.genSolutionFailure_star == True:
    #         pass
    #     else:
    #         ### record the stats
    #         successSolTimes_star += 1
    #         comTime_star += EXP.comTime_star
    #         numActions_star += EXP.totalActions_star


    # print("success times for generating instances: " + str(successGenTimes) + "/" + str(nExperiments))
    # print("success times for having a solution for biRRT: " + str(successSolTimes) + "/" + str(nExperiments))
    # if (successSolTimes != 0):
    #     print("average computation time for successful cases: " + str(comTime / successSolTimes))
    #     print("average number of actions: " + str(numActions / successSolTimes))
    # print("success times for having a solution for biRRT*: " + str(successSolTimes_star) + "/" + str(nExperiments))
    # if (successSolTimes_star != 0):
    #     print("average computation time for successful cases: " + str(comTime_star / successSolTimes_star))
    #     print("average number of actions: " + str(numActions_star / successSolTimes_star))
















