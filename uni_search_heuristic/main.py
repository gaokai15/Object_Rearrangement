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

    ### Once we have the parameters, we can create a folder to store the data with respect to this parameter set
    flag = str(numObjs) + "_" + str(int(RAD)) + "_" + str(HEIGHT) + "_" + str(WIDTH)
    data_path = os.path.join(os.getcwd(), "figures", flag)
    if os.path.exists(data_path):
        shutil.rmtree(data_path)
    os.mkdir(data_path)

    # para_combinations = [[7,80,750,750]]
    para_combinations=[[10,80,900,900], [13,80,1150,1150], [13,80,1030,1030], \
                        [15,80,1230,1230], [15,80,1100,1100]]
    f_stat = open("stat.txt", "w")

    for para_comb in para_combinations:

        numObjs = para_comb[0]
        RAD = para_comb[1]
        HEIGHT = para_comb[2]
        WIDTH = para_comb[3]
        f_stat.write(str(numObjs) + " " + str(RAD) + " " + str(HEIGHT) + " " + str(WIDTH) + "\n")


        # ### prepare the statistics
        # successGenTimes = 0 ### record the number of times you successfully generate an instance

        # successSolTimes_DP_local = 0 ### record the number of times you successfully find a solution
        # comTime_DP_local = 0
        # numActions_DP_local = 0
        # successSolTimes_biRRT = 0 ### record the number of times you successfully find a solution
        # comTime_biRRT = 0
        # numActions_biRRT = 0
        # successSolTimes_biRRTstar = 0 ### record the number of times you successfully find a solution
        # comTime_biRRTstar = 0
        # numActions_biRRTstar = 0

        ### prepare the statistics
        total_successGenTimes = 0 ### record the number of times you successfully generate an instance

        total_successSolTimes_BF = 0 ### record the number of times you successfully find a solution
        average_comTime_BF = 0
        average_numActions_BF = 0

        total_successSolTimes_DP = 0 ### record the number of times you successfully find a solution
        average_comTime_DP = 0
        average_numActions_DP = 0

        total_successSolTimes_DP_heuristic = 0 ### record the number of times you successfully find a solution
        average_comTime_DP_heuristic = 0
        average_numActions_DP_heuristic = 0


        for exp_id in range(1, nExperiments+1):
            print("\nstart experiment " + str(exp_id) + ": " + str(numObjs) + " " + str(int(RAD)) + " " + str(HEIGHT) + " " + str(WIDTH))
            EXP = Experiment(numObjs, RAD, HEIGHT, WIDTH, display, displayMore, displayAnimation, savefile, saveimage, savestat, exp_id, data_path)
            if EXP.genInstanceFailure == True: 
                continue
            else:
                total_successGenTimes += 1

            # ### method 1: DP_local
            # if EXP.genSolutionFailure_DP_local == True:
            #     pass
            # else:
            #     ### record the stats
            #     successSolTimes_DP_local += 1
            #     comTime_DP_local += EXP.comTime_DP_local
            #     numActions_DP_local += EXP.totalActions_DP_local

            # ### method 2: biRRT (arc ILP)
            # if EXP.genSolutionFailure_biRRT == True:
            #     pass
            # else:
            #     ### record the stats
            #     successSolTimes_biRRT += 1
            #     comTime_biRRT += EXP.comTime_biRRT
            #     numActions_biRRT += EXP.totalActions_biRRT

            # ### method 3: biRRT* (arc ILP)
            # if EXP.genSolutionFailure_biRRTstar == True:
            #     pass
            # else:
            #     ### record the stats
            #     successSolTimes_biRRTstar += 1
            #     comTime_biRRTstar += EXP.comTime_biRRTstar
            #     numActions_biRRTstar += EXP.totalActions_biRRTstar


            ############ method 0: brute force ############
            if EXP.genSolutionFailure_BF == True:
                average_comTime_BF += EXP.comTime_BF
            else:
                ### record the stats
                total_successSolTimes_BF += 1
                average_comTime_BF += EXP.comTime_BF
                average_numActions_BF += EXP.totalActions_BF


            ########### method 1: DP ################
            if EXP.genSolutionFailure_DP == True:
                average_comTime_DP += EXP.comTime_DP
            else:
                ### record the stats
                total_successSolTimes_DP += 1
                average_comTime_DP += EXP.comTime_DP
                average_numActions_DP += EXP.totalActions_DP


            ######### method 2: DP with heuristic ##########
            if EXP.genSolutionFailure_DP_heuristic == True:
                average_comTime_DP_heuristic += EXP.comTime_DP_heuristic
            else:
                ### record the stats
                total_successSolTimes_DP_heuristic += 1
                average_comTime_DP_heuristic += EXP.comTime_DP_heuristic
                average_numActions_DP_heuristic += EXP.totalActions_DP_heuristic

        # print("______________________________________________________")
        # print("success times for generating instances: " + str(successGenTimes) + "/" + str(nExperiments))

        # print("success times for having a solution for biRRT_DP_local: " + str(successSolTimes_DP_local) + "/" + str(nExperiments))
        # if (successSolTimes_DP_local != 0):
        #     print("average computation time for successful cases: " + str(comTime_DP_local / successSolTimes_DP_local))
        #     print("average number of actions: " + str(numActions_DP_local / successSolTimes_DP_local))

        # print("success times for having a solution for biRRT: " + str(successSolTimes_biRRT) + "/" + str(nExperiments))
        # if (successSolTimes_biRRT != 0):
        #     print("average computation time for successful cases: " + str(comTime_biRRT / successSolTimes_biRRT))
        #     print("average number of actions: " + str(numActions_biRRT / successSolTimes_biRRT))

        # print("success times for having a solution for biRRT*: " + str(successSolTimes_biRRTstar) + "/" + str(nExperiments))
        # if (successSolTimes_biRRTstar != 0):
        #     print("average computation time for successful cases: " + str(comTime_biRRTstar / successSolTimes_biRRTstar))
        #     print("average number of actions: " + str(numActions_biRRTstar / successSolTimes_biRRTstar))


        ### get the average
        average_comTime_BF = average_comTime_BF / total_successGenTimes
        average_comTime_DP = average_comTime_DP / total_successGenTimes
        average_comTime_DP_heuristic = average_comTime_DP_heuristic / total_successGenTimes
        average_numActions_BF = average_numActions_BF / total_successSolTimes_BF
        average_numActions_DP = average_numActions_DP / total_successSolTimes_DP
        average_numActions_DP_heuristic = average_numActions_DP_heuristic / total_successSolTimes_DP_heuristic
        print("______________________________________________________")
        print("success times for generating instances: " + str(total_successGenTimes) + "/" + str(nExperiments) + "\n")


        print("success times for having a solution for BF: " + str(total_successSolTimes_BF) + "/" + str(total_successGenTimes))
        if (total_successSolTimes_BF != 0):
            print("average computation time for successful cases: " + str(average_comTime_BF))
            print("average number of actions: " + str(average_numActions_BF))

        print("success times for having a solution for DP: " + str(total_successSolTimes_DP) + "/" + str(total_successGenTimes))
        if (total_successSolTimes_DP != 0):
            print("average computation time for successful cases: " + str(average_comTime_DP))
            print("average number of actions: " + str(average_numActions_DP))

        print("success times for having a solution for DP with heuristic: " + str(total_successSolTimes_DP_heuristic) + "/" + str(total_successGenTimes))
        if (total_successSolTimes_DP_heuristic != 0):
            print("average computation time for successful cases: " + str(average_comTime_DP_heuristic))
            print("average number of actions: " + str(average_numActions_DP_heuristic))

        f_stat.write(str(average_numActions_BF) + " " + str(average_comTime_BF) + " " + str(total_successSolTimes_BF / total_successGenTimes) + " " + \
                     str(average_numActions_DP) + " " + str(average_comTime_DP) + " " + str(total_successSolTimes_DP / total_successGenTimes) + " " + \
                     str(average_numActions_DP_heuristic) + " " + str(average_comTime_DP_heuristic) + " " + str(total_successSolTimes_DP_heuristic / total_successGenTimes) + "\n")



    f_stat.close()
















