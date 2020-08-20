from __future__ import division

import os
import sys
import IPython
import shutil
from Experiment import Experiment
import math

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

    radius = 80
    para_combinations = []
    for D in [0.5 for i in range(50)]:
        for num in [10]:
            H = math.sqrt(2*num*math.pi*radius*radius/D)
            para_combinations.append([num, radius, H, H])

    # para_combinations = [[5,80,700,700], [7,80,650,650]]
    # para_combinations = [[10,80,900,900]]
    # para_combinations=[[10,80,900,900], [13,80,1150,1150], [13,80,1030,1030], \
    #                     [15,80,1230,1230], [15,80,1100,1100], [18,80,1350,1350], [18,80,1200,1200]]
    f_stat = open("stat.txt", "w")

    for para_comb in para_combinations:
        numObjs = para_comb[0]
        RAD = para_comb[1]
        HEIGHT = para_comb[2]
        WIDTH = para_comb[3]
        f_stat.write(str(numObjs) + " " + str(RAD) + " " + str(HEIGHT) + " " + str(WIDTH) + "\n")

        successGenTimes = 0 ### record the number of times you successfully generate an instance

        successSolTimes_DP_local = 0 ### record the number of times you successfully find a solution
        comTime_DP_local = 0
        numActions_DP_local = 0
        successSolTimes_DP_local_A_star_furthest = 0 ### record the number of times you successfully find a solution
        comTime_DP_local_A_star_furthest = 0
        numActions_DP_local_A_star_furthest = 0
        successSolTimes_DP_local_A_star_nearest = 0 ### record the number of times you successfully find a solution
        comTime_DP_local_A_star_nearest = 0
        numActions_DP_local_A_star_nearest = 0
        successSolTimes_DP_local_suboptimal_furthest = 0 ### record the number of times you successfully find a solution
        comTime_DP_local_suboptimal_furthest = 0
        numActions_DP_local_suboptimal_furthest = 0
        successSolTimes_DP_local_random_range = 0 ### record the number of times you successfully find a solution
        comTime_DP_local_random_range = 0
        numActions_DP_local_random_range = 0
        successSolTimes_DP_local_random_nearest = 0 ### record the number of times you successfully find a solution
        comTime_DP_local_random_nearest = 0
        numActions_DP_local_random_nearest = 0
        successSolTimes_DP_local_leaf_root = 0 ### record the number of times you successfully find a solution
        comTime_DP_local_leaf_root = 0
        numActions_DP_local_leaf_root = 0
        successSolTimes_DP_local_leaf_root_Improved_Mutation = 0 ### record the number of times you successfully find a solution
        comTime_DP_local_leaf_root_Improved_Mutation = 0
        numActions_DP_local_leaf_root_Improved_Mutation = 0
        successSolTimes_DP_local_leaf_small_range = 0 ### record the number of times you successfully find a solution
        comTime_DP_local_leaf_small_range = 0
        numActions_DP_local_leaf_small_range = 0
        successSolTimes_DP_local_leaf_large_range = 0 ### record the number of times you successfully find a solution
        comTime_DP_local_leaf_large_range = 0
        numActions_DP_local_leaf_large_range = 0
        successSolTimes_DP_local_leaf_nearest = 0 ### record the number of times you successfully find a solution
        comTime_DP_local_leaf_nearest = 0
        numActions_DP_local_leaf_nearest = 0
        successSolTimes_DP_BruteForce = 0 ### record the number of times you successfully find a solution
        comTime_DP_BruteForce = 0
        numActions_DP_BruteForce = 0
        successSolTimes_biRRT = 0 ### record the number of times you successfully find a solution
        comTime_biRRT = 0
        numActions_biRRT = 0
        successSolTimes_biRRTstar = 0 ### record the number of times you successfully find a solution
        comTime_biRRTstar = 0
        numActions_biRRTstar = 0
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

            ### method 1: DP_local
            if EXP.genSolutionFailure_DP_local == True:
                pass
            else:
                ### record the stats
                successSolTimes_DP_local += 1
                comTime_DP_local += EXP.comTime_DP_local
                numActions_DP_local += EXP.totalActions_DP_local
            enablePrint()
            try:
                for i in range(EXP.plan_DP_local.num_mutation):
                    EXP.plan_DP_local.total_time += EXP.plan_DP_local.mutation_time_list[i]
                if EXP.plan_DP_local.num_mutation != 0:
                    EXP.plan_DP_local.avg_time = EXP.plan_DP_local.total_time/EXP.plan_DP_local.num_mutation
                print EXP.genSolutionFailure_DP_local, EXP.plan_DP_local.num_mutation, EXP.plan_DP_local.avg_time, EXP.plan_DP_local.total_time
            except Exception:
                print "Fail"
            blockPrint()

            ### method 1.001: DP_local_A_star_furthest
            if EXP.genSolutionFailure_DP_local_A_star_furthest == True:
                pass
            else:
                ### record the stats
                successSolTimes_DP_local_A_star_furthest += 1
                comTime_DP_local_A_star_furthest += EXP.comTime_DP_local_A_star_furthest
                numActions_DP_local_A_star_furthest += EXP.totalActions_DP_local_A_star_furthest
            enablePrint()
            try:
                for i in range(EXP.plan_DP_local_A_star_furthest.num_mutation):
                    EXP.plan_DP_local_A_star_furthest.total_time += EXP.plan_DP_local_A_star_furthest.mutation_time_list[i]
                if EXP.plan_DP_local_A_star_furthest.num_mutation != 0:
                    EXP.plan_DP_local_A_star_furthest.avg_time = EXP.plan_DP_local_A_star_furthest.total_time/EXP.plan_DP_local_A_star_furthest.num_mutation
                print EXP.genSolutionFailure_DP_local_A_star_furthest, EXP.plan_DP_local_A_star_furthest.num_mutation, EXP.plan_DP_local_A_star_furthest.avg_time, EXP.plan_DP_local_A_star_furthest.total_time
            except Exception:
                print "Fail"
            blockPrint()

            ### method 1.002: DP_local_A_star_nearest
            if EXP.genSolutionFailure_DP_local_A_star_nearest == True:
                pass
            else:
                ### record the stats
                successSolTimes_DP_local_A_star_nearest += 1
                comTime_DP_local_A_star_nearest += EXP.comTime_DP_local_A_star_nearest
                numActions_DP_local_A_star_nearest += EXP.totalActions_DP_local_A_star_nearest
            enablePrint()
            try:
                for i in range(EXP.plan_DP_local_A_star_nearest.num_mutation):
                    EXP.plan_DP_local_A_star_nearest.total_time += EXP.plan_DP_local_A_star_nearest.mutation_time_list[i]
                if EXP.plan_DP_local_A_star_nearest.num_mutation != 0:
                    EXP.plan_DP_local_A_star_nearest.avg_time = EXP.plan_DP_local_A_star_nearest.total_time/EXP.plan_DP_local_A_star_nearest.num_mutation
                print EXP.genSolutionFailure_DP_local_A_star_nearest, EXP.plan_DP_local_A_star_nearest.num_mutation, EXP.plan_DP_local_A_star_nearest.avg_time, EXP.plan_DP_local_A_star_nearest.total_time
            except Exception:
                print "Fail"
            blockPrint()

            ### method 1.003: DP_local_suboptimal_furthest
            if EXP.genSolutionFailure_DP_local_suboptimal_furthest == True:
                pass
            else:
                ### record the stats
                successSolTimes_DP_local_suboptimal_furthest += 1
                comTime_DP_local_suboptimal_furthest += EXP.comTime_DP_local_suboptimal_furthest
                numActions_DP_local_suboptimal_furthest += EXP.totalActions_DP_local_suboptimal_furthest
            enablePrint()
            try:
                for i in range(EXP.plan_DP_local_suboptimal_furthest.num_mutation):
                    EXP.plan_DP_local_suboptimal_furthest.total_time += EXP.plan_DP_local_suboptimal_furthest.mutation_time_list[i]
                if EXP.plan_DP_local_suboptimal_furthest.num_mutation != 0:
                    EXP.plan_DP_local_suboptimal_furthest.avg_time = EXP.plan_DP_local_suboptimal_furthest.total_time/EXP.plan_DP_local_suboptimal_furthest.num_mutation
                print EXP.genSolutionFailure_DP_local_suboptimal_furthest, EXP.plan_DP_local_suboptimal_furthest.num_mutation, EXP.plan_DP_local_suboptimal_furthest.avg_time, EXP.plan_DP_local_suboptimal_furthest.total_time
            except Exception:
                print "Fail"
            blockPrint()

            ### method 1.02: DP_local_random_range
            if EXP.genSolutionFailure_DP_local_random_range == True:
                pass
            else:
                ### record the stats
                successSolTimes_DP_local_random_range += 1
                comTime_DP_local_random_range += EXP.comTime_DP_local_random_range
                numActions_DP_local_random_range += EXP.totalActions_DP_local_random_range
            enablePrint()
            try:
                for i in range(EXP.plan_DP_local_random_range.num_mutation):
                    EXP.plan_DP_local_random_range.total_time += EXP.plan_DP_local_random_range.mutation_time_list[i]
                if EXP.plan_DP_local_random_range.num_mutation != 0:
                    EXP.plan_DP_local_random_range.avg_time = EXP.plan_DP_local_random_range.total_time/EXP.plan_DP_local_random_range.num_mutation
                print EXP.genSolutionFailure_DP_local_random_range, EXP.plan_DP_local_random_range.num_mutation, EXP.plan_DP_local_random_range.avg_time, EXP.plan_DP_local_random_range.total_time
            except Exception:
                print "Fail"
            blockPrint()

            ### method 1.03: DP_local_random_nearest
            if EXP.genSolutionFailure_DP_local_random_nearest == True:
                pass
            else:
                ### record the stats
                successSolTimes_DP_local_random_nearest += 1
                comTime_DP_local_random_nearest += EXP.comTime_DP_local_random_nearest
                numActions_DP_local_random_nearest += EXP.totalActions_DP_local_random_nearest
            enablePrint()
            try:
                for i in range(EXP.plan_DP_local_random_nearest.num_mutation):
                    EXP.plan_DP_local_random_nearest.total_time += EXP.plan_DP_local_random_nearest.mutation_time_list[i]
                if EXP.plan_DP_local_random_nearest.num_mutation != 0:
                    EXP.plan_DP_local_random_nearest.avg_time = EXP.plan_DP_local_random_nearest.total_time/EXP.plan_DP_local_random_nearest.num_mutation
                print EXP.genSolutionFailure_DP_local_random_nearest, EXP.plan_DP_local_random_nearest.num_mutation, EXP.plan_DP_local_random_nearest.avg_time, EXP.plan_DP_local_random_nearest.total_time
            except Exception:
                print "Fail"
            blockPrint()

            ### method 1.1: DP_local_leaf_root
            if EXP.genSolutionFailure_DP_local_leaf_root == True:
                pass
            else:
                ### record the stats
                successSolTimes_DP_local_leaf_root += 1
                comTime_DP_local_leaf_root += EXP.comTime_DP_local_leaf_root
                numActions_DP_local_leaf_root += EXP.totalActions_DP_local_leaf_root
            enablePrint()
            try:
                for i in range(EXP.plan_DP_local_leaf_root.num_mutation):
                    EXP.plan_DP_local_leaf_root.total_time += EXP.plan_DP_local_leaf_root.mutation_time_list[i]
                if EXP.plan_DP_local_leaf_root.num_mutation != 0:
                    EXP.plan_DP_local_leaf_root.avg_time = EXP.plan_DP_local_leaf_root.total_time/EXP.plan_DP_local_leaf_root.num_mutation
                print EXP.genSolutionFailure_DP_local_leaf_root, EXP.plan_DP_local_leaf_root.num_mutation, EXP.plan_DP_local_leaf_root.avg_time, EXP.plan_DP_local_leaf_root.total_time
            except Exception:
                print "Fail"
            blockPrint()


            ### method 1.11: DP_local_leaf_root_Improved_Mutation
            if EXP.genSolutionFailure_DP_local_leaf_root_Improved_Mutation == True:
                pass
            else:
                ### record the stats
                successSolTimes_DP_local_leaf_root_Improved_Mutation += 1
                comTime_DP_local_leaf_root_Improved_Mutation += EXP.comTime_DP_local_leaf_root_Improved_Mutation
                numActions_DP_local_leaf_root_Improved_Mutation += EXP.totalActions_DP_local_leaf_root_Improved_Mutation
            enablePrint()
            try:
                for i in range(EXP.plan_DP_local_leaf_root_Improved_Mutation.num_mutation):
                    EXP.plan_DP_local_leaf_root_Improved_Mutation.total_time += EXP.plan_DP_local_leaf_root_Improved_Mutation.mutation_time_list[i]
                if EXP.plan_DP_local_leaf_root_Improved_Mutation.num_mutation != 0:
                    EXP.plan_DP_local_leaf_root_Improved_Mutation.avg_time = EXP.plan_DP_local_leaf_root_Improved_Mutation.total_time/EXP.plan_DP_local_leaf_root_Improved_Mutation.num_mutation
                print EXP.genSolutionFailure_DP_local_leaf_root_Improved_Mutation, EXP.plan_DP_local_leaf_root_Improved_Mutation.num_mutation, EXP.plan_DP_local_leaf_root_Improved_Mutation.avg_time, EXP.plan_DP_local_leaf_root_Improved_Mutation.total_time
            except Exception:
                print "Fail"
            blockPrint()


            ### method 1.21: DP_local_leaf_small_range
            if EXP.genSolutionFailure_DP_local_leaf_small_range == True:
                pass
            else:
                ### record the stats
                successSolTimes_DP_local_leaf_small_range += 1
                comTime_DP_local_leaf_small_range += EXP.comTime_DP_local_leaf_small_range
                numActions_DP_local_leaf_small_range += EXP.totalActions_DP_local_leaf_small_range
            enablePrint()
            try:
                for i in range(EXP.plan_DP_local_leaf_small_range.num_mutation):
                    EXP.plan_DP_local_leaf_small_range.total_time += EXP.plan_DP_local_leaf_small_range.mutation_time_list[i]
                if EXP.plan_DP_local_leaf_small_range.num_mutation != 0:
                    EXP.plan_DP_local_leaf_small_range.avg_time = EXP.plan_DP_local_leaf_small_range.total_time/EXP.plan_DP_local_leaf_small_range.num_mutation
                print EXP.genSolutionFailure_DP_local_leaf_small_range, EXP.plan_DP_local_leaf_small_range.num_mutation, EXP.plan_DP_local_leaf_small_range.avg_time, EXP.plan_DP_local_leaf_small_range.total_time
            except Exception:
                print "Fail"
            blockPrint()

            ### method 1.22: DP_local_leaf_large_range
            if EXP.genSolutionFailure_DP_local_leaf_large_range == True:
                pass
            else:
                ### record the stats
                successSolTimes_DP_local_leaf_large_range += 1
                comTime_DP_local_leaf_large_range += EXP.comTime_DP_local_leaf_large_range
                numActions_DP_local_leaf_large_range += EXP.totalActions_DP_local_leaf_large_range
            enablePrint()
            try:
                for i in range(EXP.plan_DP_local_leaf_large_range.num_mutation):
                    EXP.plan_DP_local_leaf_large_range.total_time += EXP.plan_DP_local_leaf_large_range.mutation_time_list[i]
                if EXP.plan_DP_local_leaf_large_range.num_mutation != 0:
                    EXP.plan_DP_local_leaf_large_range.avg_time = EXP.plan_DP_local_leaf_large_range.total_time/EXP.plan_DP_local_leaf_large_range.num_mutation
                print EXP.genSolutionFailure_DP_local_leaf_large_range, EXP.plan_DP_local_leaf_large_range.num_mutation, EXP.plan_DP_local_leaf_large_range.avg_time, EXP.plan_DP_local_leaf_large_range.total_time
            except Exception:
                print "Fail"
            blockPrint()

            ### method 1.3: DP_local_leaf_nearest
            if EXP.genSolutionFailure_DP_local_leaf_nearest == True:
                pass
            else:
                ### record the stats
                successSolTimes_DP_local_leaf_nearest += 1
                comTime_DP_local_leaf_nearest += EXP.comTime_DP_local_leaf_nearest
                numActions_DP_local_leaf_nearest += EXP.totalActions_DP_local_leaf_nearest
            enablePrint()
            try:
                for i in range(EXP.plan_DP_local_leaf_nearest.num_mutation):
                    EXP.plan_DP_local_leaf_nearest.total_time += EXP.plan_DP_local_leaf_nearest.mutation_time_list[i]
                if EXP.plan_DP_local_leaf_nearest.num_mutation != 0:
                    EXP.plan_DP_local_leaf_nearest.avg_time = EXP.plan_DP_local_leaf_nearest.total_time/EXP.plan_DP_local_leaf_nearest.num_mutation
                print EXP.genSolutionFailure_DP_local_leaf_nearest, EXP.plan_DP_local_leaf_nearest.num_mutation, EXP.plan_DP_local_leaf_nearest.avg_time, EXP.plan_DP_local_leaf_nearest.total_time
            except Exception:
                print "Fail"
            blockPrint()

            ### method 1.5: DP_BruteForce
            if EXP.genSolutionFailure_DP_BruteForce == True:
                pass
            else:
                ### record the stats
                successSolTimes_DP_BruteForce += 1
                comTime_DP_BruteForce += EXP.comTime_DP_BruteForce
                numActions_DP_BruteForce += EXP.totalActions_DP_BruteForce

            ### method 2: biRRT (arc ILP)
            if EXP.genSolutionFailure_biRRT == True:
                pass
            else:
                ### record the stats
                successSolTimes_biRRT += 1
                comTime_biRRT += EXP.comTime_biRRT
                numActions_biRRT += EXP.totalActions_biRRT

            ### method 3: biRRT* (arc ILP)
            if EXP.genSolutionFailure_biRRTstar == True:
                pass
            else:
                ### record the stats
                successSolTimes_biRRTstar += 1
                comTime_biRRTstar += EXP.comTime_biRRTstar
                numActions_biRRTstar += EXP.totalActions_biRRTstar

            if EXP.genSolutionFailure_GPD == True:
                pass
            else:
                successSolTimes_GPD += 1
                comTime_PathOptions += EXP.comTime_PathOptions
                # comTime_SampleArrangements += EXP.comTime_SampleArrangements
                numActions_FVS += EXP.totalActions_FVS
        
        enablePrint()
        print("______________________________________________________")
        print("success times for generating instances: " + str(successGenTimes) + "/" + str(nExperiments))

        # print("success times for having a solution for biRRT_DP_local: " + str(successSolTimes_DP_local) + "/" + str(nExperiments))
        # if (successSolTimes_DP_local != 0):
        #     print("average computation time for successful cases: " + str(comTime_DP_local / successSolTimes_DP_local))
        #     print("average number of actions: " + str(numActions_DP_local / successSolTimes_DP_local))
        
        print("success times for having a solution for biRRT_DP_local_A_star_furthest: " + str(successSolTimes_DP_local_A_star_furthest) + "/" + str(nExperiments))
        if (successSolTimes_DP_local_A_star_furthest != 0):
            print("average computation time for successful cases: " + str(comTime_DP_local_A_star_furthest / successSolTimes_DP_local_A_star_furthest))
            print("average number of actions: " + str(numActions_DP_local_A_star_furthest / successSolTimes_DP_local_A_star_furthest))

        print("success times for having a solution for biRRT_DP_local_A_star_nearest: " + str(successSolTimes_DP_local_A_star_nearest) + "/" + str(nExperiments))
        if (successSolTimes_DP_local_A_star_nearest != 0):
            print("average computation time for successful cases: " + str(comTime_DP_local_A_star_nearest / successSolTimes_DP_local_A_star_nearest))
            print("average number of actions: " + str(numActions_DP_local_A_star_nearest / successSolTimes_DP_local_A_star_nearest))

        print("success times for having a solution for biRRT_DP_local_suboptimal_furthest: " + str(successSolTimes_DP_local_suboptimal_furthest) + "/" + str(nExperiments))
        if (successSolTimes_DP_local_suboptimal_furthest != 0):
            print("average computation time for successful cases: " + str(comTime_DP_local_suboptimal_furthest / successSolTimes_DP_local_suboptimal_furthest))
            print("average number of actions: " + str(numActions_DP_local_suboptimal_furthest / successSolTimes_DP_local_suboptimal_furthest))

        # print("success times for having a solution for biRRT_DP_local_random_range: " + str(successSolTimes_DP_local_random_range) + "/" + str(nExperiments))
        # if (successSolTimes_DP_local_random_range != 0):
        #     print("average computation time for successful cases: " + str(comTime_DP_local_random_range / successSolTimes_DP_local_random_range))
        #     print("average number of actions: " + str(numActions_DP_local_random_range / successSolTimes_DP_local_random_range))

        # print("success times for having a solution for biRRT_DP_local_random_nearest: " + str(successSolTimes_DP_local_random_nearest) + "/" + str(nExperiments))
        # if (successSolTimes_DP_local_random_nearest != 0):
        #     print("average computation time for successful cases: " + str(comTime_DP_local_random_nearest / successSolTimes_DP_local_random_nearest))
        #     print("average number of actions: " + str(numActions_DP_local_random_nearest / successSolTimes_DP_local_random_nearest))

        print("success times for having a solution for biRRT_DP_local_leaf_root: " + str(successSolTimes_DP_local_leaf_root) + "/" + str(nExperiments))
        if (successSolTimes_DP_local_leaf_root != 0):
            print("average computation time for successful cases: " + str(comTime_DP_local_leaf_root / successSolTimes_DP_local_leaf_root))
            print("average number of actions: " + str(numActions_DP_local_leaf_root / successSolTimes_DP_local_leaf_root))

        print("success times for having a solution for biRRT_DP_local_leaf_root_Improved_Mutation: " + str(successSolTimes_DP_local_leaf_root_Improved_Mutation) + "/" + str(nExperiments))
        if (successSolTimes_DP_local_leaf_root_Improved_Mutation != 0):
            print("average computation time for successful cases: " + str(comTime_DP_local_leaf_root_Improved_Mutation / successSolTimes_DP_local_leaf_root_Improved_Mutation))
            print("average number of actions: " + str(numActions_DP_local_leaf_root_Improved_Mutation / successSolTimes_DP_local_leaf_root_Improved_Mutation))
        
        # print("success times for having a solution for biRRT_DP_local_leaf_small_range: " + str(successSolTimes_DP_local_leaf_small_range) + "/" + str(nExperiments))
        # if (successSolTimes_DP_local_leaf_small_range != 0):
        #     print("average computation time for successful cases: " + str(comTime_DP_local_leaf_small_range / successSolTimes_DP_local_leaf_small_range))
        #     print("average number of actions: " + str(numActions_DP_local_leaf_small_range / successSolTimes_DP_local_leaf_small_range))

        # print("success times for having a solution for biRRT_DP_local_leaf_large_range: " + str(successSolTimes_DP_local_leaf_large_range) + "/" + str(nExperiments))
        # if (successSolTimes_DP_local_leaf_large_range != 0):
        #     print("average computation time for successful cases: " + str(comTime_DP_local_leaf_large_range / successSolTimes_DP_local_leaf_large_range))
        #     print("average number of actions: " + str(numActions_DP_local_leaf_large_range / successSolTimes_DP_local_leaf_large_range))

        # print("success times for having a solution for biRRT_DP_local_leaf_nearest: " + str(successSolTimes_DP_local_leaf_nearest) + "/" + str(nExperiments))
        # if (successSolTimes_DP_local_leaf_nearest != 0):
        #     print("average computation time for successful cases: " + str(comTime_DP_local_leaf_nearest / successSolTimes_DP_local_leaf_nearest))
        #     print("average number of actions: " + str(numActions_DP_local_leaf_nearest / successSolTimes_DP_local_leaf_nearest))

        print("success times for having a solution for biRRT_DP_BruteForce: " + str(successSolTimes_DP_BruteForce) + "/" + str(nExperiments))
        if (successSolTimes_DP_BruteForce != 0):
            print("average computation time for successful cases: " + str(comTime_DP_BruteForce / successSolTimes_DP_BruteForce))
            print("average number of actions: " + str(numActions_DP_BruteForce / successSolTimes_DP_BruteForce))

        # print("success times for having a solution for biRRT: " + str(successSolTimes_biRRT) + "/" + str(nExperiments))
        # if (successSolTimes_biRRT != 0):
        #     print("average computation time for successful cases: " + str(comTime_biRRT / successSolTimes_biRRT))
        #     print("average number of actions: " + str(numActions_biRRT / successSolTimes_biRRT))

        # print("success times for having a solution for biRRT*: " + str(successSolTimes_biRRTstar) + "/" + str(nExperiments))
        # if (successSolTimes_biRRTstar != 0):
        #     print("average computation time for successful cases: " + str(comTime_biRRTstar / successSolTimes_biRRTstar))
        #     print("average number of actions: " + str(numActions_biRRTstar / successSolTimes_biRRTstar))

        f_stat.write(str(nExperiments) + " " + str(successSolTimes_DP_local_leaf_root) + " " + str(successSolTimes_DP_local_leaf_root_Improved_Mutation) + " " + str(successSolTimes_DP_BruteForce) + "\n")

        if successSolTimes_biRRT < 1:
            successSolTimes_biRRT = -1
        if successSolTimes_biRRTstar < 1:
            successSolTimes_biRRTstar = -1
        if successSolTimes_DP_BruteForce < 1:
            successSolTimes_DP_BruteForce = -1
        if successSolTimes_DP_local < 1:
            successSolTimes_DP_local = -1
        if successSolTimes_DP_local_A_star_furthest < 1:
            successSolTimes_DP_local_A_star_furthest = -1
        if successSolTimes_DP_local_A_star_nearest < 1:
            successSolTimes_DP_local_A_star_nearest = -1
        if successSolTimes_DP_local_suboptimal_furthest < 1:
            successSolTimes_DP_local_suboptimal_furthest = -1
        if successSolTimes_DP_local_random_range < 1:
            successSolTimes_DP_local_random_range = -1
        if successSolTimes_DP_local_random_nearest < 1:
            successSolTimes_DP_local_random_nearest = -1
        if successSolTimes_DP_local_leaf_root < 1:
            successSolTimes_DP_local_leaf_root = -1
        if successSolTimes_DP_local_leaf_root_Improved_Mutation < 1:
            successSolTimes_DP_local_leaf_root_Improved_Mutation = -1
        if successSolTimes_DP_local_leaf_small_range < 1:
            successSolTimes_DP_local_leaf_small_range = -1
        if successSolTimes_DP_local_leaf_large_range < 1:
            successSolTimes_DP_local_leaf_large_range = -1
        if successSolTimes_DP_local_leaf_nearest < 1:
            successSolTimes_DP_local_leaf_nearest = -1
        if successSolTimes_GPD <1:
            successSolTimes_GPD = -1

        f_stat.write(str(float(numActions_DP_local_leaf_root) / successSolTimes_DP_local_leaf_root) + " " +str(float(numActions_DP_local_leaf_root_Improved_Mutation) / successSolTimes_DP_local_leaf_root_Improved_Mutation) + " " + str(float(numActions_DP_BruteForce) / successSolTimes_DP_BruteForce) + " " + "\n")
        f_stat.write( str(comTime_DP_local_leaf_root / successSolTimes_DP_local_leaf_root) +" " + str(comTime_DP_local_leaf_root_Improved_Mutation / successSolTimes_DP_local_leaf_root_Improved_Mutation) +" " + str(comTime_DP_BruteForce / successSolTimes_DP_BruteForce) + "\n")


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
















