from __future__ import division
### This file is used for evaluation all existing instances generated before


import os
import sys
import IPython
import shutil
import math
import IPython

from Experiment_evaluation import Experiment_evaluation


def getSubFolder(superFolder):
    final_folderList = []
    for file in os.listdir(superFolder):
        if not file.endswith(".txt") and not file.endswith(".pdf"):
            final_folderList.append(file)

    return final_folderList


if __name__ == "__main__":

    ### only need to specify the display and save setting
    display = False
    if (len(sys.argv) > 1):
        display = sys.argv[1] not in ('n', 'N')

    displayMore = False
    if (len(sys.argv) > 2):
        displayMore = sys.argv[2] not in ('n', 'N')

    displayAnimation = False
    if (len(sys.argv) > 3):
        displayAnimation = sys.argv[3] not in ('n', 'N')

    savefile = False
    if (len(sys.argv) > 4):
        savefile = sys.argv[4] not in ('n', 'N')

    saveimage = False
    if (len(sys.argv) > 5):
        saveimage = sys.argv[5] not in ('n', 'N')

    savestat = False
    if (len(sys.argv) > 6):
        savestat = sys.argv[6] not in ('n', 'N')

    ### now loop through all possible parameter combinations (paras_settings)
    paras_folder = os.path.join(os.getcwd(), "instances")
    paras_settings = getSubFolder(paras_folder)
    paras_settings = ['10_80_945_945']
    for paras in paras_settings:
        line = paras.split("_")
        numObjs = int(line[0])
        RAD = int(line[1])
        HEIGHT = int(line[2])
        WIDTH = int(line[3])

        ### now loop through each buffer folder
        buffer_folder = os.path.join(paras_folder, paras)
        buffer_settings = getSubFolder(buffer_folder)
        buffer_settings = ["one_buffer"]
        for buffs_str in buffer_settings:
            if buffs_str == "zero_buffer":
                nbuffs = 0
            elif buffs_str == "one_buffer":
                nbuffs = 1
            elif buffs_str == "two_buffer":
                nbuffs = 2
            else:
                nBuffs = 1000
            ### we need to create a stat to compute everything 
            ### in this buffer setting under this parameter setting

            if numObjs <= 10:
                ### method 0: UniDir mRS
                total_successSolTimes_UniDirmRS = 0
                average_comTime_UniDirmRS = 0
                average_numAddActions_UniDirmRS = 0

            # ### method 1: UniDir fmRS
            # total_successSolTimes_UniDirfmRS = 0
            # average_comTime_UniDirfmRS = 0
            # average_numAddActions_UniDirfmRS = 0            

            # ### method 2: unidirectional DP
            # total_successSolTimes_uniDirDP = 0
            # average_comTime_uniDirDP = 0
            # average_numAddActions_uniDirDP = 0

            # ### method 3: unidirectional leafNode DP
            # total_successSolTimes_uniDirLN_DP = 0
            # average_comTime_uniDirLN_DP = 0
            # average_numAddActions_uniDirLN_DP = 0            

            # ### method 4: Ultimate heuristic
            # total_successSolTimes_UltimateHeuristic = 0
            # average_comTime_UltimateHeuristic = 0
            # average_numAddActions_UltimateHeuristic = 0

            ### now loop through each instance folder
            instance_folder = os.path.join(buffer_folder, buffs_str)
            instance_ids = getSubFolder(instance_folder)
            total_num_instances = len(instance_ids)
            for instance_id in instance_ids:
                instance_dir = os.path.join(instance_folder, instance_id)
                ### NOW: RUN THE EXPERIMENT
                print("\nrun experiment on " + str(numObjs) + " " + buffs_str + " " + instance_id)
                EXP = Experiment_evaluation(
                        numObjs, RAD, HEIGHT, WIDTH, \
                        display, displayMore, displayAnimation, savefile, saveimage, savestat, \
                        instance_dir)
                ############ get the results for the current single experiment ############

                if numObjs <= 10:
                    ### method 0: UniDir mRS
                    if EXP.genSolutionFailure_UniDirmRS == True:
                        average_comTime_UniDirmRS += EXP.comTime_UniDirmRS
                    else:
                        ### record the stats
                        total_successSolTimes_UniDirmRS += 1
                        average_comTime_UniDirmRS += EXP.comTime_UniDirmRS
                        average_numAddActions_UniDirmRS += EXP.additionalActions_UniDirmRS                    
                
                # ### method 1: UniDir fmRS
                # if EXP.genSolutionFailure_UniDirfmRS == True:
                #     average_comTime_UniDirfmRS += EXP.comTime_UniDirfmRS
                # else:
                #     ### record the stats
                #     total_successSolTimes_UniDirfmRS += 1
                #     average_comTime_UniDirfmRS += EXP.comTime_UniDirfmRS
                #     average_numAddActions_UniDirfmRS += EXP.additionalActions_UniDirfmRS

                # ### method 2: unidirectional DP
                # if EXP.genSolutionFailure_uniDirDP == True:
                #     average_comTime_uniDirDP += EXP.comTime_uniDirDP
                # else:
                #     ### record the stats
                #     total_successSolTimes_uniDirDP += 1
                #     average_comTime_uniDirDP += EXP.comTime_uniDirDP
                #     average_numAddActions_uniDirDP += EXP.additionalActions_uniDirDP

                # ### method 3: unidirectional leafNode DP
                # if EXP.genSolutionFailure_uniDirLN_DP == True:
                #     average_comTime_uniDirLN_DP += EXP.comTime_uniDirLN_DP
                # else:
                #     ### record the stats
                #     total_successSolTimes_uniDirLN_DP += 1
                #     average_comTime_uniDirLN_DP += EXP.comTime_uniDirLN_DP
                #     average_numAddActions_uniDirLN_DP += EXP.additionalActions_uniDirLN_DP

                # ### method 4: Ultimate heuristic
                # if EXP.genSolutionFailure_UltimateHeuristic == True:
                #     average_comTime_UltimateHeuristic += EXP.comTime_UltimateHeuristic
                # else:
                #     ### record the stats
                #     total_successSolTimes_UltimateHeuristic += 1
                #     average_comTime_UltimateHeuristic += EXP.comTime_UltimateHeuristic
                #     average_numAddActions_UltimateHeuristic += EXP.additionalActions_UltimateHeuristic

            ### reach here since all the instances under this buffer and parameter setting are finished
            if numObjs <= 10:
                average_comTime_UniDirmRS = average_comTime_UniDirmRS / total_num_instances
                average_numAddActions_UniDirmRS = average_numAddActions_UniDirmRS / total_successSolTimes_UniDirmRS
                success_rate_UniDirmRS = total_successSolTimes_UniDirmRS / total_num_instances

            # average_comTime_UniDirfmRS = average_comTime_UniDirfmRS / total_num_instances
            # average_numAddActions_UniDirfmRS = average_numAddActions_UniDirfmRS / total_successSolTimes_UniDirfmRS
            # success_rate_UniDirfmRS = total_successSolTimes_UniDirfmRS / total_num_instances

            # average_comTime_uniDirDP = average_comTime_uniDirDP / total_num_instances
            # average_numAddActions_uniDirDP = average_numAddActions_uniDirDP / total_successSolTimes_uniDirDP
            # success_rate_uniDirDP = total_successSolTimes_uniDirDP / total_num_instances

            # average_comTime_uniDirLN_DP = average_comTime_uniDirLN_DP / total_num_instances
            # average_numAddActions_uniDirLN_DP = average_numAddActions_uniDirLN_DP / total_successSolTimes_uniDirLN_DP
            # success_rate_uniDirLN_DP = total_successSolTimes_uniDirLN_DP / total_num_instances

            # average_comTime_UltimateHeuristic = average_comTime_UltimateHeuristic / total_num_instances
            # average_numAddActions_UltimateHeuristic = average_numAddActions_UltimateHeuristic / total_successSolTimes_UltimateHeuristic
            # success_rate_UltimateHeuristic = total_successSolTimes_UltimateHeuristic / total_num_instances

            if numObjs <= 10:
                print("UniDirmRS: ")
                print("success rate: " + str(total_successSolTimes_UniDirmRS) + "/" + str(total_num_instances))
                if (total_successSolTimes_UniDirmRS != 0):
                    print("average computation time: " + str(average_comTime_UniDirmRS))
                    print("average additional #actions: " + str(average_numAddActions_UniDirmRS))

            # print("UniDirfmRS: ")
            # print("success rate: " + str(total_successSolTimes_UniDirfmRS) + "/" + str(total_num_instances))
            # if (total_successSolTimes_UniDirfmRS != 0):
            #     print("average computation time: " + str(average_comTime_UniDirfmRS))
            #     print("average additional #actions: " + str(average_numAddActions_UniDirfmRS))

            # print("uniDirDP: ")
            # print("success rate: " + str(total_successSolTimes_uniDirDP) + "/" + str(total_num_instances))
            # if (total_successSolTimes_uniDirDP != 0):
            #     print("average computation time: " + str(average_comTime_uniDirDP))
            #     print("average additional #actions: " + str(average_numAddActions_uniDirDP))

            # print("uniDirLN_DP: ")
            # print("success rate: " + str(total_successSolTimes_uniDirLN_DP) + "/" + str(total_num_instances))
            # if (total_successSolTimes_uniDirLN_DP != 0):
            #     print("average computation time: " + str(average_comTime_uniDirLN_DP))
            #     print("average additional #actions: " + str(average_numAddActions_uniDirLN_DP))

            # print("Ultimate heuristic search: ")
            # print("success rate: " + str(total_successSolTimes_UltimateHeuristic) + "/" + str(total_num_instances))
            # if (total_successSolTimes_UltimateHeuristic != 0):
            #     print("average computation time: " + str(average_comTime_UltimateHeuristic))
            #     print("average additional #actions: " + str(average_numAddActions_UltimateHeuristic))


            f_stat = open(instance_folder + "/mRS_10_stat.txt", "w")
            if numObjs <= 10:
                f_stat.write(
                    format(average_numAddActions_UniDirmRS, '.1f') + " " + format(average_comTime_UniDirmRS, '.3f') + " " + str(success_rate_UniDirmRS))
                # f_stat.write(
                #     format(average_numAddActions_UniDirmRS, '.1f') + " " + format(average_comTime_UniDirmRS, '.3f') + " " + str(success_rate_UniDirmRS) + "\n")
            # f_stat.write(
            #     format(average_numAddActions_UniDirfmRS, '.1f') + " " + format(average_comTime_UniDirfmRS, '.3f') + " " + str(success_rate_UniDirfmRS) + "\n")
            # f_stat.write(
            #     format(average_numAddActions_uniDirDP, '.1f') + " " + format(average_comTime_uniDirDP, '.3f') + " " + str(success_rate_uniDirDP) + "\n")
            # f_stat.write(
            #     format(average_numAddActions_uniDirLN_DP, '.1f') + " " + format(average_comTime_uniDirLN_DP, '.3f') + " " + str(success_rate_uniDirLN_DP) + "\n")
            # f_stat.write(
            #     format(average_numAddActions_UltimateHeuristic, '.1f') + " " + format(average_comTime_UltimateHeuristic, '.3f') + " " + str(success_rate_UltimateHeuristic))

            f_stat.close()


        
