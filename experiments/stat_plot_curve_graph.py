from __future__ import division
import os
import IPython
import sys
import matplotlib.pyplot as plt
import numpy as np
plt.switch_backend('TKagg')

########## graph setting ##########
FIGSIZE_1 = (3.45, 1.85)
FIGSIZE = (2, 1.8)
FONTSIZE = 7
LABELSIZE = 6
# MARKERSIZE = 4
LINEWIDTH = 1
plt.rcParams["legend.labelspacing"] = 0.2
plt.rcParams["legend.handlelength"] = 1.75
plt.rcParams["legend.handletextpad"] = 0.5
plt.rcParams["legend.columnspacing"] = 0.75

plt.rcParams.update({'figure.autolayout': True})

plt.rcParams['ps.useafm'] = True
plt.rcParams['pdf.use14corefonts'] = True
plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = "serif"
plt.rcParams['font.serif'] = "Times"

plt.rcParams.update({'axes.titlesize': 'x-small'})
#####################################

numObjs = [5, 8, 10, 13, 15, 18, 20]
exp_settings = ['5_80_668_668', '8_80_845_845', '10_80_945_945', '13_80_1077_1077', \
                '15_80_1157_1157', '18_80_1268_1268', '20_80_1336_1336']
labels = ["5", "8", "10", "13", "15", "18", "20"]

nBuffs = sys.argv[1]
if nBuffs == "1":
    folderName = "one_buffer"
    plotTitle = "One-buffer cases"
    saveFile = "one_buffer"
    numObjs_for_mRS = [5, 8]
elif nBuffs == "2":
    folderName = "two_buffer"
    plotTitle = "Two-buffer cases"
    saveFile = "two_buffer"
    numObjs_for_mRS = [5, 8]
elif nBuffs == "3":
    folderName = "unsolvable"
    plotTitle = "Harder cases"
    saveFile = "harder_cases"
    exp_settings = ['8_80_845_845', '10_80_945_945', '13_80_1077_1077', \
                    '15_80_1157_1157', '18_80_1268_1268', '20_80_1336_1336']
    labels = ["8", "10", "13", "15", "18", "20"]
    numObjs = [8, 10, 13, 15, 18, 20]
    numObjs_for_mRS = [8]
else:
    print("please type one argument, either 1, 2 or 3")
    exit()

### ground truth
if nBuffs == "1":
    addActions_BF = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
elif nBuffs == "2":
    addActions_BF = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
else:
    addActions_BF = []


addActions_UniDirmRS = []
addActions_UniDirfmRS = []
addActions_uniDirDP = []
addActions_uniDirLN_DP = []
addActions_UltimateHeuristic = []

time_UniDirmRS = []
time_UniDirfmRS = []
time_uniDirDP = []
time_uniDirLN_DP = []
time_UltimateHeuristic = []

successRate_UniDirmRS = []
successRate_UniDirfmRS = []
successRate_uniDirDP = []
successRate_uniDirLN_DP = []
successRate_UltimateHeuristic = []

for setting_id, exp_setting in enumerate(exp_settings):
    fileName = os.path.join(os.getcwd(), "instances", exp_setting, folderName, "stat.txt")
    f_stat = open(fileName, 'r')
    counter = 1 ### count the line
    for line in f_stat:
        line = line.split()

        if numObjs[setting_id] < 10:
            if counter == 1:
                addActions_UniDirmRS.append(float(line[0]))
                time_UniDirmRS.append(float(line[1]))
                successRate_UniDirmRS.append(float(line[2])*100)
            if counter == 2:
                addActions_UniDirfmRS.append(float(line[0]))
                time_UniDirfmRS.append(float(line[1]))
                successRate_UniDirfmRS.append(float(line[2])*100)
            if counter == 3:
                addActions_uniDirDP.append(float(line[0]))
                time_uniDirDP.append(float(line[1]))
                successRate_uniDirDP.append(float(line[2])*100)
            if counter == 4:
                addActions_uniDirLN_DP.append(float(line[0]))
                time_uniDirLN_DP.append(float(line[1]))
                successRate_uniDirLN_DP.append(float(line[2])*100)
            if counter == 5:
                addActions_UltimateHeuristic.append(float(line[0]))
                time_UltimateHeuristic.append(float(line[1]))
                successRate_UltimateHeuristic.append(float(line[2])*100)

            counter += 1

        else:
            if counter == 1:
                addActions_UniDirfmRS.append(float(line[0]))
                time_UniDirfmRS.append(float(line[1]))
                successRate_UniDirfmRS.append(float(line[2])*100)
            if counter == 2:
                addActions_uniDirDP.append(float(line[0]))
                time_uniDirDP.append(float(line[1]))
                successRate_uniDirDP.append(float(line[2])*100)
            if counter == 3:
                addActions_uniDirLN_DP.append(float(line[0]))
                time_uniDirLN_DP.append(float(line[1]))
                successRate_uniDirLN_DP.append(float(line[2])*100)
            if counter == 4:
                addActions_UltimateHeuristic.append(float(line[0]))
                time_UltimateHeuristic.append(float(line[1]))
                successRate_UltimateHeuristic.append(float(line[2])*100)

            counter += 1

    f_stat.close()


################ additional actions #################
fig = plt.figure(1, figsize = FIGSIZE_1)
# ax = fig.add_subplot(111)
if nBuffs == "1" or nBuffs == "2":
    plt.plot(numObjs, addActions_BF, color='chocolate', marker="2", label="Optimal", linewidth = LINEWIDTH, markersize=3)
plt.plot(numObjs_for_mRS, addActions_UniDirmRS, color='orange', marker="p", label="RRT(mRS)", linewidth = LINEWIDTH, markersize=3)
plt.plot(numObjs, addActions_UniDirfmRS, color='cyan', marker="^", label="RRT(fmRS)", linewidth = LINEWIDTH, markersize=3)
plt.plot(numObjs, addActions_uniDirDP, color='green', marker="d", label="RRT(DFS-Rec) + random pertubation", linewidth = LINEWIDTH, markersize=3)
plt.plot(numObjs, addActions_uniDirLN_DP, color='darkviolet', marker="s", label="RRT(DFS-Rec) + super-node pertubation", linewidth = LINEWIDTH, markersize=3)
plt.plot(numObjs, addActions_UltimateHeuristic, color='red', marker="o", label="Dependency Heuristic Search", linewidth = LINEWIDTH, markersize=3)

# plt.xlim(1, 7)
# plt.ylim(-0.5, 2.0)
# plt.legend(ncol = 2, loc='upper right', fontsize = FONTSIZE)
plt.xlabel("Number of objects", fontsize = FONTSIZE, labelpad = 0)
plt.ylabel("Additional actions", fontsize = FONTSIZE)
# ax.tick_params(labelsize = FONTSIZE)
fig.savefig("./instances/" + saveFile + "_addActions_curve.eps", bbox_inches="tight", pad_inches=0.05)

fig.clear()
# plt.show()
#######################################################


################ time #################
fig = plt.figure(1, figsize = FIGSIZE_1)
# ax = fig.add_subplot(111)
plt.plot(numObjs_for_mRS, time_UniDirmRS, color='orange', marker="p", label="RRT(mRS)", linewidth = LINEWIDTH, markersize=3)
plt.plot(numObjs, time_UniDirfmRS, color='cyan', marker="^", label="RRT(fmRS)", linewidth = LINEWIDTH, markersize=3)
plt.plot(numObjs, time_uniDirDP, color='green', marker="d", label="RRT(DFS-Rec) + random pertubation", linewidth = LINEWIDTH, markersize=3)
plt.plot(numObjs, time_uniDirLN_DP, color='darkviolet', marker="s", label="RRT(DFS-Rec) + super-node pertubation", linewidth = LINEWIDTH, markersize=3)
plt.plot(numObjs, time_UltimateHeuristic, color='red', marker="o", label="Dependency Heuristic Search", linewidth = LINEWIDTH, markersize=3)

# plt.xlim(1, 7)
# plt.ylim(-0.5, 2.0)
# plt.legend(ncol = 2, loc='upper right', fontsize = FONTSIZE)
plt.xlabel("Number of objects", fontsize = FONTSIZE, labelpad = 0)
plt.ylabel("Time(s)", fontsize = FONTSIZE)
# ax.tick_params(labelsize = FONTSIZE)
fig.savefig("./instances/" + saveFile + "_time_curve.eps", bbox_inches="tight", pad_inches=0.05)

fig.clear()
# plt.show()
#######################################################
