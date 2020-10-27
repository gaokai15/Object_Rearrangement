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
elif nBuffs == "2":
    folderName = "two_buffer"
    plotTitle = "Two-buffer cases"
    saveFile = "two_buffer"
elif nBuffs == "3":
    folderName = "unsolvable"
    plotTitle = "Harder cases"
    saveFile = "harder_cases"
    exp_settings = ['8_80_845_845', '10_80_945_945', '13_80_1077_1077', \
                    '15_80_1157_1157', '18_80_1268_1268', '20_80_1336_1336']
    labels = ["8", "10", "13", "15", "18", "20"]
    numObjs = [8, 10, 13, 15, 18, 20]
else:
    print("please type one argument, either 1, 2 or 3")
    exit()


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

if nBuffs == "1" or nBuffs == "2":
    addActions_UniDirmRS += [0.0, 0.0, 0.0, 0.0, 0.0]
    time_UniDirmRS += [0.0, 0.0, 0.0, 0.0, 0.0]
    successRate_UniDirmRS += [0.0, 0.0, 0.0, 0.0, 0.0]
else:
    addActions_UniDirmRS += [0.0, 0.0, 0.0, 0.0]
    time_UniDirmRS += [0.0, 0.0, 0.0, 0.0]
    successRate_UniDirmRS += [0.0, 0.0, 0.0, 0.0]    


bar_width = 16
x = np.array([i * 100 + 100 for i in range(0, len(numObjs))])

print(x)


################ additional actions #################
fig = plt.figure(1, figsize = FIGSIZE_1)
ax = fig.add_subplot(111)
ax.bar(x - bar_width * 2.0, addActions_UniDirmRS, bar_width, label="RRT(mRS)", edgecolor="black", color="orange")
ax.bar(x - bar_width * 1.0, addActions_UniDirfmRS, bar_width, label="RRT(fmRS)", edgecolor="black", color="cyan")
ax.bar(x - bar_width * 0.0, addActions_uniDirDP, bar_width, label="RRT(DFS-Rec) + random pertubation", edgecolor="black", color="green")
ax.bar(x + bar_width * 1.0, addActions_uniDirLN_DP, bar_width, label="RRT(DFS-Rec) + super-node pertubation", edgecolor="black", color="darkviolet")
ax.bar(x + bar_width * 2.0, addActions_UltimateHeuristic, bar_width, label="Dependency Heuristic Search", edgecolor="black", color="red")
if nBuffs == "1":
    ax.axhline(1, color="chocolate", linestyle='--', linewidth=0.3, label="Optimal")
elif nBuffs == "2":
    ax.axhline(2, color="chocolate", linestyle='--', linewidth=0.3, label="Optimal")
else:
    pass
ax.set_xlabel("Number of objects", fontsize = FONTSIZE)
# labels = ["5", "8", "10", "13", "15", "18", "20"]
# ax.get_xaxis().set_visible(False)
ax.set_ylabel("Additional actions", fontsize = FONTSIZE)
ax.tick_params(labelsize = FONTSIZE)
ax.set_ylim(0, 8)
# ax.legend(fontsize = LABELSIZE, ncol = 6)
ax.yaxis.grid(True, alpha = 0.99)
ax.set_axisbelow(True)
# ax.xaxis.set_label_coords(0.5, -0.15)    
ax.set_xticks(x)
ax.set_xticklabels(labels)
ax.title.set_text(plotTitle)
fig.savefig("./instances/" + saveFile + "_addActions_bar.eps", bbox_inches="tight", pad_inches=0.05)

fig.clear()
# plt.show()
#######################################################


################ time #################
fig = plt.figure(1, figsize = FIGSIZE_1)
ax = fig.add_subplot(111)
ax.bar(x - bar_width * 2.0, time_UniDirmRS, bar_width, label="RRT(mRS)", edgecolor="black", color="orange")
ax.bar(x - bar_width * 1.0, time_UniDirfmRS, bar_width, label="RRT(fmRS)", edgecolor="black", color="cyan")
ax.bar(x - bar_width * 0.0, time_uniDirDP, bar_width, label="RRT(DFS-Rec) + random pertubation", edgecolor="black", color="green")
ax.bar(x + bar_width * 1.0, time_uniDirLN_DP, bar_width, label="RRT(DFS-Rec) + super-node pertubation", edgecolor="black", color="darkviolet")
ax.bar(x + bar_width * 2.0, time_UltimateHeuristic, bar_width, label="Dependency Heuristic Search", edgecolor="black", color="red")

ax.set_xlabel("Number of objects", fontsize = FONTSIZE)
# labels = ["5", "8", "10", "13", "15", "18", "20"]
# ax.get_xaxis().set_visible(False)
ax.set_ylabel("Time(s)", fontsize = FONTSIZE)
ax.tick_params(labelsize = FONTSIZE)
ax.set_yscale('log')
# ax.set_ylim(0, 140)
# ax.legend(fontsize = LABELSIZE, ncol = 1)
ax.yaxis.grid(True, alpha = 0.99)
ax.set_axisbelow(True)
# ax.xaxis.set_label_coords(0.5, -0.15)    
ax.set_xticks(x)
ax.set_xticklabels(labels)
ax.title.set_text(plotTitle)
fig.savefig("./instances/" + saveFile + "_time_bar.eps", bbox_inches="tight", pad_inches=0.05)

fig.clear()
# plt.show()
#######################################################


################ success rate #################
fig = plt.figure(1, figsize = FIGSIZE_1)
ax = fig.add_subplot(111)
ax.bar(x - bar_width * 2.0, successRate_UniDirmRS, bar_width, label="RRT(mRS)", edgecolor="black", color="orange")
ax.bar(x - bar_width * 1.0, successRate_UniDirfmRS, bar_width, label="RRT(fmRS)", edgecolor="black", color="cyan")
ax.bar(x - bar_width * 0.0, successRate_uniDirDP, bar_width, label="RRT(DFS-Rec) + random pertubation", edgecolor="black", color="green")
ax.bar(x + bar_width * 1.0, successRate_uniDirLN_DP, bar_width, label="RRT(DFS-Rec) + super-node pertubation", edgecolor="black", color="darkviolet")
ax.bar(x + bar_width * 2.0, successRate_UltimateHeuristic, bar_width, label="Dependency Heuristic Search", edgecolor="black", color="red")

ax.set_xlabel("Number of objects", fontsize = FONTSIZE)
# labels = ["5", "8", "10", "13", "15", "18", "20"]
# ax.get_xaxis().set_visible(False)
ax.set_ylabel("Success rate (\%)", fontsize = FONTSIZE)
ax.tick_params(labelsize = FONTSIZE)
ax.set_ylim(0, 105)
# ax.legend(fontsize = LABELSIZE, ncol = 1)
ax.yaxis.grid(True, alpha = 0.99)
ax.set_axisbelow(True)
# ax.xaxis.set_label_coords(0.5, -0.15)    
ax.set_xticks(x)
ax.set_xticklabels(labels)
ax.title.set_text(plotTitle)
fig.savefig("./instances/" + saveFile + "_success_bar.eps", bbox_inches="tight", pad_inches=0.05)

fig.clear()
# plt.show()
#######################################################
