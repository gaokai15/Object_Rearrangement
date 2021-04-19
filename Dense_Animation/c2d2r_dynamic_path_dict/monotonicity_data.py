# Import deepcopy for copying data structures
from copy import deepcopy
import numpy as np
import matplotlib.pyplot as plt
import random       # Just for demo purpose
import os
import cPickle as pickle

my_path = os.path.abspath(os.path.dirname(__file__))
KEY_WORDS = "Monotone"
DATE = "1019"

obj_nums = [10, 20, 30]

'''
Style elements
    Here we can setup a drawing style for each algorithm.
    This helps to make the drawing style of an algorithm consistent.
'''
defaultStyle = {
    # 'label' : 'default',    # The name of the algorithm
    'ls' : '-',             # Line style, '-' means a solid line
    'linewidth' : 1,        # Line width
    'zorder' : 100,         # The 'height' of the plot. 
                            # Affects whether items are in the front of / behind each other.
    # You can add more style items here, e.g., markers.
}
# Here, we setup the style for an algorithm. Let's call it Alg.1.
alg1Style = deepcopy(defaultStyle)          # First copy all default styles.
# alg1Style['label'] = r'\textsc{Optimal}'    # Setup algorithm name. 
                                            # We use \textsc here which is a latex command. 
                                            # This is fine since we use latex to generate text.
                                            # https://matplotlib.org/3.1.0/gallery/color/named_colors.html
# Another algorithm
alg2Style = deepcopy(defaultStyle)
alg2Style['label'] = r'\textsc{Greedy}'
alg2Style['color'] = 'tab:green'

''' Some global variables '''
FIGURE_SIZE = (10, 7)      # Figure width and height. 
                                # This is a good value for 2-column paper.
FONT_SIZE = 28                   # Size of text
LEGEND_FONT_SIZE = 18            # We might need different font sizes for different text

''' 
Parameters for legend to better utilize space.
'''
plt.rcParams["legend.labelspacing"] = 0.2
plt.rcParams["legend.handlelength"] = 1.75
plt.rcParams["legend.handletextpad"] = 0.5
plt.rcParams["legend.columnspacing"] = 0.75

plt.rcParams.update({'figure.autolayout': True})

'''
Use latex to generate text
Note that these params usually make the code slow. If you want to preview the figure without generating latex text, feel free to comment these. 
'''
plt.rcParams['ps.useafm'] = True
plt.rcParams['pdf.use14corefonts'] = True
plt.rcParams['text.usetex'] = True
plt.rcParams['font.family'] = "serif"
plt.rcParams['font.serif'] = "Times"

''' The real drawing part starts here. '''
# Put your data over here.
# x = [i for i in range(10, 100, 10)]
# alg1ComputationTime = [2**i for i in range(1, 10)]
# alg2ComputationTime = [10 * i for i in range(1, 10)]
# alg1StdDev = [random.random() * i * 2 for i in range(1, 10)]
# alg2StdDev = [random.random() * i * 2 for i in range(1, 10)]
# # Start to create the figure
# fig = plt.figure(figsize = FIGURE_SIZE)
# ax = fig.add_subplot(111)
# ax.plot(x, alg1ComputationTime, **alg1Style)
# ax.plot(x, alg2ComputationTime, **alg2Style)
# ax.errorbar(x, alg1ComputationTime, yerr = alg1StdDev, color = alg1Style['color'], capsize = 2, ls = 'none', markeredgewidth = 1, elinewidth = 1)
# ax.errorbar(x, alg2ComputationTime, yerr = alg2StdDev, color = alg2Style['color'], capsize = 2, ls = 'none', markeredgewidth = 1, elinewidth = 1)
# # Set x and y label. We use latex to generate text
# ax.set_xlabel(r"Number of Robots $(n)$", fontsize = FONT_SIZE)
# ax.set_ylabel("Computation Time (s)", fontsize = FONT_SIZE)
# ax.tick_params(labelsize = FONT_SIZE)
# ax.legend(fontsize = LEGEND_FONT_SIZE, ncol = 2)
# ax.set_yscale("log")
# ax.yaxis.grid(True, alpha = 0.8)
# # Directly save the figure to a file.
# fig.savefig("result-computation-time.pdf", bbox_inches="tight", pad_inches=0.05)
# plt.show()
# plt.cla()

n10 = []
n20 = []
n30 = []

''' Another bar chart. '''
# Put your data over here.
for D in [2,4]:
    for n in [10, 20, 30]:
        with open(os.path.join(my_path, "data/Experiment_1030_D"+str(D)+"_"+str(n)+".pkl"), 'rb') as input:
            IP_data, DFS_rec_data, MCR_data, MRS_data, IP_ans_data, DFS_rec_ans_data, MCR_ans_data, MRS_ans_data = pickle.load(input)

        M = 0.0
        N = 0.0
        F = 0.0
        Total = 0.0

        for e in DFS_rec_ans_data[n]:
            Total+=1
            if e == 'M':
                M+=1
            elif e == 'N':
                N+=1
            else:
                F+=1

        print D, n, M/Total, N/Total, F/Total



# IP_data_average = []
# DFS_rec_data_average = []
# MCR_data_average = []
# MRS_data_average = []
# for num in obj_nums:
#     IP_data_average.append(np.average(IP_data[num]))
#     DFS_rec_data_average.append(np.average(DFS_rec_data[num]))
#     MCR_data_average.append(np.average(MCR_data[num]))
#     MRS_data_average.append(np.average(MRS_data[num]))

# IP_ans_data_average = []
# DFS_rec_ans_data_average = []
# MCR_ans_data_average = []
# MRS_ans_data_average = []
# for num in obj_nums:
#     count = 0
#     for e in IP_ans_data[num]:
#         if e != "F":
#             count+=1.0/len(IP_ans_data[num])
#     IP_ans_data_average.append(count)
#     count = 0
#     for e in DFS_rec_ans_data[num]:
#         if e != "F":
#             count+=1.0/len(DFS_rec_ans_data[num])
#     DFS_rec_ans_data_average.append(count)
#     count = 0
#     for e in MCR_ans_data[num]:
#         if e != "F":
#             count+=1.0/len(MCR_ans_data[num])
#     if num == 30:
#         count -= 2.0/len(MCR_ans_data[num])
#     MCR_ans_data_average.append(count)
#     count = 0
#     for e in MRS_ans_data[num]:
#         if e != "F":
#             count+=1.0/len(MRS_ans_data[num])
#     MRS_ans_data_average.append(count)

# print IP_data_average
# print DFS_rec_data_average
# print MCR_data_average
# print MRS_data_average

# # Start to create the figure
# fig = plt.figure(figsize = FIGURE_SIZE)
# ax1 = fig.add_subplot(2,2,2)
# ax1.set_xticks(obj_nums)
# ax1.set_yscale('log')
# width = 2.0
# ax1.yaxis.grid(True, alpha = 0.8)
# ax1.bar([x - 1.5*width for x in obj_nums], [IP_data_average[0]]+([10e-5]*2), width, label='IP-Solver', color = 'b', edgecolor = 'black', **alg1Style)
# ax1.bar([x - 0.5*width for x in obj_nums], MCR_data_average, width, label='fmRS', color =  'c', edgecolor = 'black', **alg1Style)
# ax1.bar([x + 0.5*width for x in obj_nums], MRS_data_average, width, label='mRS', color = 'tab:orange', edgecolor = 'black', **alg1Style)
# ax1.bar([x + 1.5*width for x in obj_nums], DFS_rec_data_average, width, label='DFS-rec', color = 'g', edgecolor = 'black', **alg1Style)
# ax1.set_ylim([1e-3, 1e3])
# # ax.set_xticks(x)
# # ax.set_xlabel(r"Number of Robots $(n)$", fontsize = FONT_SIZE)
# # ax.set_ylabel("Optimality Ratio", fontsize = FONT_SIZE)
# ax1.tick_params(labelsize = FONT_SIZE)



# ax2 = fig.add_subplot(2,2,1)
# ax2.set_xticks(obj_nums)
# # ax1.set_yscale('log')
# width = 2.0
# ax2.yaxis.grid(True, alpha = 0.8)
# ax2.bar([x - 1.5*width for x in obj_nums], IP_ans_data_average, width, label='IP-Solver', color = 'b', edgecolor = 'black', **alg1Style)
# ax2.bar([x - 0.5*width for x in obj_nums], MCR_ans_data_average, width, label='fmRS', color =  'c', edgecolor = 'black', **alg1Style)
# ax2.bar([x + 0.5*width for x in obj_nums], MRS_ans_data_average, width, label='mRS', color = 'tab:orange', edgecolor = 'black', **alg1Style)
# ax2.bar([x + 1.5*width for x in obj_nums], DFS_rec_ans_data_average, width, label='DFS-rec', color = 'g',edgecolor = 'black', **alg1Style)
# ax2.set_ylim([0.0, 1.2])
# ax2.tick_params(labelsize = FONT_SIZE)






# with open(os.path.join(my_path, "data/Experiment_1030_D4_30.pkl"), 'rb') as input:
#     IP_data, DFS_rec_data, MCR_data, MRS_data, IP_ans_data, DFS_rec_ans_data, MCR_ans_data, MRS_ans_data = pickle.load(input)

# IP_data_average = []
# DFS_rec_data_average = []
# MCR_data_average = []
# MRS_data_average = []
# for num in obj_nums:
#     IP_data_average.append(np.average(IP_data[num]))
#     DFS_rec_data_average.append(np.average(DFS_rec_data[num]))
#     MCR_data_average.append(np.average(MCR_data[num]))
#     MRS_data_average.append(np.average(MRS_data[num]))

# IP_ans_data_average = []
# DFS_rec_ans_data_average = []
# MCR_ans_data_average = []
# MRS_ans_data_average = []
# for num in obj_nums:
#     count = 0
#     for e in IP_ans_data[num]:
#         if e != "F":
#             count+=1.0/len(IP_ans_data[num])
#     IP_ans_data_average.append(count)
#     count = 0
#     for e in DFS_rec_ans_data[num]:
#         if e != "F":
#             count+=1.0/len(DFS_rec_ans_data[num])
#     DFS_rec_ans_data_average.append(count)
#     count = 0
#     for e in MCR_ans_data[num]:
#         if e != "F":
#             count+=1.0/len(MCR_ans_data[num])
#     MCR_ans_data_average.append(count)
#     count = 0
#     for e in MRS_ans_data[num]:
#         if e != "F":
#             count+=1.0/len(MRS_ans_data[num])
#     MRS_ans_data_average.append(count)

# print IP_data_average
# print DFS_rec_data_average
# print MCR_data_average
# print MRS_data_average

# # Start to create the figure
# ax3 = fig.add_subplot(2,2,4)
# ax3.set_xticks(obj_nums)
# ax3.set_yscale('log')
# width = 2.0
# ax3.yaxis.grid(True, alpha = 0.8)
# ax3.bar([x - 1.5*width for x in obj_nums], [IP_data_average[0]]+([10e-5]*2), width, label='IP-Solver', color = 'b', edgecolor = 'black', **alg1Style)
# ax3.bar([x - 0.5*width for x in obj_nums], MCR_data_average, width, label='fmRS', color =  'c', edgecolor = 'black', **alg1Style)
# ax3.bar([x + 0.5*width for x in obj_nums], MRS_data_average, width, label='mRS', color = 'tab:orange', edgecolor = 'black', **alg1Style)
# ax3.bar([x + 1.5*width for x in obj_nums], DFS_rec_data_average, width, label='DFS-rec', color = 'g', edgecolor = 'black', **alg1Style)
# ax3.set_ylim([1e-3, 1e3])
# # ax.set_xticks(x)
# # ax.set_xlabel(r"Number of Robots $(n)$", fontsize = FONT_SIZE)
# # ax.set_ylabel("Optimality Ratio", fontsize = FONT_SIZE)
# ax3.tick_params(labelsize = FONT_SIZE)



# ax4 = fig.add_subplot(2,2,3)
# ax4.set_xticks(obj_nums)
# # ax1.set_yscale('log')
# width = 2.0
# ax4.yaxis.grid(True, alpha = 0.8)
# ax4.bar([x - 1.5*width for x in obj_nums], IP_ans_data_average, width, label='IP-Solver', color = 'b', edgecolor = 'black', **alg1Style)
# ax4.bar([x - 0.5*width for x in obj_nums], MCR_ans_data_average, width, label='fmRS', color =  'c', edgecolor = 'black', **alg1Style)
# ax4.bar([x + 0.5*width for x in obj_nums], MRS_ans_data_average, width, label='mRS', color = 'tab:orange', edgecolor = 'black', **alg1Style)
# ax4.bar([x + 1.5*width for x in obj_nums], DFS_rec_ans_data_average, width, label='DFS-rec', color = 'g',edgecolor = 'black', **alg1Style)
# ax4.set_ylim([0.0, 1.2])
# ax4.tick_params(labelsize = FONT_SIZE)





# # plt.legend(fontsize = LEGEND_FONT_SIZE, ncol = 4, loc='upper center', bbox_to_anchor=(0.5, -0.05), fancybox=True, shadow=True)
# # plt.savefig(my_path + "/pictures/mono-D4-rate.eps")
# # plt.savefig(my_path + "/pictures/mono-D4-time.eps")
# plt.savefig(my_path + "/pictures/mono-total.eps")
# plt.show()
# plt.cla()