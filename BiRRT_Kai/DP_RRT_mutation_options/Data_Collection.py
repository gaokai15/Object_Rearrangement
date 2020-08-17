# Import deepcopy for copying data structures
from copy import deepcopy
import numpy as np
import matplotlib.pyplot as plt
import random       # Just for demo purpose
import os
import cPickle as pickle


my_path = os.path.abspath(os.path.dirname(__file__))
'''
Style elements
    Here we can setup a drawing style for each algorithm.
    This helps to make the drawing style of an algorithm consistent.
'''
defaultStyle = {
    'label' : 'default',    # The name of the algorithm
    'ls' : '-',             # Line style, '-' means a solid line
    'linewidth' : 1,        # Line width
    'color' : 'k',          # Line color, 'k' means color
    'zorder' : 100,         # The 'height' of the plot. 
                            # Affects whether items are in the front of / behind each other.
    # You can add more style items here, e.g., markers.
}
# Here, we setup the style for an algorithm. Let's call it Alg.1.
alg1Style = deepcopy(defaultStyle)          # First copy all default styles.
alg1Style['label'] = r'\textsc{Optimal}'    # Setup algorithm name. 
                                            # We use \textsc here which is a latex command. 
                                            # This is fine since we use latex to generate text.
alg1Style['color'] = 'tab:orange'           # Customized line color
                                            # https://matplotlib.org/3.1.0/gallery/color/named_colors.html
# Another algorithm
alg2Style = deepcopy(defaultStyle)
alg2Style['label'] = r'\textsc{Leaf-Root}'
alg2Style['color'] = 'tab:green'

# Another algorithm
alg3Style = deepcopy(defaultStyle)
alg3Style['label'] = r'\textsc{Leaf-Root-Improved-Perturbation}'
alg3Style['color'] = 'c'

# Another algorithm
alg4Style = deepcopy(defaultStyle)
alg4Style['label'] = r'\textsc{Optimal}'
alg4Style['color'] = 'tab:blue'


''' Some global variables '''
FIGURE_SIZE = (10, 5)      # Figure width and height. 
                                # This is a good value for 2-column paper.
FONT_SIZE = 18                   # Size of text
LEGEND_FONT_SIZE = 12            # We might need different font sizes for different text

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
# with open(os.path.join(my_path, "Experiment_larger_range_0625.pkl"), 'rb') as input:
#         DP_data, DP_DFS_data, True_rate_data, DP_mem_data, DP_DFS_mem_data = pickle.load(input)

# Time
Time = []
# Time.append([26.0, 28.0, 28.0, 26.0, 29.0, 30.0, 26.0])
# Time.append([26.0, 26.0, 26.0, 26.0, 26.0, 26.0, -0.0])
# Time.append([28.0, 26.0, 29.0, 26.0, 28.0, 26.0, 26.0])
# Time.append([-0.0, -0.0, -0.0, -0.0, -0.0, -0.0, 25.0])
# Time.append([28.0, 27.0, 28.0, 27.0, 27.0, 30.0, 26.0])
# Time.append([26.0, 26.0, 26.0, 27.0, 26.0, 29.0, 26.0])
# Time.append([34.0, 32.0, 31.0, 28.0, 30.0, 31.0, -0.0])
# Time.append([25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 25.0])
# Time.append([29.0, 32.0, 28.0, 30.0, 26.0, 30.0, 26.0])
# Time.append([34.0, 27.0, 33.0, 32.0, 33.0, 28.0, -0.0])

# Time.append([11.103789, 71.965312, 4.670135, 0.001])
# Time.append([48.295397, 2.969197, 6.437553, 291.327483])
# Time.append([31.02628, 30.303285, 19.91159, 0.001])
# Time.append([18.607245, 5.30471, 22.962513, 0.001])
# Time.append([17.416609, 8.717555, 8.767762, 12.607976])
# Time.append([14.426064, 26.994346, 13.922365, 111.162863])
# Time.append([205.256389, 145.299882, 128.088478, 0.001])
# Time.append([314.878836, 311.993669, 312.825, 47.34573])
# Time.append([45.860972, 45.261881, 45.350243, 0.0493689999994])
# Time.append([185.79877, 185.09057, 186.822499, 23.523532])

# Time.append([105.0, 52.0, 0.001, 0.001, 0.001])
# Time.append([9.341898, 16.502421, 11.625287, 2.152493, 0.303463])
# Time.append([63.97721, 134.055529, 0.001, 0.001, 0.001])
# Time.append([0.001, 0.001, 0.001, 0.001, 0.001])
# Time.append([65.030341, 25.88344, 0.001, 3.841368, 997.096543])
# Time.append([0.001, 46.177415, 0.001, 36.950259, 0.001])
# Time.append([102.050534, 110.365625, 0.001, 52.581009, 0.001])
# Time.append([12.720973, 54.371995, 0.001, 768.488938, 0.001])
# Time.append([45.177591, 55.993727, 0.001, 23.680832, 0.001])
# Time.append([35.36101, 120.068339, 0.001, 4.268243, 92.178258])

Time.append([3.185147, 3.168417, 0.074186])
Time.append([0.001, 0.001, 2.004004])
Time.append([11.851682, 22.978859, 0.001])
Time.append([38.586965, 38.379061, 5.837838])
Time.append([18.563968, 17.377276, 0.001])
Time.append([54.270698, 54.047206, 0.623507])
Time.append([488.126747, 478.259764, 23.90231])
Time.append([65.654711, 61.039525, 0.001])
Time.append([4.628409, 3.499696, 60.466723])
Time.append([10.474342, 7.104801, 9.26449])


# Actions
Actions = []
# Actions.append([26.023722, 31.495505, 38.689191, 38.93439, 24.141227, 43.665495, 5.811535])
# Actions.append([264.131774, 424.610777, 392.421557, 394.451759, 394.610105, 395.230088, 0.001])
# Actions.append([13.713163, 10.344733, 28.138233, 33.065327, 13.028133, 11.615503, 7.152138])
# Actions.append([0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.514687999999])
# Actions.append([5.304983, 1.955873, 3.318189, 9.028893, 12.717948, 2.769185, 161.940985])
# Actions.append([34.808254, 33.638817, 32.810842, 30.668179, 33.076919, 48.48726, 521.650255])
# Actions.append([102.855533, 9.361407, 8.837853, 15.117354, 8.434415, 61.53763, 0.001])
# Actions.append([48.321761, 56.980628, 56.087228, 56.254119, 56.364565, 56.133938, 7.691841])
# Actions.append([11.722536, 34.232728, 14.433505, 13.836496, 2.421577, 22.783789, 40.326759])
# Actions.append([37.793608, 11.831719, 13.699389, 30.54787, 9.992101, 14.400874, 0.001])
# Actions.append([26.0, 26.0, 26.0, -0.0])
# Actions.append([31.0, 37.0, 34.0, -0.0])
# Actions.append([26.0, 30.0, 26.0, 26.0])
# Actions.append([31.0, 37.0, 34.0, -0.0])
# Actions.append([26.0, 30.0, 26.0, 26.0])
# Actions.append([26.0, 28.0, 26.0, 26.0])
# Actions.append([27.0, 29.0, 37.0, -0.0])
# Actions.append([25.0, 25.0, 25.0, 25.0])
# Actions.append([25.0, 25.0, 25.0, 25.0])
# Actions.append([25.0, 25.0, 25.0, 25.0])
# Actions.append([])
# Actions.append([31.0, 37.0, 34.0, -0.0])
# Actions.append([26.0, 30.0, 26.0, 26.0])
# Actions.append([31.0, 37.0, 34.0, -0.0])
# Actions.append([26.0, 30.0, 26.0, 26.0])
# Actions.append([26.0, 28.0, 26.0, 26.0])
# Actions.append([27.0, 29.0, 37.0, -0.0])
# Actions.append([25.0, 25.0, 25.0, 25.0])
# Actions.append([25.0, 25.0, 25.0, 25.0])
# Actions.append([25.0, 25.0, 25.0, 25.0])

# mutation experiment
# Actions.append([25.0, 25.0, 25.0])
# Actions.append([0.0, 0.0, 0.0])
# Actions.append([28.0, 28.0, -0.0])
# Actions.append([25.0, 25.0, 25.0])
# Actions.append([27.0, 27.0, -0.0])
# Actions.append([25.0, 25.0, 25.0])
# Actions.append([25.0, 25.0, 25.0])
# Actions.append([26.0, 26.0, -0.0])
# Actions.append([26.0, 26.0, 26.0])
# Actions.append([29.0, 29.0, 26.0])




bar_width = 0.4
# Start to create the figure
fig = plt.figure(figsize = FIGURE_SIZE)
ax = fig.add_subplot(111)
ax.set_xticks(range(10))
ax.set_ylabel('Time(secs)')
ax.set_yscale('log')
# ax.set_ylabel('Actions')
# ax.bar([x - 1.5*bar_width for x in range(10)], [Time[i][0] for i in range(len(Time))], bar_width, edgecolor = 'black', **alg1Style)
ax.bar([x - 0.5*bar_width for x in range(10)], [Time[i][0] for i in range(len(Time))], bar_width, edgecolor = 'black', **alg2Style)
ax.bar([x + 0.5*bar_width for x in range(10)], [Time[i][1] for i in range(len(Time))], bar_width, edgecolor = 'black', **alg3Style)
# ax.bar([x + 1.5*bar_width for x in range(10)], [Time[i][3] for i in range(len(Time))], bar_width, edgecolor = 'black', **alg4Style)
# ax.bar([x - 1.0*bar_width for x in range(10)], [Actions[i][0] for i in range(len(Time))], bar_width, edgecolor = 'black', **alg2Style)
# ax.bar([x - 0.0*bar_width for x in range(10)], [Actions[i][1] for i in range(len(Time))], bar_width, edgecolor = 'black', **alg3Style)
# ax.bar([x + 1.0*bar_width for x in range(10)], [Actions[i][2] for i in range(len(Time))], bar_width, edgecolor = 'black', **alg1Style)
# ax.bar([x + 1.5*bar_width for x in range(10)], [Actions[i][3] for i in range(len(Time))], bar_width, edgecolor = 'black', **alg4Style)
ax.set_ylim([10**(-2), 2000])
# ax.set_ylim([0, 35])
# ax.bar(x, True_rate, bar_width, edgecolor = 'black', **alg1Style)
# Set x and y label. We use latex to generate text
ax.set_xlabel("Instance Index", fontsize = FONT_SIZE)
ax.legend(fontsize = LEGEND_FONT_SIZE, ncol = 4)
# ax.yaxis.grid(True, alpha = 0.8)
# Directly save the figure to a file.
# fig.savefig("result-computation-time.pdf", bbox_inches="tight", pad_inches=0.05)







plt.show()
plt.cla()

''' Another bar chart. '''
# Put your data over here.
# x = np.array([i for i in range(-200000,200001, 40000)])
# alg1OptimalityRatio = [1 for i in range(1, 10)]
# alg2OptimalityRatio = [0.1 * i + 1 for i in range(1, 10)]
# # Start to create the figure
# fig = plt.figure(figsize = FIGURE_SIZE)
# ax = fig.add_subplot(111)
# bar_width = 15000
# ax.bar(x - bar_width / 2, DP_mem_avg, bar_width, edgecolor = 'black', **alg2Style)
# ax.bar(x + bar_width / 2, DP_DFS_mem_avg, bar_width, edgecolor = 'black', **alg3Style)
# ax.set_xticks(x)
# ax.set_xlabel(r"Sum", fontsize = FONT_SIZE)
# ax.set_ylabel("Memory", fontsize = FONT_SIZE)
# ax.set_yscale('log')
# ax.tick_params(labelsize = FONT_SIZE)
# ax.legend(fontsize = LEGEND_FONT_SIZE, ncol = 1)
# ax.yaxis.grid(True, alpha = 0.8)
# # Directly save the figure to a file.
# fig.savefig("result-optimality-ratio.pdf", bbox_inches="tight", pad_inches=0.05)
# plt.show()
# plt.cla()