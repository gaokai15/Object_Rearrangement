import os
import sys
import copy
from collections import OrderedDict
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

import matplotlib
# matplotlib.use('Agg')
from matplotlib import pyplot as plt

import math

import cPickle as pickle

import IPython

import time

my_path = os.path.abspath(os.path.dirname(__file__))
data_collection = []

with open(os.path.join(my_path, "Experiments05.pkl"),
            'rb') as input:
    data = pickle.load(input)
data_collection = data_collection + data
with open(os.path.join(my_path, "Experiments07.pkl"),
            'rb') as input:
    data = pickle.load(input)
data_collection = data_collection + data
with open(os.path.join(my_path, "Experiments09.pkl"),
            'rb') as input:
    data = pickle.load(input)
data_collection = data_collection + data
with open(os.path.join(my_path, "Experiments11.pkl"),
            'rb') as input:
    data = pickle.load(input)
data_collection = data_collection + data
print data_collection
mem = 0
time = 0
for data in data_collection:
    if data[3] == False:
        mem +=1
    elif data[6] == False:
        time +=1
print mem
print time
# max_max_tours = 0
# min_max_tours = 200
# max_total_tours = 0
# min_total_tours = 2000
# for data in data_collection:
#     if data[3]:
#         max_tours = data[4]
#         print "max_tours", max_tours
#         if max_max_tours <= max_tours:
#             max_max_tours = max_tours
#         if min_max_tours >= max_tours:
#             min_max_tours = max_tours
#         total_tours = data[5]
#         print "total_tours", total_tours
#         if max_total_tours <= total_tours:
#             max_total_tours = total_tours
#         if min_total_tours >= total_tours:
#             min_total_tours = total_tours

# print max_max_tours
# print min_max_tours
# print max_total_tours
# print min_total_tours

# max_tour_dictionary = {}
# for data in data_collection:
#     max_tours = data[0]
#     if max_tours not in max_tour_dictionary:
#         max_tour_dictionary[max_tours] = [0,0]
#     if data[3] == False:
#         max_tour_dictionary[max_tours][1] += 1
#         continue
#     if (data[6] == False) and (data[-1] >200):
#         max_tour_dictionary[max_tours][1] += 1
#         continue
#     max_tour_dictionary[max_tours][0] += 1

# # X = sorted(max_tour_dictionary.keys())

# print max_tour_dictionary
# c1 = [0,0]
# c2 = [0,0]
# c3 = [0,0]
# c4 = [0,0]

# for i in X:
#     if i <=200:
#         c1[0] += max_tour_dictionary[i][0]
#         c1[1] += max_tour_dictionary[i][1]
#     elif i <=300:
#         c2[0] += max_tour_dictionary[i][0]
#         c2[1] += max_tour_dictionary[i][1]
#     elif i <=400:
#         c3[0] += max_tour_dictionary[i][0]
#         c3[1] += max_tour_dictionary[i][1]
#     else:
#         c4[0] += max_tour_dictionary[i][0]
#         c4[1] += max_tour_dictionary[i][1]

# print c1
# print c2
# print c3
# print c4
    

