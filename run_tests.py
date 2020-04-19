#!/usr/bin/env python2
from __future__ import print_function

import os
from random import random as rng
from c2d2r.cgraph import *
from c2d2r.util import *

numObjs = 100
RAD = 40
HEIGHT = 1000
WIDTH = 1000
display = False
displayMore = False
saveDir = "workspaces"
numTrials = 10
density = 0.5

for trial in range(numTrials):
    print('\n', trial)
    tmpfile = saveDir + "/TEMP"
    didgen = False
    while not didgen:
        didgen = genDenseCGraph(numObjs, RAD, HEIGHT, WIDTH, display, displayMore, tmpfile, True)
    graph, paths, objects, obj2reg, regions, polygon = didgen
    print("Generated Instance. Starting Tests...")
    c = 0
    for rind, r in regions.items():
        c += 1.0 / len(regions)
        print('\r{:.2%}'.format(c), end='')
        r, center = r
        num_samples = int(r.area() * density)
        num_samples += 5
        collisions = set()
        for s in range(num_samples):
            objAt = np.add(r.sample(rng), polygon)
            for i in range(len(objects)):
                if polysCollide(objects[i], objAt):
                    collisions.add(i)
        exp_collisions = set(rind[:-1])
        if collisions != exp_collisions:
            print("Failed: ", exp_collisions, collisions)
            savefile = "{}/failed{}_{}_{}_{}x{}.json".format(saveDir, trial, numObjs, RAD, HEIGHT, WIDTH)
            print(savefile)
            os.rename(tmpfile, savefile)
            break
