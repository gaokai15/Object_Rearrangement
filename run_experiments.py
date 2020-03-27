#!/usr/bin/env python2
import os
import matplotlib.pyplot as plt
from c2d2r.DG_Space import Experiments

numObjs = 11
RAD = 80
HEIGHT = 1200
WIDTH = 1200
display = False
displayMore = False
saveDir = "workspaces"
numTrials = 1000

sucfreq = {}
for trial in range(numTrials):
    tmpfile = saveDir + "/TEMP"
    EXP = Experiments()
    success = -1
    while success < 0:
        success = int(
            EXP.single_instance(numObjs, RAD, HEIGHT, WIDTH, display, displayMore, tmpfile)
        )

    sucfreq[success] = sucfreq.get(success, 0) + 1
    print(trial)
    if trial % 100 == 0:
        print(sucfreq)
    if success > 0:
        savefile = "{}/exp{}-{}_{}_{}_{}x{}.json".format(
            saveDir, trial, success, numObjs, RAD, HEIGHT, WIDTH
        )
        print(savefile)
        os.rename(tmpfile, savefile)

print(sucfreq)
plt.bar(sucfreq.keys(), [x / 10.0 for x in sucfreq.values()])
for k, v in sucfreq.items():
    plt.text(k - 1.0 / 8, v + 1, str(v / 10.0) + "%")
plt.xlabel("|FVS|")
plt.ylabel("% Trials")
plt.title("Trials: {}, Obj:{}, r={}, size:{}x{}".format(numTrials, numObjs, RAD, HEIGHT, WIDTH))
plt.show()
