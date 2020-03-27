import os
from c2d2r.DG_Space import Experiments

numObjs = 8
RAD = 80
HEIGHT = 400
WIDTH = 2600
display = False
displayMore = False
saveDir = "workspaces"

sucfreq = {}
for trial in range(1000):
    tmpfile = saveDir + "/TEMP"
    EXP = Experiments()
    success = -1
    while success < 0:
        success = EXP.single_instance(
            numObjs, RAD, HEIGHT, WIDTH, display, displayMore, tmpfile
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
