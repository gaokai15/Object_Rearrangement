from __future__ import print_function, division

import sys
import glob
import time
import json
from pprint import pprint

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

cc1 = 0
cc2 = 0
tcc1 = 0
tcc2 = 0
bcc1 = 0
bcc2 = 0
progress = 0
toprint = []
dfdata = []
dfdata2 = []
for filename in sorted(glob.glob(sys.argv[1] + '/*/*/*/*.json')):
    D, n, trial = filename.split('/')[2:]
    D = float(D.split('=')[-1])
    n = int(n.split('=')[-1])
    trial = int(trial.split('.')[0])
    # print(D, n, trial)

    if progress % 1600 == 0 and progress > 0:
        print("ch1:", cc1, "/", bcc1, "ch2 ", cc2, "/", bcc2, file=sys.stderr)
        cc1 = 0
        cc2 = 0
        bcc1 = 0
        bcc2 = 0
    progress += 1
    with open(filename) as f:
        data = json.load(f)
    if 'ch1' in filename:
        bcc1 += 1
        if 'computation_time' in data:
            cc1 += 1
            tcc1 += 1
            dfdata.append([D, n, trial, data['is_monotone'], data['computation_time']])
            if data['is_monotone']:
                toprint.append(filename)
        # else:
        #     toprint.append(filename)
    if 'ch2' in filename:
        bcc2 += 1
        if 'computation_time' in data:
            cc2 += 1
            tcc2 += 1
            if data['is_perturbable'] != 'Timeout':
                if 'Error' not in data['is_perturbable'].values():
                    vals = sum(data['is_perturbable'].values()) / int(n)
                else:
                    vals = -1  # Error
            else:
                vals = 2  # Timeout
            dfdata2.append([D, n, trial, vals, data['computation_time']])

pprint(toprint)
print("Total: ch1:", tcc1, "ch2", tcc2, file=sys.stderr)

columns = ["density", "number", "trial", "monotone", "time"]
df = pd.DataFrame(data=dfdata, columns=columns)
columns2 = ["density", "number", "trial", "num_pert", "time"]
df2 = pd.DataFrame(data=dfdata2, columns=columns2)