#!/usr/bin/env python3
from __future__ import print_function, division

import sys
import glob
import json
import numpy as np
import pandas as pd
import plotly_express as px
import plotly.graph_objects as go
from plotly.offline import plot

cc1 = 0
cc2 = 0
tcc1 = 0
tcc2 = 0
bcc1 = 0
bcc2 = 0
progress = 0
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

print("Total: ch1:", tcc1, "ch2", tcc2, file=sys.stderr)

columns = ["density", "number", "trial", "monotone", "time"]
df = pd.DataFrame(data=dfdata, columns=columns)
print(
    '\nTimeouts: ',
    len(df.query('monotone=="Timeout"')),
    '\nErrors: ',
    len(df.query('monotone=="Error"')),
    '\nTotal: ',
    len(df),
    file=sys.stderr,
)
df = df.query('monotone!="Error"')  # and monotone!="Timeout"')
df = df.groupby(['density', 'number', 'monotone']
                ).agg({
                    # 'trial': lambda x: np.log(len(x)+1),
                    'trial': lambda x: len(x),
                    'time': 'mean',
                }).reset_index()

columns2 = ["density", "number", "trial", "num_pert", "time"]
df2 = pd.DataFrame(data=dfdata2, columns=columns2)
print(
    '\nTimeouts: ',
    len(df2.query('num_pert==2')),
    '\nErrors: ',
    len(df2.query('num_pert==-1')),
    '\nTotal: ',
    len(df2),
    file=sys.stderr,
)
df2 = df2.query("num_pert>=0")  # and num_pert<2")
print("Not non-2tone: ", len(df2))
df2 = df2.groupby(['density', 'number', 'num_pert']).agg({
    'trial': lambda x: np.log2(len(x) + 1),
    'time': 'mean',
}).reset_index()

fig = px.scatter_3d(
    df,
    x='density',
    y='number',
    z='time',
    color='monotone',
    size='trial',
    size_max=30,
    # log_x=True,
    # log_y=True,
    log_z=True,
    color_discrete_map={
        True: 'blue',
        False: 'red'
    },
    hover_data=['density', 'number', 'time', 'monotone'],
    labels={
        'density': '2*n*pi/(w*h)',
        'number': '# Obj',
        'time': 'Time (sec)',
        'monotone': 'Is Monotone?'
    },
    title='Computation Time',
)
fig.update_layout(
    scene_aspectmode='manual',
    scene_aspectratio=dict(x=1, y=1, z=3),
)
plot(fig, filename='challenge1.html')

fig = px.scatter_3d(
    df2,
    x='density',
    y='number',
    z='time',
    color='num_pert',
    size='trial',
    size_max=30,
    # log_x=True,
    # log_y=True,
    log_z=True,
    color_continuous_scale=((0.0, 'red'), (0.51, 'blue'), (0.51, 'green'), (1.0, 'green')),
    # color_continuous_scale=((0.0, 'black'), (0.001, 'black'), (0.001, 'red'), (1.0, 'blue')),
    hover_data=['density', 'number', 'time', 'num_pert'],
    labels={
        'density': '2*n*pi/(w*h)',
        'number': '# Obj',
        'time': 'Time (sec)',
        'num_pert': '% Perturbable'
    },
    title='Computation Time',
)
fig.update_layout(
    scene_aspectmode='manual',
    scene_aspectratio=dict(x=1, y=1, z=3),
)
plot(fig, filename='challenge2.html')
