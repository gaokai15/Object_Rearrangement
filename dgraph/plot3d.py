from __future__ import print_function, division

import sys
import glob
import time
import json

import numpy as np
import pandas as pd
import plotly_express as px
import plotly.graph_objects as go
from plotly.offline import plot

### Read Files ###
cc1 = 0
cc2 = 0
tcc1 = 0
tcc2 = 0
bcc1 = 0
bcc2 = 0
cc3 = 0
tcc3 = 0
bcc3 = 0
progress = 0
dfdata = []
dfdata2 = []
dfdata3 = []
toignore = {2: set(), 3: set()}
for filename in sorted(glob.glob(sys.argv[1] + '/*/*/*/*.json')):
    D, n, trial = filename.split('/')[2:]
    D = float(D.split('=')[-1])
    n = int(n.split('=')[-1])
    trial = trial.split('.')[0]
    # print(D, n, trial)

    if progress % 1600 == 0 and progress > 0:
        print("ch1:", cc1, "/", bcc1, "ch2 ", cc2, "/", bcc2, "ch3 ", cc3, "/", bcc3, file=sys.stderr)
        cc1 = 0
        cc2 = 0
        cc3 = 0
        bcc1 = 0
        bcc2 = 0
        bcc3 = 0
    progress += 1
    with open(filename) as f:
        data = json.load(f)
    if 'ch1' in filename:
        bcc1 += 1
        if 'is_monotone' in data and data['is_monotone'] is not None:
            cc1 += 1
            tcc1 += 1
            dfdata.append([D, n, trial, data['is_monotone'], data['computation_time']])
            if data['is_monotone']:
                toignore[2].add((D, n, trial))
                toignore[3].add((D, n, trial))
    if 'ch2' in filename:
        bcc2 += 1
        if 'is_perturbable' in data and data['is_perturbable'] is not None:
            cc2 += 1
            tcc2 += 1
            if type(data['is_perturbable']) == bool:
                continue
            if (D, n, trial) in toignore[2]:
                continue
            if data['is_perturbable'] is not None:
                if data['is_perturbable'] != 'Timeout':
                    for obj, vals in data['is_perturbable'].items():
                        dfdata2.append([D, n, trial + obj, vals, data['computation_time'][obj]])
                        if not vals:
                            toignore[3].add((D, n, trial + obj))
                else:
                    dfdata2.append([D, n, trial, 'Timeout', data['computation_time']])
                    toignore[3].add((D, n, trial))
            # if data['is_perturbable'] != 'Timeout':
            #     if 'Error' not in data['is_perturbable'].values():
            #         vals = sum(data['is_perturbable'].values()) / int(n)
            #     else:
            #         vals = -1  # Error
            # else:
            #     vals = 2  # Timeout
            # if not (D, n, trial) in toignore[2]:
            #     dfdata2.append([D, n, trial, vals, data['computation_time']])
            # if vals == 0:
            #     toignore[3].add((D, n, trial))
    if 'ch3' in filename:
        bcc3 += 1
        if 'is_valid_buffer' in data and data['is_valid_buffer'] is not None:
            cc3 += 1
            tcc3 += 1
            if data['is_valid_buffer'] == 'Timeout':
                vals = 2  # Timeout
            elif data['is_valid_buffer'] == 'Bad instance':
                vals = -1  # Error
            else:
                if 'Timeout' in data['is_valid_buffer'].values():
                    vals = 2  # Timeout
                elif 'Error' in data['is_valid_buffer'].values():
                    vals = -1  # Error
                else:
                    vals = sum(data['is_valid_buffer'].values()) / len(data['is_valid_buffer'])
            obj = str(data['object_selected']) if 'object_selected' in data else ''
            if not (D, n, trial + obj) in toignore[3]:
                dfdata3.append([D, n, trial, vals, data['computation_time']])

print("Total: ch1:", tcc1, "ch2", tcc2, "ch3", tcc3, file=sys.stderr)

### Create Data Frames ###
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
columns2 = ["density", "number", "trial", "perturbable", "time"]
df2 = pd.DataFrame(data=dfdata2, columns=columns2)
print(
    '\nTimeouts: ',
    len(df2.query('perturbable=="Timeout"')),
    '\nErrors: ',
    len(df2.query('perturbable=="Error"')),
    '\nTotal: ',
    len(df2),
    file=sys.stderr,
)
print("Not non-2-tone: ", len(df2.query("perturbable==True")), file=sys.stderr)
columns3 = ["density", "number", "trial", "num_buff", "time"]
df3 = pd.DataFrame(data=dfdata3, columns=columns3)
print(
    '\nTimeouts: ',
    len(df3.query('num_buff==2')),
    '\nErrors: ',
    len(df3.query('num_buff==-1')),
    '\nTotal: ',
    len(df3),
    file=sys.stderr,
)
print("2-tone: ", len(df3.query("num_buff>0 and num_buff<2")), file=sys.stderr)

### Filter and Aggregate ###
df = df.query('monotone!="Error"')  # and monotone!="Timeout"')
df = df.groupby(['density', 'number', 'monotone']
                ).agg({
                    # 'trial': lambda x: np.log(len(x)+1),
                    'trial': lambda x: len(x),
                    'time': 'mean',
                }).reset_index()

df2 = df2.query('perturbable!="Error"')  # and perturbable<2")
df2 = df2.groupby(['density', 'number', 'perturbable']).agg({
    'trial': lambda x: np.log2(len(x) + 1),
    'time': 'mean',
}).reset_index()

df3 = df3.query("num_buff>=0")  # and num_buff<2")
df3 = df3.groupby(['density', 'number', 'num_buff']).agg({
    'trial': lambda x: np.log2(len(x) + 1),
    'time': 'mean',
}).reset_index()

### Create Plots ###
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
        False: 'red',
        'Timeout': 'grey',
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
    color='perturbable',
    size='trial',
    size_max=30,
    log_z=True,
    # color_continuous_scale=((0.0, 'red'), (0.51, 'blue'), (0.51, 'grey'), (1.0, 'grey')),
    # color_continuous_scale=((0.0, 'black'), (0.001, 'black'), (0.001, 'red'), (1.0, 'blue')),
    color_discrete_map={
        True: 'blue',
        False: 'red',
        'Timeout': 'grey',
    },
    hover_data=['density', 'number', 'time', 'perturbable'],
    labels={
        'density': '2*n*pi/(w*h)',
        'number': '# Obj',
        'time': 'Time (sec)',
        'perturbable': '% Perturbable'
    },
    title='Computation Time',
)
fig.update_layout(
    scene_aspectmode='manual',
    scene_aspectratio=dict(x=1, y=1, z=3),
)
plot(fig, filename='challenge2.html')

fig = px.scatter_3d(
    df3,
    x='density',
    y='number',
    z='time',
    color='num_buff',
    size='trial',
    size_max=30,
    log_z=True,
    color_continuous_scale=((0.0, 'red'), (0.51, 'blue'), (0.51, 'grey'), (1.0, 'grey')),
    # color_continuous_scale=((0.0, 'black'), (0.001, 'black'), (0.001, 'red'), (1.0, 'blue')),
    hover_data=['density', 'number', 'time', 'num_buff'],
    labels={
        'density': '2*n*pi/(w*h)',
        'number': '# Obj',
        'time': 'Time (sec)',
        'num_buff': '% Buffer Valid'
    },
    title='Computation Time',
)
fig.update_layout(
    scene_aspectmode='manual',
    scene_aspectratio=dict(x=1, y=1, z=3),
)
plot(fig, filename='challenge3.html')
