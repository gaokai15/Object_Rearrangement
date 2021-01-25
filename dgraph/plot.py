import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

datas = {}
for filename in sys.argv[1:]:
    key = filename.split('.')[0]
    data = pd.read_csv(filename, skipinitialspace=True)
    dtypes = list(data.drop(['name', 'num_objs', 'radius', 'trial'], axis=1).keys()) + ['success_rate']
    datas[key] = (data[data['actions'] > -1].groupby(['name', 'num_objs', 'radius']).mean().drop('trial', axis=1))
    datas[key]['success_rate'] = data.groupby(['name', 'num_objs', 'radius'])['actions'].agg(
        lambda x: float(len(x) - x.value_counts()[-1]) / len(x) if x.value_counts().index.min() == -1 else 1
    )

toplot = {col: pd.DataFrame() for col in dtypes}
for col in dtypes:
    for method, dat in datas.items():
        toplot[col][method] = dat[col]

fig, arrx = plt.subplots(len(toplot.values()[0].index.levels[0]), len(toplot))
x = 0
for env in toplot.values()[0].index.levels[0]:
    y = 0
    arrx[y][x].set_title(env)
    for title, alldat in toplot.items():
        arrx[y][0].set_ylabel(title)
        dat = alldat.query('name==@env').droplevel(0)
        if title in ('time', 'iterations'):
            dat.plot.bar(ax=arrx[y][x], rot=20, logy=True)
        else:
            dat.plot.bar(ax=arrx[y][x], rot=20)
        y += 1
    x += 1

plt.show()

##########
"""
dim = 3
w = 0.75
dimw = w / dim

plt.xlabel('# of Objects')
plt.ylabel('Time')
plt.title('Alg')

data1 = [23, 85, 72, 43, 52]
data2 = [42, 35, 21, 16, 9]
data3 = [42, 35, 21, 16, 9]
x = np.arange(len(data1))
plt.bar(x + 0 * dimw, data1, dimw)
plt.bar(x + 1 * dimw, data2, dimw)
plt.bar(x + 2 * dimw, data3, dimw)
print(x + dimw / 2.0)
plt.xticks(x + (w - dimw) / 2, x)
plt.show()
"""
