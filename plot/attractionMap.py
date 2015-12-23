#!/usr/bin/env python3

import csv
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

FILENAME = 'attractionBassin.csv'

attractionMap = []
with open(FILENAME, 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')
    for row in reader:
        attractionMapRow = []
        for value in row:
            attractionMapRow.append(float(value))
        attractionMap.append(attractionMapRow)

X = np.arange(0,30,1)
Y = np.arange(0,30,1)
X, Y = np.meshgrid(X,Y)

fig = plt.figure()
axes = fig.gca(projection='3d')
surf = axes.plot_surface(X,Y,attractionMap, rstride=1, cstride=1, cmap='hot',
                         linewidth=0.5, antialiased=False)
# fig.colorbar(surf, shrink=0.9, aspect=5)
plt.show()

