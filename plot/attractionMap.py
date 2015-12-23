#!/usr/bin/env python3

import csv
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm


def xyzOfCsv(csvFilename):
    Z = []
    with open(csvFilename, 'r') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            ZRow = []
            for value in row:
                ZRow.append(float(value))
            Z.append(ZRow)

    X = np.arange(-3,3, 6.0/20)
    print(len(X))
    Y = np.arange(-3,3, 6.0/20)
    X, Y = np.meshgrid(X,Y)
    return X,Y,Z

def altMain():
    FILENAME = 'attractionBassinForet-10vers11.csv'

    filenames = ['attractionBassinForet-10vers11.csv',
                 'attractionBassinForet-10vers13.csv',
                 'attractionBassinForet-10vers15.csv']

    figure = plt.figure(figsize=plt.figaspect(2.))
    for i, filename in enumerate(filenames):
        X,Y,Z = xyzOfCsv(filename)

        ax = figure.add_subplot(len(filenames),1,i + 1, projection='3d')
        ax.plot_surface(X,Y,Z, rstride=1, cstride=1, cmap='hot', linewidth=0.3)

    plt.show()

if __name__ == '__main__':
    filename = 'attractionBassinForet-10vers11.csv'
    figure = plt.figure()
    ax = figure.gca(projection='3d')

    X,Y,Z = xyzOfCsv(filename)

    ax.plot_surface(X,Y,Z, rstride=1, cstride=1, cmap='hot', linewidth=0.3)
    ax.set_xlabel('Induced translation (X)')
    ax.set_ylabel('Induced translation (Y)')
    ax.set_zlabel('Convergence error')
    plt.show()
