# -*- coding: utf-8 -*-

import argparse
import csv
import numpy as np
import matplotlib.pyplot as plt

CORRECTED_FILE = 'correctedSetOfAnchorPoints.csv'
OPTIMIZED_FILE = 'optimizedSetOfAnchorPoints.csv'

def bring_poses_to_origin(poses):
    firstPose = (float(poses[0][0]), float(poses[0][1]))

    return map(lambda x: (float(x[0]) - firstPose[0], float(x[1]) - firstPose[1]), poses)

def poses_of_csv(csvFile):
    reader = csv.reader(csvFile, delimiter=',')

    poses = []
    for row in reader:
        poses.append((row[1], row[2]))
    return poses

if __name__ == '__main__':

    correctedFile = open(CORRECTED_FILE)
    correctedPoses = poses_of_csv(correctedFile)
    correctedFile.close()

    optimizedFile = open(OPTIMIZED_FILE)
    optimizedPoses = poses_of_csv(optimizedFile)
    optimizedFile.close()


    xs, ys = zip(*optimizedPoses)
    plt.scatter(xs, ys, s=50.0, color='red')
    plt.axis('equal')

    xs, ys = zip(*correctedPoses)
    plt.scatter(xs,ys,marker='*')
    plt.axis('equal')

    plt.legend(['Anchor points of the optimized map', 'Anchor points of the original map'])
    plt.xlabel('Position of the anchor point (m)')
    plt.ylabel('Position of the anchor point (m)')
 
    plt.show()
