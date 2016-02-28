#!/usr/bin/env python

import csv
import argparse
import matplotlib.pyplot as plt


def parseCsv(csvFile):
    reader = csv.reader(csvFile, delimiter=',')
    reader.next()

    data = []
    for row in reader:
        data_row = map(lambda x: float(x), row)
        if data_row:
            data.append(data_row)

    return data


def translationErrorOfData(data):
    return map(lambda x: (x[0], x[1], x[2]), data)


def rotationErrorOfData(data):
    return map(lambda x: (x[0], x[3], x[4]), data)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Plot the variance of the icp')
    parser.add_argument('--input', '-i', type=argparse.FileType('r'))
    args = parser.parse_args()

    data = parseCsv(args.input)

    (traveledDistance, translationError,
     translationVar, rotationError, rotationVar, _) = zip(*data)

    f, axesarray = plt.subplots(2, sharex=True)

    transError = axesarray[0].errorbar(traveledDistance, translationError, translationVar)
    rotError = axesarray[1].errorbar(traveledDistance, rotationError, rotationVar)

    axesarray[0].set_ylabel('Translation error $e_{t,p}$ (m)', fontsize=16)
    axesarray[1].set_ylabel('Rotation error $e_{r,p}$ (rad)', fontsize=16)

    plt.xlabel('Travelled distance (m)', fontsize=16)

    plt.show()
