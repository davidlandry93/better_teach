#!/usr/bin/env python3

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

    fig1 = plt.figure()
    translationBars = plt.errorbar(traveledDistance, translationError, translationVar)
    plt.xlabel('Travelled distance (m)')
    plt.ylabel('Localization error (m)')

    fig2 = plt.figure()
    plt.xlabel('Travelled distance (m)')
    plt.ylabel('Localization error (rad)')
    rotationBars = plt.errorbar(traveledDistance, rotationError, rotationVar)

    plt.show()
