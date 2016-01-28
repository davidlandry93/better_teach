#!/usr/bin/env python3

import csv
import matplotlib.pyplot as plt
import numpy as np


def parseCsv(filename):
    with open(filename) as csvFile:
        reader = csv.reader(csvFile, delimiter=',')
        reader.next()

        data = []
        for row in reader:
            data_row = map(lambda x: float(x), row)
            data.append(data_row)

        return data

if __name__ == '__main__':
    filename = 'mean_and_var.csv'
    data = parseCsv(filename)

    bars = plt.boxplot(data)

    plt.show()
