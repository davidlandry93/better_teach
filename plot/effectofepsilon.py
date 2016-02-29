#!/usr/bin/env python3

import matplotlib.pyplot as plt

runs_with_009_epsilon = [(0.05, 304), (0.1, 291), (0.15, 245), (0.2, 206), (0.25, 180), (0.3, 159)]

runs_with_017_epsilon = [(0.05, 304), (0.1, 291), (0.15, 245), (0.2, 202)]

if __name__ == '__main__':
    xs, ys = zip(*runs_with_009_epsilon)
    ys = map(lambda x: (float(x) / 304) * 100, ys)
    plt.plot(xs, ys, 'yo')

    xs, ys = zip(*runs_with_017_epsilon)
    ys = map(lambda x: (float(x) / 304) * 100, ys)
    plt.plot(xs, ys, 'k*')

    plt.xlabel('Value of $\epsilon_{pos}$ (m)', fontsize=16)
    plt.ylabel('% of nodes kept', fontsize=16)
    plt.legend(['$\epsilon_{angle}$ = 0.08 rad', '$\epsilon_{angle}$ = 0.17 rad'], loc=4)
    plt.axis([0.0, 0.4, 0, 100])

    plt.show()
