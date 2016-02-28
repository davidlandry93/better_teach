#!/usr/bin/env python3

import matplotlib.pyplot as plt

runs_with_017_rotation = [(0.2, 250), (0.6, 251), (1.2, 249), (1.4, 254), (1.6, 248),
                          (1.8, 255), (2.0, 264), (3.0, 274)]

runs_with_026_rotation = [(0.5, 285), (1.0, 291), (2.0, 294), (3.0, 295)]

runs_with_013_rotation = [(0.25, 203), (0.50, 205), (0.75, 202), (1.0, 213), (1.25, 207), (1.50, 210), (1.75, 211), (2.0, 216), (2.25, 218), (2.50, 219), (2.75, 223), (3.0, 239)]

if __name__ == '__main__':
    xs, ys = zip(*runs_with_013_rotation)
    ys = map(lambda x: (float(x) / 304) * 100, ys)
    plt.plot(xs, ys, 'k*')

    xs, ys = zip(*runs_with_017_rotation)
    ys = map(lambda x: (float(x) / 304) * 100, ys)
    plt.plot(xs, ys, 'bo')

    xs, ys = zip(*runs_with_026_rotation)
    ys = map(lambda x: (float(x) / 304) * 100, ys)
    plt.plot(xs, ys, 'rs')

    plt.xlabel('Values of parameters $a$, $b$ (m)', fontsize=16)
    plt.ylabel('% of nodes kept', fontsize=16)
    plt.legend(['c = 0.13 rad', 'c = 0.17 rad', 'c = 0.26 rad'], loc=4)
    plt.axis([0.0, 4.0, 0, 100])

    plt.show()
