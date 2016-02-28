#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

N = 3
unoptimizedMeans = (0.21, 0.27, 0.05)
unpotimizedStd = (0.26, 0.13, 0.01)

ind = np.arange(N)  # the x locations for the groups
width = 0.35       # the width of the bars

fig, ax = plt.subplots()
rects1 = ax.bar(ind, unoptimizedMeans, width, color='w', yerr=unpotimizedStd, error_kw=dict(ecolor='black'))

optimizedMeans = (0.30, 0.34, 0.10)
optimizedStd = (0.30, 0.13, 0.09)
rects2 = ax.bar(ind + width, optimizedMeans, width, color='gray', yerr=optimizedStd, error_kw=dict(ecolor='black'))

# add some text for labels, title and axes ticks
ax.set_ylabel('Average distance between nodes (m)', fontsize=16)
ax.set_xticks(ind + width)
ax.set_xticklabels(('terasse', 'forest', 'hallway'))

for tick in ax.xaxis.get_major_ticks():
    tick.label.set_fontsize(14)

ax.legend((rects1[0], rects2[0]), ('Unoptimized', 'Optimized'))

plt.show()
