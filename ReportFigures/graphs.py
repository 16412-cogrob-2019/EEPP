import numpy as np
import matplotlib.pyplot as plt

# data to plot
n_groups = 4
times_Astar = (1.51, .64, 1.14, 1.15)
times_PCAstar = (1.26, .55, .64, .19)

costs_Astar = (17.14, 11.60, 16.82, 15.35)
costs_PCAstar = (17.48, 12.13, 17.15, 15.68)

# create plot
fig, ax = plt.subplots()
index = np.arange(n_groups)
bar_width = 0.35
opacity = 0.8

# rects1 = plt.bar(index, times_Astar, bar_width,
rects1 = plt.bar(index, costs_Astar, bar_width,
alpha=opacity,
label='A*')

# rects2 = plt.bar(index + bar_width, times_PCAstar, bar_width,
rects2 = plt.bar(index + bar_width, costs_PCAstar, bar_width,
alpha=opacity,
label='PCA*')

plt.xlabel('Goal Location Color (in order of path found)')
# plt.ylabel('Computation Time')
plt.ylabel('Path Cost')
plt.title('Traversal Cost for Each Path to Goal')
plt.xticks(index + bar_width, ('Red', 'Blue', 'White', 'Cyan'))
plt.legend()

plt.tight_layout()
plt.show()