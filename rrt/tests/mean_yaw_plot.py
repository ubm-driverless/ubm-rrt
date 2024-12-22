import numpy as np
import matplotlib.pyplot as plt

# path = [[-2.0, 0.5], [1.0, 1.0], [3.0, 0.2], [3.2, 5.0], [3.6, 5.0], [4.0, 2.0], [4.4, 5.0], [5.0, -8.0]]

path = [[0.0, 0.0], [1.0, 1.0], [3.0, 0.2], [3.2, 5.0], [3.6, 5.0], [4.0, 2.0], [4.4, 5.0], [5.0, 5.0]]

# read csv file called `mean_yaw.csv` and plot the data. Each row is a tuple (x, y, yaw). For each point plot an arrow
# with the corresponding yaw angle.
data = np.genfromtxt('mean_yaw.csv', delimiter=',', skip_header=0, names=['x', 'y', 'yaw'])

plt.figure(dpi=500, figsize=(10, 10))
plt.plot([x[0] for x in path], [x[1] for x in path], 'ro-')
for i in range(0, len(data), 1):
    plt.arrow(data['x'][i], data['y'][i], 0.01 * np.cos(data['yaw'][i]), 0.01 * np.sin(data['yaw'][i]), head_width=0.01)

plt.tight_layout()
plt.show()
