import numpy as np
import matplotlib.pyplot as plt

data = np.genfromtxt('posq2.csv', delimiter=',')

data = data[::1]

plt.scatter(data[:, 0], data[:, 1])
plt.scatter(data[0, 0], data[0, 1], color='red')
plt.scatter(0.8, -0.9, color='green')

plt.gca().set_aspect('equal', adjustable='box')

plt.show()
