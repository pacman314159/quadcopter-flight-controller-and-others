import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

X = np.random.rand(100, 3)*10
Y = np.random.rand(100, 3)*5

plt.ion()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
sc = ax.scatter(X[:, 0], X[:, 1], X[:, 2])
fig.show()

for i in range(0, 20):
    plt.pause(1)

    Y = np.random.rand(100, 3)*5

    sc._offsets3d = (Y[:,0], Y[:,1], Y[:,2])
    plt.draw()