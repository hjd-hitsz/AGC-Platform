import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':
    points = np.loadtxt('/home/hjd-nrsl/code/AGC_Platform_ws/src/AGC-Platform/control/uav_geometry_control/data/point.txt')
    plt.figure()
    ax = plt.axes(projection='3d')
    ax.scatter3D(points[:, 0], points[:, 1], points[:, 2])
    plt.show()