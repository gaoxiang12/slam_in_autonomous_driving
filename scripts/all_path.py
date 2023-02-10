# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def PlotPath(keyframes):
    fig = plt.figure('All path 3d')
    ax = Axes3D(fig)

    # lidar pose
    p00, = ax.plot(keyframes[:, 5], keyframes[:, 6], keyframes[:, 7], 'y-')

    # gps pose
    p01, = ax.plot(keyframes[:, 12], keyframes[:, 13], keyframes[:, 14], 'g-')

    # opti1
    p02, = ax.plot(keyframes[:, 19], keyframes[:, 20], keyframes[:, 21], 'b-')

    # opti2
    # p03, = ax.plot(keyframes[:, 26], keyframes[:, 27], keyframes[:, 28], 'r-')

    plt.title("All path", fontsize=16)
    plt.grid()
    fig = plt.figure('All path 2d')

    # lidar
    p0, = plt.plot(keyframes[:, 5], keyframes[:, 6], 'y-')

    # gps
    p1, = plt.plot(keyframes[:, 12], keyframes[:, 13], 'g-')

    # opti1
    p2, = plt.plot(keyframes[:, 19], keyframes[:, 20], 'b-')

    # opti2
    # p3, = plt.plot(keyframes[:, 26], keyframes[:, 27], 'r-')
    # plt.legend(['Lidar', 'RTK', 'Opti1', 'Opti2'])
    plt.legend(['Lidar', 'RTK', 'Opti1'])

    plt.show()


def LoadMappingTxt(filepath):
    keyframes = np.loadtxt(filepath)
    PlotPath(keyframes)


if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Please input vaild param !!!')
        exit(1)
    else:
        path = sys.argv[1]
        LoadMappingTxt(path)
        exit(1)
        exit(1)
