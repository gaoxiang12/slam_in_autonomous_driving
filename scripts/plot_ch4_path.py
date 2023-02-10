# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Please input valid file')
        exit(1)
    else:
        path = sys.argv[1]
        path_data = np.loadtxt(path)
        plt.rcParams['figure.figsize'] = (16.0, 12.0)

        # 轨迹
        plt.subplot(121)
        plt.scatter(path_data[:, 1], path_data[:, 2], s=2)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.grid()
        plt.title('2D trajectory')

        # 速度
        plt.subplot(322)
        plt.plot(path_data[:, 0], path_data[:, 8], 'r')
        plt.plot(path_data[:, 0], path_data[:, 9], 'g')
        plt.plot(path_data[:, 0], path_data[:, 10], 'b')
        plt.title('v')
        plt.legend(['vx', 'vy', 'vz'])

        # 零偏
        plt.subplot(324)
        plt.plot(path_data[:, 0], path_data[:, 11], 'r')
        plt.plot(path_data[:, 0], path_data[:, 12], 'g')
        plt.plot(path_data[:, 0], path_data[:, 13], 'b')
        plt.title('bias gyro')
        plt.legend(['bg_x', 'bg_y', 'bg_z'])

        plt.subplot(326)
        plt.plot(path_data[:, 0], path_data[:, 14], 'r')
        plt.plot(path_data[:, 0], path_data[:, 15], 'g')
        plt.plot(path_data[:, 0], path_data[:, 16], 'b')
        plt.title('bias acce')
        plt.legend(['ba_x', 'ba_y', 'ba_z'])

        # 3D轨迹
        fig = plt.figure()
        ax = Axes3D(fig)
        ax.scatter(path_data[:, 1], path_data[:, 2], path_data[:, 3], s=2)

        plt.show()
        exit(1)
