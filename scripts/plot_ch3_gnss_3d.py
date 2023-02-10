# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# 格式：t x y z qx qy qz qw
if __name__ == '__main__':
    if len(sys.argv) != 2:
        print('Please input valid file')
        exit(1)
    else:
        path = sys.argv[1]
        path_data = np.loadtxt(path)
        fig = plt.figure()
        ax = Axes3D(fig)
        ax.plot(path_data[:, 1], path_data[:, 2], path_data[:, 3])
        plt.title('3D path')
        plt.show()
        exit(1)
