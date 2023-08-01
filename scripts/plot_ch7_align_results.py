# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt

method_names = ['Point-Point ICP', 'Point-Plane ICP', 'Point-Line ICP', 'NDT', 'PCL ICP', 'PCL NDT']
time_usage = [399.808, 334.409, 492.718, 142.278, 2774.16, 255.947]

res_icp_point = [0.211044, 0.155513, 0.114218, 0.0837873, 0.0611407, 0.0444493, 0.0320967, 0.0229813]
res_icp_plane = [0.0946098, 0.00664127, 0.000288226]
res_icp_line = [0.20137, 0.138774, 0.0931768, 0.0618093, 0.0410299, 0.0276539, 0.0192892]
res_ndt = [0.148351, 0.0597357, 0.0227735, 0.0100937, 0.00690136]
res_icp_pcl = [0.0483]
res_ndt_pcl = [0.1603]

# 用时
plt.figure()
plt.bar(method_names, time_usage, color='rgbkyc')
plt.title('Align Time Usage')
plt.xlabel("method name")
plt.ylabel("runtime(ms)")
plt.grid()
plt.show()

# 收敛曲线
plt.figure()
plt.plot(range(0, len(res_icp_point)), res_icp_point, color='r', marker='o')
plt.plot(range(0, len(res_icp_plane)), res_icp_plane, color='g', marker='o')
plt.plot(range(0, len(res_icp_line)), res_icp_line, color='b', marker='o')
plt.plot(range(0, len(res_ndt)), res_ndt, color='k', marker='o')
plt.axhline(res_icp_pcl, color='y')
plt.axhline(res_ndt_pcl, color='c')
# plt.yscale('log')
plt.title('Pose Convergence Speed')
plt.xlabel("iteration")
plt.ylabel("Pose error")
plt.legend(method_names)
plt.grid()
plt.show()
