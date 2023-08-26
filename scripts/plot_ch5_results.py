# coding=UTF-8
import sys
import numpy as np
import matplotlib.pyplot as plt

method_names = ['Brute-force', 'Brute-force MT', '2D Grid8', '2D Grid8 MT', '3D Grid', '3D Grid MT', 'KD Tree',
                'KD Tree MT',
                'KD Tree PCL', 'Octo Tree', 'Octo Tree MT']
time_usage = [821.109, 102.074, 7.6908, 0.7044, 3.1735, 0.3916, 9.6234, 2.0651, 11.8725, 31.9253, 4.7698]

fig, ax = plt.subplots()
plt.bar(method_names, time_usage, color='rgbky')
plt.title('NN methods')
ax.set_yscale('log')
# plt.xlabel("method name")
plt.ylabel("runtime(s)")
plt.grid()
plt.show()
