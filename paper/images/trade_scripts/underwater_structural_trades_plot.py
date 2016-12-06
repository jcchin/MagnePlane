import numpy as np
import matplotlib.pyplot as plt

depth = np.loadtxt('../data_files/underwater_structural_trades/depth.txt', delimiter = '\t')
A_tube = np.loadtxt('../data_files/underwater_structural_trades/A_tube.txt', delimiter = '\t')
t = np.loadtxt('../data_files/underwater_structural_trades/t.txt', delimiter = '\t')
cost = np.loadtxt('../data_files/underwater_structural_trades/cost.txt', delimiter = '\t')

fig = plt.figure(figsize = (3.25,3.5), tight_layout = True)
ax = plt.axes()
plt.setp(ax.get_xticklabels(), fontsize=8)
plt.setp(ax.get_yticklabels(), fontsize=8)
line1, = plt.plot(A_tube, t[0,:], 'b-', linewidth = 2.0, label = 'depth = 20 m')
line2, = plt.plot(A_tube, t[1,:], 'r-', linewidth = 2.0, label = 'depth = 40 m')
line3, = plt.plot(A_tube, t[2,:], 'g-', linewidth = 2.0, label = 'depth = 60 m')
plt.xlabel('Tube Area ($m^2$)', fontsize = 10, fontweight = 'bold')
plt.ylabel('Tube Thickness (m)', fontsize = 10, fontweight = 'bold')
plt.grid('on')
plt.legend(handles = [line1, line2, line3], loc = 2, fontsize = 8)
plt.savefig('../graphs/underwater_structural_trades/tube_area_vs_depth.png', format = 'png', dpi = 300)
plt.show()