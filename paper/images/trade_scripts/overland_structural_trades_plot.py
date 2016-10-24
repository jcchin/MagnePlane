import numpy as np
import matplotlib.pyplot as plt

m_pod = np.loadtxt('../data_files/overland_structural_trades/m_pod.txt', delimiter = '\t')
A_tube = np.loadtxt('../data_files/overland_structural_trades/A_tube.txt', delimiter = '\t')
dx = np.loadtxt('../data_files/overland_structural_trades/dx.txt', delimiter = '\t')
cost = np.loadtxt('../data_files/overland_structural_trades/cost.txt', delimiter = '\t')

fig = plt.figure(figsize = (3.25,3.5), tight_layout = True)
ax = plt.axes()
plt.setp(ax.get_xticklabels(), fontsize=8)
plt.setp(ax.get_yticklabels(), fontsize=8)
plt.hold(True)
line1, = plt.plot(A_tube, dx[0,:], 'b-', linewidth = 2.0, label = 'pod mass = 10000 kg')
line2, = plt.plot(A_tube, dx[1,:], 'r-', linewidth = 2.0, label = 'pod mass = 15000 kg')
line3, = plt.plot(A_tube, dx[2,:], 'g-', linewidth = 2.0, label = 'pod mass = 20000 kg')
plt.xlabel('Tube Area (m^2)', fontsize = 10, fontweight = 'bold')
plt.ylabel('Pylon Spacing (m)', fontsize = 10, fontweight = 'bold')
plt.grid('on')
plt.legend(handles = [line1, line2, line3], loc = 1, fontsize = 8)
plt.savefig('../graphs/overland_structural_trades/pylon_spacing_vs_tube_area.png', format = 'png', dpi = 300)
plt.show()