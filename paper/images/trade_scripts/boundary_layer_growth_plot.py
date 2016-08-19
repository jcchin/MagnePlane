import numpy as np
import matplotlib.pyplot as plt

delta_star = np.loadtxt('../data_files/boundary_layer_growth_trades/delta_star.txt', delimiter = '\t')
A_tube = np.loadtxt('../data_files/boundary_layer_growth_trades/A_tube.txt', delimiter = '\t')

plt.hold(True)
line1, = plt.plot(delta_star, A_tube[0,:], 'b-', linewidth = 2.0, label = 'A_pod = 2.0 m^2')
line2, = plt.plot(delta_star, A_tube[1,:], 'r-', linewidth = 2.0, label = 'A_pod = 2.5 m^2')
line3, = plt.plot(delta_star, A_tube[2,:], 'g-', linewidth = 2.0, label = 'A_pod = 3.0 m^2')
plt.xlabel('Displacement Boundary Layer (m)', fontsize = 16, fontweight = 'bold')
plt.ylabel('Tube Cross Sectional Area (m**2)', fontsize = 16, fontweight = 'bold')
plt.grid('on')
plt.xlim(.02, .12)
plt.legend(handles = [line1, line2, line3], loc = 2)
plt.savefig('../graphs/boundary_layer_growth_trades/Tube_Area_vs_boundary_layer.png', format = 'png', dpi = 300)
plt.show()