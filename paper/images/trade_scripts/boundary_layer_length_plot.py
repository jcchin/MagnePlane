import numpy as np
import matplotlib.pyplot as plt

L_pod = np.loadtxt('../data_files/boundary_layer_length_trades/L_pod.txt', delimiter = '\t')
A_tube = np.loadtxt('../data_files/boundary_layer_length_trades/A_tube.txt', delimiter = '\t')

plt.plot(L_pod, A_tube, linewidth = 2.0)
plt.xlabel('Pod Length (m)', fontsize = 16, fontweight = 'bold')
plt.ylabel('Tube Area (m^2)', fontsize = 16, fontweight = 'bold')
plt.savefig('../graphs/boundary_layer_length_trades/Tube_Area_vs_pod_length.png', format = 'png', dpi = 300)
plt.show()
