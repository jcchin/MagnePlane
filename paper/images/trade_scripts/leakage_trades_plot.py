import numpy as np
import matplotlib.pyplot as plt

m_dot = np.loadtxt('../data_files/leakage_trades/m_dot.txt', delimiter = '\t')
p_tunnel = np.loadtxt('../data_files/leakage_trades/p_tunnel.txt', delimiter = '\t')
total_energy_cost = np.loadtxt('../data_files/leakage_trades/total_energy_cost.txt', delimiter = '\t')

plt.plot(m_dot, p_tunnel, 'b-', linewidth = 2.0)
plt.xlabel('Leakage Rate (kg/s)', fontsize = 12, fontweight = 'bold')
plt.ylabel('Pressure at Minimum Total Energy (Pa)', fontsize = 12, fontweight = 'bold')
plt.savefig('../graphs/leakage_trades/leakage_vs_pressure.png', format = 'png', dpi = 300)
plt.show()
plt.plot(m_dot, total_energy_cost/(1.0e6), 'r-', linewidth = 2.0)
plt.xlabel('Leakage Rate (kg/s)', fontsize = 12, fontweight = 'bold')
plt.ylabel('Minimum Energy Cost (Million USD)', fontsize = 12, fontweight = 'bold')
plt.savefig('../graphs/leakage_trades/leakage_vs_energy.png', format = 'png', dpi = 300)
plt.show()