import numpy as np
import matplotlib.pyplot as plt

n_passengers = np.loadtxt('../data_files/capacity_trades/n_passengers.txt', delimiter = '\t')
A_tube = np.loadtxt('../data_files/capacity_trades/A_tube.txt', delimiter = '\t')
ticket_cost = np.loadtxt('../data_files/capacity_trades/ticket_cost.txt', delimiter = '\t')
total_energy_cost = np.loadtxt('../data_files/capacity_trades/total_energy_cost.txt', delimiter = '\t')

plt.plot(n_passengers, A_tube, linewidth = 2.0)
plt.xlabel('Passengers per Pod', fontsize = 12, fontweight = 'bold')
plt.ylabel('Tube Area (m^2)', fontsize = 12, fontweight = 'bold')
plt.savefig('../graphs/capacity_trades/passengers_vs_area.png', format = 'png', dpi = 300)
plt.show()
plt.plot(n_passengers, total_energy_cost/(1e6), linewidth = 2.0)
plt.xlabel('Passengers per Pod', fontsize = 12, fontweight = 'bold')
plt.ylabel('Yearly Energy Cost (USD)', fontsize = 12, fontweight = 'bold')
plt.ylim(25.0, 35.0)
plt.savefig('../graphs/capacity_trades/passengers_vs_energy.png', format = 'png', dpi = 300)
plt.show()
plt.plot(n_passengers, ticket_cost, linewidth = 2.0)
plt.xlabel('Passengers per Pod', fontsize = 12, fontweight = 'bold')
plt.ylabel('Estimated Ticket Cost (USD)', fontsize = 12, fontweight = 'bold')
plt.savefig('../graphs/capacity_trades/passengers_vs_ticket_cost.png', format = 'png', dpi = 300)
plt.show()