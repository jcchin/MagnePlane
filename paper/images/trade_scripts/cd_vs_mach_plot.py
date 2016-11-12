import numpy as np 
import matplotlib.pyplot as plt
from scipy import interpolate as interp

cd = np.loadtxt('../data_files/cd_vs_mach/cd.txt', delimiter = '\t')
mach = np.loadtxt('../data_files/cd_vs_mach/mach.txt', delimiter = '\t')

f = interp.UnivariateSpline(mach, cd)

x = np.linspace(mach[0], mach[-1])

fig = plt.figure(figsize = (3.25,3.5), tight_layout = True)
plt.hold(True)
ax = plt.axes()
ax.plot(mach, cd, 'bo', linewidth = 2.0)
ax.plot(x, f(x), 'b--', linewidth = 1.0)
plt.setp(ax.get_xticklabels(), fontsize=8)
plt.setp(ax.get_yticklabels(), fontsize=8)
ax.set_xlabel('Mach Number', fontsize = 10, fontweight = 'bold')
ax.set_ylabel('Drag Coefficient', fontsize = 10, fontweight = 'bold')
plt.savefig('../graphs/cd_vs_mach/cd_vs_mach.png', format = 'png', dpi = 300)
plt.show()