import numpy as np

mach_array = np.array([ 0.5  ,  0.6  ,  0.625,  0.65 ,  0.675,  0.7  ,  0.725])
cd_array = np.array([ 0.04241176,  0.03947743,  0.04061261,  0.04464372,  0.05726695,
    0.07248304,  0.08451007])

np.savetxt('../../../paper/images/data_files/cd_vs_mach/cd.txt', cd_array, fmt = '%f', delimiter = '\t', newline = '\r\n')
np.savetxt('../../../paper/images/data_files/cd_vs_mach/mach.txt', mach_array, fmt = '%f', delimiter = '\t', newline = '\r\n')