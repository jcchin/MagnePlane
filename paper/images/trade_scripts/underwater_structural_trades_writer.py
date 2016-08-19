import numpy as np
import matplotlib.pylab as plt
from openmdao.api import Group, Problem, IndepVarComp

from hyperloop.Python.tube import submerged_tube

if __name__ == '__main__':
	top = Problem()
	root = top.root = Group()

	root.add('p', submerged_tube.SubmergedTube())

	top.setup()
	top['p.p_tube'] = 850.0

	depth = np.linspace(20.0, 60.0, num = 3)
	A_tube = np.linspace(20.0, 50.0, num = 30)

	cost = np.zeros((len(depth), len(A_tube)))
	t = np.zeros((len(depth), len(A_tube)))

	for i in range(len(A_tube)):
		for j in range(len(depth)):
			top['p.A_tube'] = A_tube[i]
			top['p.depth'] = depth[j]

			top.run()

			t[j,i] = top['p.t']
			cost[j,i] = top['p.material_cost']

	np.savetxt('../../../paper/images/data_files/underwater_structural_trades/depth.txt', depth, fmt = '%f', delimiter = '\t', newline = '\r\n')
	np.savetxt('../../../paper/images/data_files/underwater_structural_trades/A_tube.txt', A_tube, fmt = '%f', delimiter = '\t', newline = '\r\n')
	np.savetxt('../../../paper/images/data_files/underwater_structural_trades/t.txt', t, fmt = '%f', delimiter = '\t', newline = '\r\n')
	np.savetxt('../../../paper/images/data_files/underwater_structural_trades/cost.txt', cost, fmt = '%f', delimiter = '\t', newline = '\r\n')

