import numpy as np
import matplotlib.pylab as plt
from openmdao.api import Group, Problem, IndepVarComp

from hyperloop.Python import boundary_layer_sensitivity

if __name__ == '__main__':

	top = Problem()
	root = top.root = Group()

	params = (
		('delta_star', .02, {'units' : 'm'}),
		('A_pod', 2.0, {'units' : 'm**2'}),
		('L_pod', 22.0, {'units' : 'm'}),
		('length_calc', True))

	root.add('input_vars', IndepVarComp(params), promotes = ['delta_star', 'A_pod', 'L_pod', 'length_calc'])
	root.add('p', boundary_layer_sensitivity.BoundaryLayerSensitivity())

	root.connect('delta_star', 'p.delta_star')
	root.connect('A_pod', 'p.A_pod')
	root.connect('L_pod', 'p.L')
	root.connect('length_calc', 'p.length_calc')

	top.setup()

	L_pod = np.linspace(20.0, 40.0, num = 50)
	A_tube = np.zeros((1, len(L_pod)))
	
	for i in range(len(L_pod)):
		top['L_pod'] = L_pod[i]

		top.run()

		A_tube[0, i] = top['p.A_tube']

	np.savetxt('../../../paper/images/data_files/boundary_layer_length_trades/L_pod.txt', L_pod, fmt = '%f', delimiter = '\t', newline = '\r\n')
	np.savetxt('../../../paper/images/data_files/boundary_layer_length_trades/A_tube.txt', A_tube, fmt = '%f', delimiter = '\t', newline = '\r\n')