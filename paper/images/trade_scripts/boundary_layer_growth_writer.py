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
        ('length_calc', False))

    root.add('input_vars', IndepVarComp(params), promotes = ['delta_star', 'A_pod', 'L_pod', 'length_calc'])
    root.add('p', boundary_layer_sensitivity.BoundaryLayerSensitivity())

    root.connect('delta_star', 'p.delta_star')
    root.connect('A_pod', 'p.A_pod')
    root.connect('L_pod', 'p.L')
    root.connect('length_calc', 'p.length_calc')

    top.setup()

    delta_star = np.linspace(.02, .12, num = 50)
    A_pod = np.linspace(2, 3, num = 3)

    A_tube = np.zeros((len(A_pod), len(delta_star)))

    for i in range(len(delta_star)):
        for j in range(len(A_pod)):
            top['delta_star'] = delta_star[i]
            top['A_pod'] = A_pod[j]

            top.run()

            A_tube[j,i] = top['p.A_tube']

    np.savetxt('../../../paper/images/data_files/boundary_layer_growth_trades/delta_star.txt', delta_star, fmt = '%f', delimiter = '\t', newline = '\r\n')
    np.savetxt('../../../paper/images/data_files/boundary_layer_growth_trades/A_tube.txt', A_tube, fmt = '%f', delimiter = '\t', newline = '\r\n')

