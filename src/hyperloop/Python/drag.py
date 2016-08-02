from __future__ import print_function

import numpy as np
from scipy import interpolate as interp
from openmdao.api import IndepVarComp, Component, Group, Problem
import matplotlib.pylab as plt

class Drag(Component):
	def __init__(self):
		super(Drag, self).__init__()

		self.add_param('M_pod', val = .8, desc = 'Pod Mach Number', units = 'unitless')
		self.add_param('mach_array', val = np.zeros((1,7)), desc = 'Array of Mach numebrs from CFD', units = 'unitless')
		self.add_param('cd_array', val = np.zeros((1,7)), desc = 'Array of Drag values from CFD', units = 'unitless')

		self.add_output('Cd', val = 1.0, desc = 'Drag Coefficient', units = 'unitless')

	def solve_nonlinear(self, p, u ,r):

		f = interp.UnivariateSpline(p['mach_array'], p['cd_array'])
		u['Cd'] = float(f(p['M_pod']))

if __name__ == '__main__':
	top = Problem()
	root = top.root = Group()

	root.add('p', Drag())
	root.add('p1', IndepVarComp('M_pod', .8))
	root.connect('p1.M_pod', 'p.M_pod')

	top.setup()

	top['p.mach_array'] = np.array([ 0.5  ,  0.6  ,  0.625,  0.65 ,  0.675,  0.7  ,  0.725])
	top['p.cd_array'] = np.array([ 0.04241176,  0.03947743,  0.04061261,  0.04464372,  0.05726695,
        0.07248304,  0.08451007])

	top.run()
	print(top['p.Cd'])