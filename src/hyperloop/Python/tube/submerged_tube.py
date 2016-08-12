from __future__ import print_function

import numpy as np
import matplotlib.pylab as plt
from openmdao.api import IndepVarComp, Component, Group, Problem

class SubmergedTube(Component):
	'''
	Params
	-------

	p_tube : float
		Tube pressure. Default valut is 850 Pa
	A_tube : float
		Cross sectional area of tube. Default valut is 30 m**2
	Su : float
		Ultimate strength pf tube material. Default valut is 400.0e6 Pa
	SF : float
		Tube safety factor. Default valut is 5.0
	rho_water : float
		Density of sea water. Default value is 1025.0e3 kg/m**3
	depth : float
		Depth of the tube. Default value is 10.0m
	g : float
		Gravitational acceleration. Default value is 9.81 m/s**2
	Pa : float
		Ambient pressure at sea level. Default value is 101.3e3 Pa
	unit_cost_tube : float
		Cost of tube materials per unit mass. Default value is .3307 USD/kg

	Returns
	-------
	t : float
		Returns tube thickness in m
	dF_buoyancy : float
		Returns buoyant force on tube per unit length in N/m
	material cost : float
		Returns material cost of tube per unit length in USD/m
	m_prime : float
		Returns mass of tube per unit length in kg/m

	'''
	def __init__(self):
		super(SubmergedTube, self).__init__()

		self.add_param('p_tube', val = 850.0, desc = 'Tube pressure', units = 'Pa')
		self.add_param('A_tube', val = 30.0, desc = 'Tube cross sectional area', units = 'm**2')
		self.add_param('Su', val = 400.0e6, desc = 'Tube material yield strength', units = 'Pa')
		self.add_param('E_tube', val = 200.0e9, desc = 'Young\'s Modulus of the tube', units = 'Pa')
		self.add_param('v_tube', val = .33, desc = 'Poissoin\'s ratio of the tube')
		self.add_param('SF', val = 5.0, desc = 'Safety factor', units = 'unitless')
		self.add_param('rho_water', val = 1025.0, desc = 'Density of sea wateer', units = 'kg/m**3')
		self.add_param('rho_tube', val = 7800.0, desc = 'Density of tube material', units = 'kg/m**3')
		self.add_param('depth', val = 10.0, desc = 'Tunnel depth underwater', units = 'm')
		self.add_param('g', val = 9.81, desc = 'Gravity', units = 'm/s**2')
		self.add_param('Pa', val = 101.3e3, desc = 'Ambient pressure at sea level', units = 'Pa')
		self.add_param('unit_cost_tube', val = .3307, desc = 'Cost of tube material per unit mass', units = 'USD/kg')

		self.add_output('t', val = 1.0, desc = 'Tube thickness', units = 'm')
		self.add_output('dF_buoyancy', val = 1.0, desc = 'Sectional buoyant force', units = 'N/m')
		self.add_output('material_cost', val = 1.0, desc = 'Material cost per unit length', units = 'USD/m')
		self.add_output('m_prime', val = 1.0, desc = 'Tube mass per unit length')
		self.add_output('t_crit', 1.0, desc = 'Critical buckling thickness', units = 'm')

	def solve_nonlinear(self, p, u, r):
		'''
		t = (p*r)/(Su/SF); p = pa + rho*g*h; F_buoyant/L = rho*A_tube*g
		'''
		p_ambient = p['Pa'] + p['rho_water']*p['depth']*p['g']
		dp = p_ambient - p['p_tube']
		r = np.sqrt(p['A_tube']/np.pi)
		t = ((p_ambient-p['p_tube'])*r)/(p['Su']/p['SF'])
		t_crit = r * (((4.0 * dp * (1.0 - (p['v_tube']**2))) / p['E_tube'])**(1.0 / 3.0))
		
		if t>t_crit:
			u['t'] = t
		else:
			u['t'] = t_crit
		u['dF_buoyancy'] = p['rho_water']*p['g']*p['A_tube']
		u['material_cost'] = (np.pi*((r+u['t'])**2)-p['A_tube'])*p['rho_tube']*p['unit_cost_tube']
		u['m_prime'] = (np.pi*((r+u['t'])**2)-p['A_tube'])*p['rho_tube']
		u['t_crit'] = t_crit

if __name__ == '__main__':
	top = Problem()
	root = top.root = Group()

	root.add('p', SubmergedTube())

	top.setup()
	top['p.p_tube'] = 850.0
	# top['p.m_pod']= 10000.0

	import csv

	# f = open('/Users/kennethdecker/Desktop/Paper figures/water_structural_trades.csv', 'wt')
	# writer = csv.writer(f)
	# writer.writerow(('A_tube', 't 20m', 't 40m', 't 60m', 'cost 20m', 'cost 40m', 'cost 60m'))

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


		# writer.writerow((A_tube[i], t[0,i], t[1,i], t[2,i], cost[0,i], cost[1,i], cost[2,i]))

	# f.close()
	line1, = plt.plot(A_tube, t[0,:], 'b-', linewidth = 2.0, label = 'depth = 20 m')
	line2, = plt.plot(A_tube, t[1,:], 'r-', linewidth = 2.0, label = 'depth = 40 m')
	line3, = plt.plot(A_tube, t[2,:], 'g-', linewidth = 2.0, label = 'depth = 60 m')
	plt.xlabel('Tube Area (m^2)', fontsize = 12, fontweight = 'bold')
	plt.ylabel('Tube Thickness (m)', fontsize = 12, fontweight = 'bold')
	plt.grid('on')
	plt.legend(handles = [line1, line2, line3], loc = 2)
	plt.show()
	line1, = plt.plot(A_tube, cost[0,:], 'b-', linewidth = 2.0, label = 'depth = 20 m')
	line2, = plt.plot(A_tube, cost[1,:], 'r-', linewidth = 2.0, label = 'depth = 40 m')
	line3, = plt.plot(A_tube, cost[2,:], 'g-', linewidth = 2.0, label = 'depth = 60 m')
	plt.xlabel('Tube Area (m^2)', fontsize = 12, fontweight = 'bold')
	plt.ylabel('Cost (USD/m)', fontsize = 12, fontweight = 'bold')
	plt.grid('on')
	plt.legend(handles = [line1, line2, line3], loc = 2)
	plt.show()
	print(top['p.t_crit'])

