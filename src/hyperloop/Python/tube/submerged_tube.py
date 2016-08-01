from __future__ import print_function

import numpy as np
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


	def solve_nonlinear(self, p, u, r):
		'''
		t = (p*r)/(Su/SF); p = pa + rho*g*h; F_buoyant/L = rho*A_tube*g
		'''
		p_ambient = p['Pa'] + p['rho_water']*p['depth']*p['g']
		r = np.sqrt(p['A_tube']/np.pi)
		t = ((p_ambient-p['p_tube'])*r)/(p['Su']/p['SF'])
		
		u['t'] = t
		u['dF_buoyancy'] = p['rho_water']*p['g']*p['A_tube']
		u['material_cost'] = (np.pi*((r+t)**2)-p['A_tube'])*p['rho_tube']*p['unit_cost_tube']
		u['m_prime'] = (np.pi*((r+t)**2)-p['A_tube'])*p['rho_tube']

if __name__ == '__main__':
	top = Problem()
	root = top.root = Group()

	root.add('p', SubmergedTube())

	top.setup()
	top.run()

	print('\n')
	print('thickness %f' % top['p.t'])
	print('buoyancy %f' % top['p.dF_buoyancy'])
	print('material cost %f' % top['p.material_cost'])
	print('Sectional mass %f' % top['p.m_prime'])


