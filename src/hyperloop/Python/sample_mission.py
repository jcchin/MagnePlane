from __future__ import print_function

import numpy as np
from openmdao.api import IndepVarComp, Component, Group, Problem

class SampleMission(Component):
	'''
	Notes
	-------
	This component outputs relevant mission parameters assuming a flat trajectore from LA to SF
	'''

	def __init__(self):
		super(SampleMission, self).__init__()

		self.add_param('p_tunnel', 850.0, desc = 'Tunnel pressure', units = 'Pa')
		self.add_param('T_tunnel', 320.0, desc = 'T_tunnel', units = 'K')
		self.add_param('Cd', .25, desc = 'Pod drag coefficient', units = 'unitless')
		self.add_param('S', 40.0, desc = 'Pod planform area', units = 'm**2')
		self.add_param('m_pod', 10000.0, desc = 'Pod mass', units = 'kg')
		self.add_param('D_mag', (10000.0*9.81)/200.0, desc = 'Magnetic Drag', units = 'N')
		self.add_param('ram_drag', 1855.44, desc = 'Pod planform area', units = 'N')
		self.add_param('nozzle_thrust', 5503.12, desc = 'Pod planform area', units = 'N')
		self.add_param('M_pod', .8, desc = 'Pod mach number', units = 'unitless')
		self.add_param('g', 9.81, desc = 'gravity', units = 'm/s**2')
		self.add_param('R', 287.0, desc = 'Ideal gas constant', units = 'J/(kg*K)')
		self.add_param('gam', 1.4, desc = 'Pod planform area', units = 'm**2')
		self.add_param('theta', 0.0, desc = 'Track elevation angle', units= 'rad')
		self.add_param('track_length', val = 600.0e3, desc = 'Track Length', units = 'm')

		self.add_output('dx_start', 1.0, desc = 'Start up distance', units = 'm')
		self.add_output('dx_boost', 1.0, desc = 'Booster length', units = 'm')
		self.add_output('boost_time', 1.0, desc = 'Time on booster', units = 's')
		self.add_output('prop_period', 1.0, desc = 'Distance between boosters', units = 'm')
		self.add_output('num_thrust', 1.0, desc = 'Start up distance', units = 'm')
		self.add_output('coast_time', 1.0, desc = 'Start up distance', units = 'm')

	def solve_nonlinear(self, p, u, r):
		p_tunnel = p['p_tunnel']
		T_tunnel = p['T_tunnel']
		Cd = p['Cd']
		S = p['S']
		m_pod = p['m_pod']
		D_mag = p['D_mag']
		ram_drag = p['ram_drag']
		nozzle_thrust = p['nozzle_thrust']
		M_pod = p['M_pod']
		g = p['g']
		R = p['R']
		gam = p['gam']
		theta = p['theta']
		track_length = p['track_length']

		vf = M_pod * np.sqrt(gam*R*T_tunnel)
		v0 = vf - 15.0
		
		dx_start = (vf**2)/(2*g)
		dx_boost = ((vf**2) - (v0**2))/(2*g)
		boost_time = (vf - v0)/g

		rho = p_tunnel/(R*T_tunnel)
		
		#Initialize numerical integration
		i = x = 0.0
		v = vf
		dt = .01
		net_thrust = nozzle_thrust - ram_drag
		
		while v > v0:
			#Integrate numerically using predictor-corrector method for velocity
			v_old = v
			v_init = ((dt/m_pod)*(net_thrust - (.5*Cd*rho*S*(v**2.0)) - (m_pod*g*np.sin(theta)) - D_mag)) + v
			v = v + ((dt/m_pod)*(net_thrust - (.5*Cd*rho*S*(v_init**2.0)) - (m_pod*g*np.sin(theta)) - D_mag) + \
				((dt/m_pod)*(net_thrust - (.5*Cd*rho*S*(v**2.0)) - (m_pod*g*np.sin(theta)) - D_mag)))/2.0
			x = x + ((v_old+v)/2.0)*dt
			if v > v_old:
				print('thrust greater than drag')
				break

			i = i + 1.0

			if i>100000:
				break

		u['dx_start'] = dx_start
		u['dx_boost'] = dx_boost
		u['boost_time'] = boost_time
		u['prop_period'] = x
		u['num_thrust'] = np.ceil(track_length/x)
		u['coast_time'] = i*dt

if __name__ == '__main__':
	top = Problem()
	root = top.root = Group()

	root.add('p', SampleMission())

	top.setup()
	top.run()

	print('Start up disance  			%f' % top['p.dx_start'])
	print('Booster length 				%f' % top['p.dx_boost'])
	print('Booster time 				%f' % top['p.boost_time'])
	print('Distance between boosters		%f' % top['p.prop_period'])
	print('Number of boosters 			%.0f' % top['p.num_thrust'])
	print('Coasting time 				%f' % top['p.coast_time'])


