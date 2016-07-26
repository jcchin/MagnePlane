from __future__ import print_function

import numpy as np
from openmdao.api import IndepVarComp, Component, Group, Problem, ExecComp
import matplotlib.pylab as plt

class TicketCost(Component):
	def __init__(self):

		super(TicketCost, self).__init__()

		self.add_param('length_cost', val = 2.437e6, desc = 'Cost of materials per unit length', units = 'USD/km')
		self.add_param('pod_cost', val = 1.0e6, desc = 'Cost of individual pod', units = 'USD')
		self.add_param('capital_cost', val = 1.0e10, desc = 'Estimate of overhead capital cost', units = 'USD')
		self.add_param('energy_cost', val = .13, desc = 'Cost of electricity', units = 'USD/kW/h')
		self.add_param('ib', val = .04, desc = 'Bond interest rate', units = 'unitless')
		self.add_param('bm', val = 20.0, desc = 'Bond maturity', units = 'yr')
		self.add_param('operating_time', val = 16.0*3600, desc = 'Operating time per day', units = 's')
		self.add_param('JtokWh', val = 2.7778e-7, desc = 'Convert Joules to kWh', units = '(kw*h)/J')
		self.add_param('m_pod', val = 3100.0, desc = 'Pod Mass', units = 'kg')
		self.add_param('n_passengers', val = 28.0, desc = 'number of passengers', units = 'unitless')
		self.add_param('pod_period', val = 120.0, desc = 'Time in between departures', units = 's')
		self.add_param('avg_speed', val = 286.86, desc = 'Average Pod Speed', units = 'm/s')
		self.add_param('track_length', val = 600.0e3, desc = 'Track Length', units = 'm')
		self.add_param('pod_power', val = 1.5e6, desc = 'Power required by pod motor', units = 'W')
		self.add_param('prop_power', val = 350.0e3, desc = 'Power of single propulsive section', units = 'W')
		self.add_param('vac_power', val = 71.049e6, desc = 'Power of vacuums', units = 'W')
		self.add_param('alpha', val = .0001, desc = 'Steady State run capacity of vacuum pumps', units = 'unitless')
		self.add_param('vf', val = 286.86, desc = 'Pod top speed', units = 'm/s')
		self.add_param('g', val = 9.81, desc = 'Gravity', units = 'm/s/s')
		self.add_param('Cd', val = .2, desc = 'Pod drag coefficient', units = 'unitless')
		self.add_param('S', val = 40.42, desc = 'Pod planform area', units = 'm**2')
		self.add_param('p_tunnel', val = 850.0, desc = 'Tunnel Pressure', units = 'Pa')
		self.add_param('T_tunnel', val = 320.0, desc = 'Tunnel Temperature', units = 'K')
		self.add_param('R', val = 287.0, desc = 'Ideal gas constant', units = 'J/kg/K')
		self.add_param('eta', val = .8, desc = 'Propulsive efficiency', units = 'unitless')
		self.add_param('D_mag', val = (9.81*3100.0)/200.0, desc = 'Magnetic Drag', units = 'N')
		self.add_param('thrust_time', val = 1.5, desc = 'Time that pod is over propulsive section', units = 's')
		self.add_param('prop_period', val = 25.0e3, desc = 'distance between propulsive sections', units = 'm')

		self.add_output('ticket_cost', val = 0.0, desc = 'Ticket cost', units = 'USD')

	def solve_nonlinear(self, p, u,r):


		length_cost = p['length_cost']
		pod_cost= p['pod_cost']
		capital_cost = p['capital_cost']
		energy_cost = p['energy_cost']
		ib = p['ib']
		bm = p['bm']
		operating_time = p['operating_time']
		JtokWh = p['JtokWh']
		m_pod = p['m_pod']
		n_passengers = p['n_passengers']
		pod_period = p['pod_period']
		avg_speed = p['avg_speed']
		track_length = p['track_length']
		pod_power = p['pod_power']
		prop_power = p['prop_power']
		vac_power = p['vac_power']
		alpha = p['alpha']
		vf = p['vf']
		g = p['g']
		Cd = p['Cd']
		S = p['S']
		p_tunnel = p['p_tunnel']
		T_tunnel = p['T_tunnel']
		R = p['R']
		eta = p['eta']
		D_mag = p['D_mag']
		thrust_time = p['thrust_time']
		prop_period = p['prop_period']

		pod_frequency = 1.0/pod_period
		num_pods = np.ceil((track_length/avg_speed)*pod_frequency)
		flights_per_pod = (operating_time*pod_frequency)/num_pods
		energy_per_flight = pod_power*(track_length/avg_speed)*.9
		pod_energy = energy_per_flight*flights_per_pod*num_pods*JtokWh
		vac_energy = vac_power*operating_time*alpha*JtokWh

		rho = p_tunnel/(R*T_tunnel)
		start_distance = (vf**2)/(2*g)
		start_energy = ((m_pod*g+D_mag)*start_distance + (.5*Cd*rho*g*S*(start_distance**2)))/eta
		num_thrusts = track_length/prop_period
		prop_energy = (num_thrusts*thrust_time*prop_power + start_energy)*flights_per_pod*num_pods*JtokWh
		tube_energy = prop_energy + vac_energy
		print(prop_energy)
		print(tube_energy)
		print(pod_energy)

		u['ticket_cost'] = cost_ticket = (length_cost*(track_length/1000.0) + pod_cost*num_pods + capital_cost*(1.0+ib) + \
			energy_cost*(tube_energy + pod_energy)*365.0)/(n_passengers*pod_frequency*bm*365.0*24.0*3600.0)

if __name__ == '__main__':

	top = Problem()
	root = top.root = Group()

	params = (('n_passengers', 28.0),
		('track_length', 600.0e3, {'units' : 'm'}))

	root.add('p', TicketCost())
	root.add('des_vars', IndepVarComp(params), promotes = ['n_passengers'])

	root.connect('n_passengers', 'p.n_passengers')
	root.connect('des_vars.track_length', 'p.track_length')

	top.setup()

	n_passengers = np.linspace(10,100,num = 90)
	ticket_cost = np.zeros((1, len(n_passengers)))

	for i in range(len(n_passengers)):

		top['n_passengers'] = n_passengers[i]
		top.run()
		ticket_cost[0, i] = top['p.ticket_cost']

	plt.plot(n_passengers*(175200.0/(1.0e6)), ticket_cost[0,:])
	plt.show()
