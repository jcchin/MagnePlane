from __future__ import print_function

import numpy as np
from openmdao.api import IndepVarComp, Component, Group, Problem, ExecComp
import matplotlib.pylab as plt

class TicketCost(Component):
	'''
	Notes
	-------
	This Component takes into account various cost figures from the system model and combines them to estimate tickt cost per passenger.

	Params
	-------
	length_cost : float
		Cost of materials per unit length. Default value is 2.437e6 USD/km
	pod_cost : float
		Cost per individual pod. Default value is 1.0e6 USD.
	capital_cost : float
		Estimate of overhead capital cost. Default value is 1.0e10 USD.
	energy_cost : float
		Cost of electricity. Default value is .13 USD/kWh
	ib : float
		Bond interest rate. Default value is .04
	bm : float
		Bond maturity. Default value is 20.0 years.
	operating_time : float
		operating time per day. Default value is 16.0*3600 s
	JtokWh : float
		Convert J to kWh. Default value is J/kWh
	m_pod : float
		Pod mass. Default value is 3100 kg
	n_passengers : float
		Number of passengers. Default value is 28.0
	pod_period : float
		Time in between pod departures. Default value is 120.0 s
	avg_speed : float
		average pod speed. Default value is 286.86 m/s
	track_length : float
		length of the track. Default value is 600e3 m
	pod_power : float
		Power consumption of the pod. Default value is 1.5e6 W
	prop_power : float
		power of an individual propulsion section. Default value is 350e3 W
	vac_power : float
		Power of the vacuum pumps. Default value is 71.049e6 W
	alpha : float
		percent of vacuum power used in steady state. Default value is .0001
	vf : float
		Pod top speed. Default value is 286.86 m/s
	g : float
		Gravity. Default value is 9.81 m/s/s
	Cd : float
		Pod drag coefficient. Default value is .2
	S : float
		Pod planform area. Default value is 40.42 m**2
	p_tunnel : float
		Tunnel pressure. Default value is 850.0 Pa
	T_tunnel : float
		Tunnel temperature. Default value is 320 K
	R : float
		Ideal gas constant. Default value is 287 J/kg/K
	eta : float
		Efficiency of propulsion system
	D_mag : float
		Magnetic drag. Default value is (9.81*3100.0)/200.0 N
	thrust_time : float
		Time spent during a propulsive section. Default value is 1.5 s
	prop_period : float
		distance between pripulsion sections. Defualt value is 25.0e3 km

	Returns
	-------
	ticket_cost : float
		cost of individual ticket. Default value is 0.0 USD
	prop_energy_cost : float
		cost of energy used by propulsion section per year. Default value is 0.0 USD

	'''
	def __init__(self):

		super(TicketCost, self).__init__()

		self.add_param('land_cost', val = 2.437e6, desc = 'Cost of materials over land per unit length', units = 'USD/km')
		self.add_param('water_cost', val = 389.346941e3, desc = 'Cost of materials underwater per unit length', units = 'USD/km')
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
		self.add_param('land_length', val = 600e3, desc = 'Length traveled over land', units = 'm')
		self.add_param('water_length', val = 0.0e3, desc = 'Length traveled underwater', units = 'm')
		self.add_param('pod_power', val = 1.5e6, desc = 'Power required by pod motor', units = 'W')
		self.add_param('prop_power', val = 350.0e3, desc = 'Power of single propulsive section', units = 'W')
		self.add_param('vac_power', val = 71.049e6, desc = 'Power of vacuums', units = 'W')
		self.add_param('steady_vac_power', val = 950.0e3, desc = 'Steady State run power of vacuum pumps', units = 'W')
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

		self.add_output('num_pods', val = 0.0, desc = 'Number of Pods', units = 'unitless')
		self.add_output('ticket_cost', val = 0.0, desc = 'Ticket cost', units = 'USD')
		self.add_output('prop_energy_cost', val = 0.0, desc = 'Cost of propulsion energy', units = 'USD')

	def solve_nonlinear(self, p, u,r):

		land_cost = p['land_cost']
		water_cost = p['water_cost']
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
		land_length = p['land_length']
		water_length = p['water_length']
		pod_power = p['pod_power']
		prop_power = p['prop_power']
		vac_power = p['vac_power']
		steady_vac_power = -1.0*p['steady_vac_power']
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

		length_cost = ((water_length/track_length)*water_cost) + ((land_length/track_length)*land_cost)
		pod_frequency = 1.0/pod_period
		num_pods = np.ceil((track_length/avg_speed)*pod_frequency)
		flights_per_pod = (operating_time*pod_frequency)/num_pods
		energy_per_flight = pod_power*(track_length/avg_speed)*.9
		pod_energy = energy_per_flight*flights_per_pod*num_pods*JtokWh
		vac_energy = steady_vac_power*operating_time*JtokWh

		rho = p_tunnel/(R*T_tunnel)
		start_distance = (vf**2)/(2*g)
		start_energy = ((m_pod*g+D_mag)*start_distance + (.5*Cd*rho*g*S*(start_distance**2)))/eta
		num_thrusts = track_length/prop_period
		prop_energy = (num_thrusts*thrust_time*prop_power + start_energy)*flights_per_pod*num_pods*JtokWh
		tube_energy = prop_energy + vac_energy

		u['num_pods'] = num_pods
		u['prop_energy_cost'] = prop_energy*energy_cost*365
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
	top.run()

	print(top['p.ticket_cost'])

	# n_passengers = np.linspace(10,100,num = 90)
	# ticket_cost = np.zeros((1, len(n_passengers)))

	# for i in range(len(n_passengers)):

	# 	top['n_passengers'] = n_passengers[i]
	# 	top.run()
	# 	ticket_cost[0, i] = top['p.ticket_cost']

	# plt.plot(n_passengers*(175200.0/(1.0e6)), ticket_cost[0,:])
	# plt.show()
