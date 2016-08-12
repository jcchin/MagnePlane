import numpy as np
import matplotlib.pylab as plt
from openmdao.api import Group, Problem, IndepVarComp

from hyperloop.Python import tube_and_pod

# def create_problem(component):
#     root = Group()
#     prob = Problem(root)
#     prob.root.add('comp', component)
#     return prob

# class PressureTradeStudy(object):
#     def test_case1_vs_npss(self):

#         component = tube_and_pod.TubeAndPod()
#         prob = create_problem(component)

if __name__ == '__main__':
	prob = Problem()
	root = prob.root = Group()

	root.add('TubeAndPod', tube_and_pod.TubeAndPod())

	params = (('tube_pressure', 850.0, {'units' : 'Pa'}),
				('pressure_initial', 760.2, {'units' : 'torr'}),
				('num_pods', 18.),
				('pwr', 18.5, {'units' : 'kW'}),
				('speed', 163333.3, {'units' : 'L/min'}),
				('time_down', 1440.0, {'units' : 'min'}),
				('gamma', .8, {'units' : 'unitless'}),
				('pump_weight', 715.0, {'units' : 'kg'}),
				('electricity_price', 0.13, {'units' : 'USD/(kW*h)'}),
				('tube_thickness', .0415014, {'units' : 'm'}),
				('tube_length', 480000., {'units' : 'm'}),
				('vf', 286.85, {'units' : 'm/s'}),
				('v0', 286.85-15.0, {'units' : 'm/s'}),
				('time_thrust', 1.5, {'units' : 's'}),
				('pod_mach', .8, {'units': 'unitless'}),
				('comp_inlet_area', 2.3884, {'units': 'm**2'}),
				('comp_PR', 6.0, {'units': 'unitless'}),
				('PsE', 0.05588, {'units': 'psi'}),
				('des_time', 1.0),
				('time_of_flight', 1.0),
				('motor_max_current', 800.0),
				('motor_LD_ratio', 0.83),
				('motor_oversize_factor', 1.0),
				('inverter_efficiency', 1.0),
				('battery_cross_section_area', 15000.0, {'units': 'cm**2'}),
				('n_passengers', 28.),
				('A_payload', 2.3248, {'units' : 'm**2'}),
				('r_pylon', 0.232, {'units' : 'm'}),
				('h', 10.0, {'units' : 'm'}),
				('vel_b', 23.0, {'units': 'm/s'}),
				('h_lev', 0.01, {'unit': 'm'}),
				('vel', 286.86, {'units': 'm/s'}),
				('pod_period', 120.0, {'units' : 's'}),
				('ib', .04),
				('bm', 20.0, {'units' : 'yr'}),
				('track_length', 600.0, {'units' : 'km'}),
				('avg_speed', 286.86, {'units' : 'm/s'}),
				('depth', 10.0, {'units' : 'm'}),
				('land_length', 600.0e3, {'units' : 'm'}),
				('water_length', 0.0e3, {'units' : 'm'}),
				('W', 1.0, {'units' : 'kg/s'}),
				('operating_time', 16.0*3600.0, {'units' : 's'})
				)

	prob.root.add('des_vars', IndepVarComp(params))
	prob.root.connect('des_vars.tube_pressure', 'TubeAndPod.tube_pressure')
	prob.root.connect('des_vars.pressure_initial', 'TubeAndPod.pressure_initial')
	prob.root.connect('des_vars.num_pods', 'TubeAndPod.num_pods')
	prob.root.connect('des_vars.pwr','TubeAndPod.pwr')
	prob.root.connect('des_vars.speed', 'TubeAndPod.speed')
	prob.root.connect('des_vars.time_down', 'TubeAndPod.time_down')
	prob.root.connect('des_vars.gamma','TubeAndPod.gamma')
	prob.root.connect('des_vars.pump_weight','TubeAndPod.pump_weight')
	prob.root.connect('des_vars.electricity_price','TubeAndPod.electricity_price')
	prob.root.connect('des_vars.tube_thickness', 'TubeAndPod.tube_thickness')
	prob.root.connect('des_vars.tube_length', 'TubeAndPod.tube_length')
	prob.root.connect('des_vars.h', 'TubeAndPod.h')
	prob.root.connect('des_vars.r_pylon', 'TubeAndPod.r_pylon')
	prob.root.connect('des_vars.vf', 'TubeAndPod.vf')
	prob.root.connect('des_vars.v0', 'TubeAndPod.v0')
	prob.root.connect('des_vars.time_thrust', 'TubeAndPod.time_thrust')
	prob.root.connect('des_vars.pod_mach', 'TubeAndPod.pod_mach')
	prob.root.connect('des_vars.comp_inlet_area', 'TubeAndPod.comp_inlet_area')
	prob.root.connect('des_vars.comp_PR', 'TubeAndPod.comp.map.PRdes')
	prob.root.connect('des_vars.PsE', 'TubeAndPod.nozzle.Ps_exhaust')
	prob.root.connect('des_vars.des_time', 'TubeAndPod.des_time')
	prob.root.connect('des_vars.time_of_flight', 'TubeAndPod.time_of_flight')
	prob.root.connect('des_vars.motor_max_current', 'TubeAndPod.motor_max_current')
	prob.root.connect('des_vars.motor_LD_ratio', 'TubeAndPod.motor_LD_ratio')
	prob.root.connect('des_vars.motor_oversize_factor', 'TubeAndPod.motor_oversize_factor')
	prob.root.connect('des_vars.inverter_efficiency', 'TubeAndPod.inverter_efficiency')
	prob.root.connect('des_vars.battery_cross_section_area', 'TubeAndPod.battery_cross_section_area')
	prob.root.connect('des_vars.n_passengers', 'TubeAndPod.n_passengers')
	prob.root.connect('des_vars.A_payload', 'TubeAndPod.A_payload')
	prob.root.connect('des_vars.vel_b', 'TubeAndPod.vel_b')
	prob.root.connect('des_vars.h_lev', 'TubeAndPod.h_lev')
	prob.root.connect('des_vars.vel', 'TubeAndPod.vel')
	prob.root.connect('des_vars.pod_period', 'TubeAndPod.cost.pod_period')
	prob.root.connect('des_vars.ib', 'TubeAndPod.cost.ib')
	prob.root.connect('des_vars.bm', 'TubeAndPod.cost.bm')
	prob.root.connect('des_vars.track_length', 'TubeAndPod.track_length')
	prob.root.connect('des_vars.avg_speed', 'TubeAndPod.cost.avg_speed')
	prob.root.connect('des_vars.land_length', 'TubeAndPod.land_length')
	prob.root.connect('des_vars.water_length', 'TubeAndPod.water_length')
	prob.root.connect('des_vars.operating_time', 'TubeAndPod.operating_time')
	prob.root.connect('des_vars.W', 'TubeAndPod.fl_start.W')

	prob.setup()

	p_tunnel = np.concatenate((np.linspace(50.0, 1000.0, num = 20, endpoint = False), np.linspace(1000.0,4000.0, num = 20, endpoint = True)))
	A_tube = np.zeros((1, len(p_tunnel)))
	Re = np.zeros((1, len(p_tunnel)))
	T_tunnel = np.zeros((1, len(p_tunnel)))
	L_pod = np.zeros((1, len(p_tunnel)))
	Drag = np.zeros((1, len(p_tunnel)))
	power = np.zeros((1, len(p_tunnel)))
	steady_vac = np.zeros((1,len(p_tunnel)))
	total_energy = np.zeros((1, len(p_tunnel)))
	thrust = np.zeros((1, len(p_tunnel)))

	for i in range(len(p_tunnel)):

		prob['des_vars.tube_pressure'] = p_tunnel[i]

		prob.run()

		A_tube[0,i] = prob['TubeAndPod.pod.A_tube']
		Re[0,i] = prob['TubeAndPod.pod.pod_mach.Re']
		T_tunnel[0,i] = prob['TubeAndPod.tube.temp_boundary']
		L_pod[0,i] = prob['TubeAndPod.L_pod']
		power[0,i] = -1.0*prob['TubeAndPod.pod.cycle.comp.power']
		steady_vac[0,i] = -1.0*prob['TubeAndPod.tube.comp.power']
		total_energy[0,i] = prob['TubeAndPod.cost.total_energy_cost']

		print(i)


	np.savetxt('../../../paper/images/data_files/pressure_trades/p_tunnel.txt', p_tunnel, fmt = '%f', delimiter = '\t', newline = '\r\n')
	np.savetxt('../../../paper/images/data_files/pressure_trades/Re.txt', Re, fmt = '%f', delimiter = '\t', newline = '\r\n')
	np.savetxt('../../../paper/images/data_files/pressure_trades/A_tube.txt', A_tube, fmt = '%f', delimiter = '\t', newline = '\r\n')
	np.savetxt('../../../paper/images/data_files/pressure_trades/T_tunnel.txt', T_tunnel, fmt = '%f', delimiter = '\t', newline = '\r\n')
	np.savetxt('../../../paper/images/data_files/pressure_trades/L_pod.txt', L_pod, fmt = '%f', delimiter = '\t', newline = '\r\n')
	np.savetxt('../../../paper/images/data_files/pressure_trades/comp_power.txt', power, fmt = '%f', delimiter = '\t', newline = '\r\n')
	np.savetxt('../../../paper/images/data_files/pressure_trades/vac_power.txt', steady_vac, fmt = '%f', delimiter = '\t', newline = '\r\n')
	np.savetxt('../../../paper/images/data_files/pressure_trades/total_energy.txt', total_energy, fmt = '%f', delimiter = '\t', newline = '\r\n')
