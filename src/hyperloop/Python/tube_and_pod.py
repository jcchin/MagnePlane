"""
Group for Tube and Pod components containing the following two sub-groups:
Pod and Tube
"""

from openmdao.api import Component, Group, Problem, IndepVarComp
from hyperloop.Python.tube.tube_group import TubeGroup
from hyperloop.Python.pod.pod_group import PodGroup

class TubeAndPod(Group):
	def __init__(self):
		super(TubeAndPod, self).__init__()

		self.add('pod', PodGroup(), promotes=['p_tube', 'M_pod', 'A_payload', 'M_duct',
											  'w_track', 'prc', 'n_passengers', 'T_tunnel',
											  'p_tunnel', 'motor_max_current', 'inverter_efficiency',
											  'des_time', 'time_of_flight', 'S', 'nozzle.Fl_O:tot:T',
											  'nozzle.Fl_O:stat:W', 'nozzle.Fg', 'inlet.F_ram'])
		self.add('tube', TubeGroup(), promotes=['length_tube', 'h'])

		self.connect('tube.T_ambient', 'pod.T_ambient')
		self.connect('pod.A_tube', 'tube.tube_area')
		self.connect('pod.pod_mass', 'tube.m_pod')
		self.connect('pod.S', 'tube.S')
		self.connect('pod.mag_drag', 'tube.D_mag')
		self.connect('pod.nozzle.Fg', 'tube.nozzle_thrust')
		self.connect('pod.inlet.F_ram', 'tube.ram_drag')


if __name__ == '__main__':

    prob = Problem()
    root = prob.root = Group()
    root.add('TubeAndPod', TubeAndPod())

    params = (('p_tube', 850.0, {'units' : 'Pa'}),
    		 ('M_pod', .8, {'units' : 'unitless'}),
             ('A_payload', 1.4),
             ('w_track', 2.0, {'units': 'm'}),
             ('prc', 12.5, {'units' : 'unitless'}),
    		 ('n_passengers', 0),
    		 ('T_tunnel', 0),
    		 ('p_tunnel', 0),
    		 ('max_current', 0),
    		 ('inverter.efficiency', 0),
    		 ('des_time', 0),
    		 ('time_of_flight', 0),
    		 ('length_tube', 0),
    		 ('h', 0))

    prob.root.add('des_vars', IndepVarComp(params))
    prob.root.connect('des_vars.p_tube', 'TubeAndPod.p_tube')
    prob.root.connect('des_vars.M_pod', 'TubeAndPod.M_pod')
    prob.root.connect('des_vars.A_payload', 'TubeAndPod.A_payload')
    prob.root.connect('des_vars.w_track', 'TubeAndPod.w_track')
    prob.root.connect('des_vars.prc', 'TubeAndPod.prc')
    prob.root.connect('des_vars.n_passengers', 'TubeAndPod.n_passengers')
    prob.root.connect('des_vars.T_tunnel', 'TubeAndPod.T_tunnel')
    prob.root.connect('des_vars.p_tunnel', 'TubeAndPod.p_tunnel')
    prob.root.connect('des_vars.max_current', 'TubeAndPod.max_current')
    prob.root.connect('des_vars.inverter.efficiency', 'TubeAndPod.inverter.efficiency')
    prob.root.connect('des_vars.des_time', 'TubeAndPod.des_time')
    prob.root.connect('des_vars.time_of_flight', 'TubeAndPod.time_of_flight')
    prob.root.connect('des_vars.length_tube', 'TubeAndPod.length_tube')
    prob.root.connect('des_vars.h', 'TubeAndPod.h')

    prob.setup()
    prob.root.list_connections()
    prob.run()

    print('S: %f' % prob['TubeAndPod.S'])
