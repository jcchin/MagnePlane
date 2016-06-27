from openmdao.core.component import Component
from openmdao.api import IndepVarComp, Component, Problem, Group, ScipyOptimizer, ExecComp, SqliteRecorder
from BatteryP import BatteryP
from BatteryWeight import BatteryWeight
from BatteryPerf import Battery_perf

class battery(Group):
	""" Group containing the battery MDA. This version uses the disciplines
    with derivatives."""

	def __init__(self):
		super(battery, self).__init__()

		# creates components of group
		self.add('batteryP', BatteryP())
		self.add('batteryPerf', Battery_perf())
		self.add('batteryWeight', BatteryWeight())

		self.connect('batteryP.Ncells', ['batteryPerf.Ncells', 'batteryWeight.Ncells'])
		self.connect('batteryP.C_max', 'batteryWeight.C_max')
		self.connect('batteryP.Nparallel', 'batteryPerf.Nparallel')

if __name__ == '__main__':
	from openmdao.api import IndepVarComp, Newton, ScipyGMRES, NLGaussSeidel

	p = Problem()
	p.root = Group()

	p.root.add('p1', IndepVarComp('Ncells', 10.0))
	p.root.add('p2', IndepVarComp('C_max', 4.1))
	p.root.add('p3', IndepVarComp('Nparallel', 2.0))
	p.root.add('battery', battery())

	p.root.deriv_options['type'] = 'fd'
	p.root.nl_solver = Newton()
	p.root.ln_solver = ScipyGMRES()

	p.root.connect('p1.Ncells', 'battery.batteryP.NumCells')
	p.root.connect('p2.C_max', 'battery.batteryP.C_max_in')
	p.root.connect('p3.Nparallel', 'battery.batteryP.Nparallel')

	p.setup()
	p.root.list_connections()
	p.run()

    #print following properties
	print ('StackWeight(kg) : %f' % p['battery.batteryWeight.StackWeight'])
	print ('StackVol(m^3) : %f' % p['battery.batteryWeight.StackVol'])
	print ('Ncells : %f' % p['battery.batteryWeight.Ncells'])
	print ('Voltage Stack : %f' % p['battery.batteryP.Voltage'])
	print ('Voltage Current : %f' % p['battery.batteryP.Current'])
