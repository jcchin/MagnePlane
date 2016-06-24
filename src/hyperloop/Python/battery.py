import math, numpy, scipy
from openmdao.core.component import Component
from openmdao.api import IndepVarComp, Component, Problem, Group, ScipyOptimizer, ExecComp, SqliteRecorder
from BatteryP import BatteryP
from BatteryWeight import BatteryWeight

class battery(Group):
	""" Group containing the battery MDA. This version uses the disciplines
    with derivatives."""
	def __init__(self):
		super(battery, self).__init__()

		# creates components of group
		self.add('batteryP', BatteryP())
		self.add('batteryWeight', BatteryWeight())

		self.connect('batteryP.Ncells', 'batteryWeight.Ncells')
		self.connect('batteryP.C_max', 'batteryWeight.C_max')

if __name__ == '__main__':
	from openmdao.api import IndepVarComp

	p = Problem()
	p.root = Group()
	p.root.add('battery', battery())

	p.setup()
	p.root.list_connections()
	p.run()

     #print following properties
	print ('StackWeight(kg) : %f' % p['battery.batteryWeight.StackWeight'])
	print ('StackVol(m^3) : %f' % p['battery.batteryP.StackVol'])
