from openmdao.core.component import Component
from openmdao.api import IndepVarComp, Component, Problem, Group, ScipyOptimizer, ExecComp, SqliteRecorder
from src.hyperloop.Python.BatteryP import BatteryP
from src.hyperloop.Python.BatteryWeight import BatteryWeight
from src.hyperloop.Python.BatteryPerf import Battery_perf

class battery(Group):
	"""
	Notes
	-----
		Battery group of components BatteryP, BatteryPerf, and BatteryWeight.

    Params
    ------
        DesPower: float
            Fully Charged Voltage in V. Default value is 2.0.
        FlightTime: float
            End of Nominal Zone Voltage in V. Default value is 2.0
        StackDesignVoltage: float
            Design stack voltage in V. Default value is 300.0

    Components
  	----------
  		batteryP : from BatteryP.py
  			Calculates parameters like Nparallel(number of cells in parallel), Nseries(Number of cells in series), Ncells(total no of cells) and C_Max(max rating)
       		The calculated parameters are then use to estimate battery weight in BatteryWeight.py
		batteryPerf : from Battery_perf.py
			Allows stand-alone batteries to be simulated without an electric circuit within NPSS for use in validating the procedure.
		batteryWeight : from BatteryWeight.py
			Allows sizing of battery based on design power load and necessary capacity

    Outputs
    -------
        StackWeight : float
            Total Capacity required for design in A*h. Default value is 1.0.
        StackVol : float
            Number of cells in parallel in the battery stack in cells. Default value is 1.0.
        Voltage: float
            Voltage lost due to polarization in ohms. Default value is 0.0.
        Ncells: float
             Number of cells necessary to perform that mission in cells. Default value is 2.0
        Current: float
            Charge at end of Exponential Curve in A*h. Default value is 2.0
    """
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
	print ('Current Stack: %f' % p['battery.batteryP.Current'])
	print ('Voltage Current : %f' % p['battery.batteryP.Current'])
