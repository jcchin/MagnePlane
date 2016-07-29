from __future__ import print_function
import numpy as np
from os import remove

from openmdao.core.group import Group, Component, IndepVarComp
from openmdao.solvers.newton import Newton
from openmdao.api import NLGaussSeidel
from openmdao.solvers.scipy_gmres import ScipyGMRES
from openmdao.units.units import convert_units as cu
from openmdao.api import Problem, LinearGaussSeidel, ExecComp

from pycycle.components import Compressor, FlowStart
from pycycle.species_data import janaf
from pycycle.connect_flow import connect_flow
from pycycle.constants import AIR_FUEL_MIX, AIR_MIX
from pycycle.constants import R_UNIVERSAL_ENG, R_UNIVERSAL_SI

from openmdao.solvers.ln_gauss_seidel import LinearGaussSeidel
from openmdao.solvers.ln_direct import DirectSolver
from openmdao.api import SqliteRecorder

C_IN2toM2 = 144. * (3.28084**2.)
HPtoKW = 0.7457
tubeLen = 563270.0  # // 350 miles in meters
teslaPack = 90.0  # // kw-hours

class SteadyStateVacuum(Group):
    """
    Params
    ------
    fl_start.P : float
        Tube total pressure
    fl_start.T : float
        Tube total temperature
    fl_start.W : float
        Tube total mass flow
    fl_start.MN_target : float
        Vehicle mach number
    comp.map.PRdes : float
        Pressure ratio of compressor
    nozzle.Ps_exhaust : float
        Exit pressure of nozzle

    Returns
    -------
    comp.torque : float
        Total torque required by motor
    comp.power : float
        Total power required by motor
    comp.Fl_O:stat:area : float
        Area of the duct
    nozzle.Fg : float
        Nozzle thrust
    inlet.F_ram : float
        Ram drag
    nozzle.Fl_O:tot:T : float
        Total temperature at nozzle exit
    nozzle.Fl_O:stat:W : float
        Total mass flow rate at nozzle exit
    p.inlet.Fl_O:tot:h : float
        Inlet enthalpy of compressor
    p.comp.Fl_O:tot:h : float
        Exit enthalpy of compressor

    Notes
    -----
    [1] see https://github.com/jcchin/pycycle2/wiki
    """

    def __init__(self):
        super(SteadyStateVacuum, self).__init__()

        des_vars = (('ram_recovery', 0.99),
                    ('effDes', 0.9),
                    ('duct_MN', 0.0),
                    ('duct_dPqP', 0.),
                    ('nozzle_Cfg', 1.0),
                    ('nozzle_dPqP', 0.),
                    ('shaft_Nmech', 10000.),
                    ('inlet_MN', 0.0),
                    ('comp_MN', 0.0),
                    ('vehicle_mach', 0.0),
                    ('Pa', 101.3e3, {'units' : 'Pa'}))

        self.add('input_vars',IndepVarComp(des_vars))

        self.add('fl_start', FlowStart(thermo_data=janaf, elements=AIR_MIX))
        # internal flow
        self.add('comp', Compressor(thermo_data=janaf, elements=AIR_MIX))
        self.add('q', ExecComp('Prc = Pa/Ps'), promotes = ['Prc', 'Pa'])


        # connect components
        connect_flow(self, 'fl_start.Fl_O', 'comp.Fl_I')

        self.connect('input_vars.effDes', 'comp.map.effDes')
        self.connect('input_vars.comp_MN', 'comp.MN_target')
        self.connect('input_vars.shaft_Nmech', 'comp.Nmech')
        self.connect('input_vars.vehicle_mach', 'fl_start.MN_target')
        self.connect('input_vars.Pa', 'Pa')
        self.connect('Prc', 'comp.map.PRdes')
        self.connect('fl_start.P', 'q.Ps')

if __name__ == '__main__':
	prob = Problem()
	root = prob.root = Group()

	root.add('p', SteadyStateVacuum())

	# recorder = SqliteRecorder('pdb')
	# recorder.options['record_params'] = True
	# recorder.options['record_metadata'] = True
	# prob.driver.add_recorder(recorder)

	params = (('Pt', 850.0, {'units': 'Pa'}),
	          ('T', 320.0, {'units': 'K'}),
	          ('W', 1.0, {'units': 'kg/s'}))

	prob.root.add('des_vars', IndepVarComp(params))

	prob.root.connect('des_vars.Pt', 'p.fl_start.P')
	prob.root.connect('des_vars.T', 'p.fl_start.T')
	prob.root.connect('des_vars.W', 'p.fl_start.W')


	prob.setup()
	prob.run()

	print('\n')
	print("--- Freestream Static Conditions ---")
	print("Pod Mach No.:  %.6f " % (prob['p.fl_start.Fl_O:stat:MN']))
	print("Ambient Ps:    %.6f psi" % (prob['p.fl_start.Fl_O:stat:P']))
	print("Ambient Ts:    %.6f R" % (prob['p.fl_start.Fl_O:stat:T']))
	print("Ambient Pt:    %.6f psi" % (prob['p.fl_start.Fl_O:tot:P']))
	print("Ambient Tt:    %.6f R" % (prob['p.fl_start.Fl_O:tot:T']))

	print('\n')
	print("--- Compressor Exit Conditions ---")
	print("Compressor Ps:         %.6f psi" % (prob['p.comp.Fl_O:stat:P']))
	print("Compressor Ts:         %.6f degR" % (prob['p.comp.Fl_O:stat:T']))
	print("Compressor Pt:         %.6f psi" % (prob['p.comp.Fl_O:tot:P']))
	print("Compressor Tt:         %.6f degR" % (prob['p.comp.Fl_O:tot:T']))
	print("Compressor Power Reqd:  %.6f hp" % (prob['p.comp.power']))








