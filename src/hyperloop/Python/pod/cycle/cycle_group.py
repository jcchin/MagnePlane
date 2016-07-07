"""
Group for the Cycle containing the following components:
FlowPath and Compressor Mass
"""
from __future__ import print_function
from openmdao.api import IndepVarComp, Component, Problem, Group
import numpy as np
from os import remove

from openmdao.core.group import Group, Component, IndepVarComp
from openmdao.solvers.newton import Newton
from openmdao.api import NLGaussSeidel
from openmdao.solvers.scipy_gmres import ScipyGMRES
from openmdao.units.units import convert_units as cu
from openmdao.api import Problem, LinearGaussSeidel

from pycycle.components import Compressor, Shaft, FlowStart, Inlet, Nozzle, Duct, Splitter, FlightConditions
from pycycle.species_data import janaf
from pycycle.connect_flow import connect_flow
from pycycle.constants import AIR_FUEL_MIX, AIR_MIX
from pycycle.constants import R_UNIVERSAL_ENG, R_UNIVERSAL_SI

from openmdao.solvers.ln_gauss_seidel import LinearGaussSeidel
from openmdao.solvers.ln_direct import DirectSolver
from openmdao.api import SqliteRecorder

from hyperloop.Python.pod.cycle.flow_path import FlowPath
from hyperloop.Python.pod.cycle.compressor_mass import CompressorMass

class Cycle(Group):
	def __init__(self):
		super(Cycle, self).__init__()

		self.add('FlowPath', FlowPath(), promotes=['comp.trq', 'comp.power', 'comp.power', 'inlet.Fl_O:stat:area'])
		self.add('CompressorMass', CompressorMass(), promotes=['comp_mass', 'comp_inletArea'])

		self.connect('FlowPath.inlet.Fl_O:tot:h', 'CompressorMass.h_in')
		self.connect('FlowPath.comp.Fl_O:tot:h', 'CompressorMass.h_out')
		self.connect('inlet.Fl_O:stat:area', 'comp_inletArea')
		self.connect('FlowPath.inlet.Fl_O:stat:W', 'CompressorMass.mass_flow')

if __name__ == "__main__":
    prob = Problem()
    root = prob.root = Group()

    root.add('Cycle', Cycle())

    params = (('vehicleMach', 0.8),
              ('inlet_MN', 0.65),
              ('P', 0.1885057735, {'units': 'psi'}),
              ('T', 591.0961831, {'units': 'degR'}),
              ('W', 4.53592, {'units': 'kg/s'}),
              ('PsE', 0.59344451, {'units': 'psi'}))

    prob.root.add('des_vars', IndepVarComp(params))
    prob.root.connect('des_vars.PsE', 'Cycle.FlowPath.nozzle.Ps_exhaust')
    prob.root.connect('des_vars.P', 'Cycle.FlowPath.fl_start.P')
    prob.root.connect('des_vars.T', 'Cycle.FlowPath.fl_start.T')
    prob.root.connect('des_vars.W', 'Cycle.FlowPath.fl_start.W')

    prob.root.connect('des_vars.vehicleMach', 'Cycle.FlowPath.fl_start.MN_target')
    prob.root.connect('des_vars.inlet_MN', 'Cycle.FlowPath.inlet.MN_target')

    prob.setup()
    prob.root.list_connections()
    prob.run()

    print('Comp_Mass %f' % prob['Cycle.comp_mass'])
