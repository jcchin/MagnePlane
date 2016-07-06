"""
Group for the Cycle containing the following components:
Flowpath and Compressor Mass
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

from hyperloop.Python.pod.cycle.flowpath import FlowPath
from hyperloop.Python.pod.cycle.compressor_mass import CompressorMass

class Cycle(Group):
	def __init__(self):
		super(Cycle, self).__init__()

		self.add('Flowpath', FlowPath())
		self.add('CompressorMass', CompressorMass())

		self.connect('Flowpath.inlet.Fl_O:tot:h', 'CompressorMass.h_in')
		self.connect('Flowpath.comp.Fl_O:tot:h', 'CompressorMass.h_out')
		self.connect('Flowpath.inlet.Fl_O:stat:area', 'CompressorMass.comp_inletArea')
		self.connect('Flowpath.inlet.Fl_O:stat:W', 'CompressorMass.mass_flow')

if __name__ == "__main__":
    prob = Problem()
    root = prob.root = Group()

    root.add('Cycle', Cycle())

    recorder = SqliteRecorder('FlowPathdb')
    recorder.options['record_params'] = True
    recorder.options['record_metadata'] = True
    prob.driver.add_recorder(recorder)

    params = (('vehicleMach', 0.8),
              ('inlet_MN', 0.65),
              ('P', 0.1885057735, {'units': 'psi'}),
              ('T', 591.0961831, {'units': 'degR'}),
              ('W', 4.53592, {'units': 'kg/s'}),
              ('PsE', 0.59344451, {'units': 'psi'}),
              ('cmpMach', 0.65), )

    prob.root.add('des_vars', IndepVarComp(params))
    prob.root.connect('des_vars.PsE', 'Cycle.FlowPath.nozzle.Ps_exhaust')

    prob.setup(check=True)
    prob.root.list_connections()
    prob.run()

    print("%f", prob['cycle.Flowpath.inlet.Fl_O:tot:h'])