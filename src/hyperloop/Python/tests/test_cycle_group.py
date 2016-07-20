"""
Test for cycle_group.py. Uses test values and outputs given by NPSS
"""
from __future__ import print_function

import numpy as np
from openmdao.api import Group, Problem, IndepVarComp

from openmdao.core.group import Group, Component, IndepVarComp
from openmdao.solvers.newton import Newton
from openmdao.api import NLGaussSeidel
from openmdao.solvers.scipy_gmres import ScipyGMRES
from openmdao.units.units import convert_units as cu
from openmdao.api import Problem, LinearGaussSeidel
from openmdao.solvers.ln_direct import DirectSolver
from openmdao.api import SqliteRecorder

from pycycle.components import Compressor, Shaft, FlowStart, Inlet, Nozzle, Duct, Splitter, FlightConditions
from pycycle.species_data import janaf
from pycycle.connect_flow import connect_flow
from pycycle.constants import AIR_FUEL_MIX, AIR_MIX
from pycycle.constants import R_UNIVERSAL_ENG, R_UNIVERSAL_SI

from hyperloop.Python.pod.cycle import cycle_group

def create_problem(GroupName):
    root = Group()
    prob = Problem(root)
    prob.root.add('Cycle', GroupName)
    return prob

class TestCycle(object):
    def test_case1_vs_inductrack(self):

        CycleGroup = cycle_group.Cycle()

        prob = create_problem(CycleGroup)

        params = (('A_inlet_pod', 2.0869, {'units': 'm**2'}),
              ('comp_PR', 12.6, {'units': 'unitless'}),
              ('PsE', 0.05588, {'units': 'psi'}),
              ('pod_mach_number', .8, {'units': 'unitless'}),
              ('tube_pressure', 850., {'units': 'Pa'}),
              ('tube_temp', 320., {'units': 'K'}),
              ('comp_inlet_area', 2.3884, {'units': 'm**2'}))

        prob.root.add('des_vars', IndepVarComp(params))

        prob.root.connect('des_vars.A_inlet_pod', 'Cycle.A_inlet')
        prob.root.connect('des_vars.comp_PR', 'Cycle.FlowPath.comp.map.PRdes')
        prob.root.connect('des_vars.PsE', 'Cycle.FlowPath.nozzle.Ps_exhaust')
        prob.root.connect('des_vars.pod_mach_number', 'Cycle.pod_mach')
        prob.root.connect('des_vars.tube_pressure', 'Cycle.tube_pressure')
        prob.root.connect('des_vars.tube_temp', 'Cycle.tube_temp')
        prob.root.connect('des_vars.comp_inlet_area', 'Cycle.comp_inlet_area')

        prob.setup()
        prob.root.list_connections()

        prob.run()

        # Test Values
        assert np.isclose(prob['Cycle.comp_len'], 0.775, rtol=.01)
        assert np.isclose(prob['Cycle.comp_mass'], 774.18, rtol=.01)
        assert np.isclose(cu(prob['Cycle.comp.trq'], 'ft*lbf', 'N*m'), -2622.13, rtol=.01)
        assert np.isclose(cu(prob['Cycle.comp.power'], 'hp', 'W'), -2745896.44, rtol=.01)
        assert np.isclose(cu(prob['Cycle.comp.Fl_O:stat:area'], 'inch**2', 'm**2'), 0.371, rtol=.01)
        assert np.isclose(cu(prob['Cycle.nozzle.Fg'], 'lbf', 'N'), 6562.36, rtol=.01)
        assert np.isclose(cu(prob['Cycle.inlet.F_ram'], 'lbf', 'N'), 1855.47, rtol=.01)
        assert np.isclose(cu(prob['Cycle.nozzle.Fl_O:tot:T'], 'degR', 'K'), 767.132, rtol=.01)
        assert np.isclose(cu(prob['Cycle.nozzle.Fl_O:stat:W'], 'lbm/s', 'kg/s'), 6.467, rtol=.01)
