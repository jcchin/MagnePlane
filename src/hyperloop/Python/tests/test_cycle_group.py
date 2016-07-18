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
    prob.root.add('comp', GroupName)
    return prob

class TestCycle(object):
    def test_case1_vs_inductrack(self):

        CycleGroup = cycle_group.Cycle()

        prob = create_problem(CycleGroup)
        prob.setup()

        params = (('vehicleMach', 0.8),
                  ('inlet_MN', 0.65),
                  ('P', 0.1885057735, {'units': 'psi'}),
                  ('T', 591.0961831, {'units': 'degR'}),
                  ('W', 4.53592, {'units': 'kg/s'}),
                  ('PsE', 0.59344451, {'units': 'psi'}),
                  ('cmpMach', 0.65), )

        prob.root.add('des_vars', IndepVarComp(params))
        prob.root.connect('des_vars.PsE', 'Cycle.FlowPath.nozzle.Ps_exhaust')
        prob.root.connect('des_vars.P', 'Cycle.FlowPath.fl_start.P')
        prob.root.connect('des_vars.T', 'Cycle.FlowPath.fl_start.T')
        prob.root.connect('des_vars.W', 'Cycle.FlowPath.fl_start.W')

        prob.root.connect('des_vars.vehicleMach', 'Cycle.FlowPath.fl_start.MN_target')
        prob.root.connect('des_vars.inlet_MN', 'Cycle.FlowPath.inlet.MN_target')

        prob.setup()
        prob.root.list_connections()

        # Inlet Conditions

        prob['Cycle.FlowPath.inlet.ram_recovery'] = 0.99
        if prob['des_vars.inlet_MN'] > prob['des_vars.vehicleMach']:
            prob['des_vars.inlet_MN'] = prob['des_vars.vehicleMach']

        # Compressor Conditions
        prob['Cycle.FlowPath.comp.map.PRdes'] = 6.0
        prob['Cycle.FlowPath.comp.map.effDes'] = 0.9
        prob['Cycle.FlowPath.comp.MN_target'] = 0.65

        # Duct
        prob['Cycle.FlowPath.duct.MN_target'] = 0.65
        prob['Cycle.FlowPath.duct.dPqP'] = 0.

        # Nozzle Conditions
        prob['Cycle.FlowPath.nozzle.Cfg'] = 1.0
        prob['Cycle.FlowPath.nozzle.dPqP'] = 0.

        # Shaft
        prob['Cycle.FlowPath.shaft.Nmech'] = 10000.

        prob.run()

        # Test Values
        assert np.isclose(prob['comp.Comp_Mass'], 583.259285, rtol=.01)
        assert np.isclose(prob['comp.Comp_len'], 0.609973, rtol=.01)
        assert np.isclose(prob['comp.FlowPath.inlet.Fl_O:tot:h'], 11.208383, rtol=.01)
        assert np.isclose(prob['comp.FlowPath.comp.Fl_O:tot:h'], 116.307430, rtol=.01)
        assert np.isclose(prob['comp.inlet.Fl_O:stat:area'], 1.794921, rtol=.01)
        assert np.isclose(prob['comp.inlet.Fl_O:tot:T'], 591.096183, rtol=.01)

