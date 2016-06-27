import pytest
from hyperloop.Python import BatteryWeight
import numpy as np
from openmdao.api import Group, Problem, Newton, ScipyGMRES, NLGaussSeidel

def create_problem(batteryWeight):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', batteryWeight)
    prob.root.deriv_options['type'] = 'fd'
    prob.root.nl_solver = Newton()
    prob.root.ln_solver = ScipyGMRES()
    return prob

class TestBatteryWeight(object):

    def test_case1(self):

        batteryWeight = BatteryWeight.BatteryWeight()

        prob = create_problem(batteryWeight)
        prob.setup()

        prob['comp.C_max'] = 3.37037
        prob['comp.Ncells'] = 146.0

        prob.run()

        assert np.isclose(prob['comp.StackWeight'], 168.600, rtol=0.001)
        assert np.isclose(prob['comp.StackVol'], 32247.1375, rtol=0.001)
