import pytest
from hyperloop.Python import battery
import numpy as np
from openmdao.api import Group, Problem, Newton, ScipyGMRES, NLGaussSeidel

def create_problem(batt):
    root = Group()
    prob = Problem(root)
    prob.root.add('Group', batt)
    prob.root.deriv_options['type'] = 'fd'
    prob.root.nl_solver = Newton()
    prob.root.ln_solver = ScipyGMRES()
    return prob

class TestBattery(object):

    def test_case1(self):

        batt = battery.battery()

        prob = create_problem(batt)
        prob.setup()

        prob['Group.batteryP.DesPower'] = 65000.0
        prob['Group.batteryP.FlightTime'] = 750.0
        prob['Group.batteryP.StackDesignVoltage'] = 300.0

        prob.run()

        assert np.isclose(prob['Group.batteryWeight.StackWeight'], 168.600, rtol=0.001)
        assert np.isclose(prob['Group.batteryWeight.StackVol'], 32247.1375, rtol=0.001)