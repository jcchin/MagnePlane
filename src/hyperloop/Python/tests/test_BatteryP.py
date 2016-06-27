import pytest
from src.hyperloop.Python import BatteryP
import numpy as np
from openmdao.api import Group, Problem

def create_problem(batteryP):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', batteryP)
    return prob

class TestBatteryP(object):

    def test_case1(self):

        batteryP = BatteryP.BatteryP()

        prob = create_problem(batteryP)
        prob.setup()

        prob['comp.DesPower'] = 65000.
        prob['comp.FlightTime'] = 750.
        prob['comp.StackDesignVoltage'] = 300.

        prob.run()

        assert np.isclose(prob['comp.Voltage'], 303.000, rtol = 0.001)
        assert np.isclose(prob['comp.Current'], 200.000, rtol = 0.001)
        assert np.isclose(prob['comp.Nseries'],73.000 , rtol = 0.001)
        assert np.isclose(prob['comp.Nparallel'],2.000 , rtol = 0.001)
        assert np.isclose(prob['comp.Ncells'], 146.000, rtol = 0.001)
        assert np.isclose(prob['comp.C_max'], 3.37, rtol = 0.001)
