import pytest
from hyperloop.Python import BatteryPerf
import numpy as np
from openmdao.api import Group, Problem

def create_problem(batteryPerf):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', batteryPerf)
    return prob

class TestBatteryPerf(object):

    def test_case1(self):

        batteryPerf = BatteryPerf.Battery_perf()

        prob = create_problem(batteryPerf)
        prob.setup()

        prob['comp.Nparallel'] = 2.0
        prob['comp.Ncells'] = 146.0

        prob.run()
        
        assert np.isclose(prob['comp.Voltage'], 320.259, rtol=0.001)
        assert np.isclose(prob['comp.Current'], 200.00, rtol=0.001)