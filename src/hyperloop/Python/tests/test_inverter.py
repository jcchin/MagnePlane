from __future__ import print_function
import pytest
from hyperloop.Python import inverter
import numpy as np
from openmdao.api import Group, Problem

def create_problem(inverter):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', inverter)
    return prob

class TestInverter(object):

    def test_case1_vs_npss(self):

        prob = create_problem(inverter.Inverter())
        prob.setup()

        prob['comp.Efficiency'] = 0.704995
        prob['comp.DesignPower'] = 8000
        prob['comp.OutputVoltage'] = 10.1528
        prob['comp.OutputCurrent'] = 11.0613
        prob['comp.InputVoltage'] = 24
        prob['comp.OutputFrequency'] = 200

        prob.run()

        assert np.isclose(prob['comp.OutputPower'], 275.085454767, rtol = 0.001)
        assert np.isclose(prob['comp.InputCurrent'], -16.2581209067, rtol = 0.001)


    def test_case2_vs_npss(self):
        prob = create_problem(inverter.Inverter())
        prob.setup()

        prob['comp.Efficiency'] = 0.85
        prob['comp.DesignPower'] = 8000

        prob['comp.OutputVoltage'] = 15
        prob['comp.OutputCurrent'] = 30
        prob['comp.InputVoltage'] = 60

        prob['comp.OutputFrequency'] = 200

        prob.run()

        assert np.isclose(prob['comp.OutputPower'], 1102.27, rtol=0.001)
        assert np.isclose(prob['comp.InputCurrent'], -21.6131, rtol=0.001)