from __future__ import print_function

import numpy as np
from openmdao.api import Group, Problem

from Python.pod.drivetrain import inverter


def create_problem(inverter):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', inverter)
    return prob


class TestInverter(object):
    def test_case1_vs_npss(self):

        prob = create_problem(inverter.Inverter())
        prob.setup()

        prob['comp.efficiency'] = 0.704995
        prob['comp.output_voltage'] = 10.1528
        prob['comp.output_current'] = 11.0613
        prob['comp.input_voltage'] = 24
        prob['comp.output_frequency'] = 200

        prob.run()

        assert np.isclose(prob['comp.input_power'], 390.1949, rtol=0.001)
        assert np.isclose(prob['comp.input_current'],
                          16.2581209067,
                          rtol=0.001)

    def test_case2_vs_npss(self):
        prob = create_problem(inverter.Inverter())
        prob.setup()

        prob['comp.efficiency'] = 0.85

        prob['comp.output_voltage'] = 15
        prob['comp.output_current'] = 30
        prob['comp.input_voltage'] = 60

        prob['comp.output_frequency'] = 200

        prob.run()

        assert np.isclose(prob['comp.input_power'], 1296.78, rtol=0.001)
        assert np.isclose(prob['comp.input_current'], 21.6131, rtol=0.001)
