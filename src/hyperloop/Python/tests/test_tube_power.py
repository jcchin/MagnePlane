from openmdao.api import Group, Problem, Component
import numpy as np

from hyperloop.Python.tube import tube_power

def create_problem(component):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp',component)
    return prob

class TestTubePower(object):
    def test_case1(self):
        comp = tube_power.TubePower()
        prob = create_problem(comp)

        prob.setup()

        prob['comp.vac_power'] = 200000.0
        prob['comp.prop_power'] = 250000.0
        prob['comp.tube_temp'] = 320.0

        prob.run()

        assert np.isclose(prob['comp.tot_power'], 450000.0, rtol=0.1)

 