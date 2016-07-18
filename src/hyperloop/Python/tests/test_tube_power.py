from openmdao.api import Group, Problem, Component
import numpy as np

from hyperloop.Python.tube import tube_power

def create_problem(component):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp',component)
    return prob

class TestTubePower(object):
    #Test for TubePower component
    def test_case1(self):
        comp = tube_power.TubePower()
        prob = create_problem(comp)

        prob.setup()

        prob['comp.vac_power'] = 40.0
        prob['comp.vac_energy_day'] = 40.0*24.0*60.0*60.0
        prob['comp.prop_power'] = 300000.0
        prob['comp.num_thrust'] = 5.0
        prob['comp.time_thrust'] = 1.5
        prob['comp.tube_temp'] = 320.0
        prob['comp.elec_price'] = 0.13


        prob.run()

        assert np.isclose(prob['comp.tot_power'], 1540.00, rtol=0.1)
        assert np.isclose(prob['comp.tot_energy'], 144000.625, rtol=0.1)
        assert np.isclose(prob['comp.cost_pwr'],200.200, rtol=.01)

 