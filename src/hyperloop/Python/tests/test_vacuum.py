import numpy as np
from openmdao.api import Group, Problem

from hyperloop.Python.tube import tube_vacuum

def create_problem(vac):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', vac)
    return prob

class TestVac(object):
    def test_case1_vs_handcalc(self):

        vac = tube_vacuum.Vacuum()
        prob = create_problem(vac)

        prob.setup()

        prob['comp.pressure_initial'] = 760.2
        prob['comp.pressure_final'] = 7.0
        prob['comp.speed'] = 163333.3
        prob['comp.tube_radius'] = 5.0
        prob['comp.tube_length'] = 5000.0
        prob['comp.pwr'] = 18.5
        prob['comp.electricity_price'] = 0.13
        prob['comp.time_down'] = 300.0
        prob['comp.gamma'] = 0.8
        prob['comp.pump_weight'] = 715.0

        prob.run()

        assert np.isclose(prob['comp.number_pumps'], 2.13, rtol=0.01)
        assert np.isclose(prob['comp.cost_annual'], 35859.51, rtol=0.01)
        assert np.isclose(prob['comp.weight_tot'], 1521.25, rtol=0.01)

        # assert np.isclose(prob['comp.vol'], 190, rtol = 0.001)
        # assert np.isclose(prob['comp.etot'], 0, rtol = 0.001)
        # assert np.isclose(prob['comp.totpwr'], 101.686, rtol = 0.001)
