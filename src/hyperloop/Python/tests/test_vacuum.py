import pytest
from hyperloop.Python import tube_vacuum
import numpy as np
from openmdao.api import Group, Problem


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

        prob['comp.pinit'] = 760.2
        prob['comp.pfinal'] = 7.0
        prob['comp.speed'] = 163333.3
        prob['comp.rad'] = 5.0
        prob['comp.len'] = 5000.0
        prob['comp.pwr'] = 18.5
        prob['comp.eprice'] = 0.13
        prob['comp.tdown'] = 300.0
        prob['comp.gamma'] = 0.8
        prob['comp.pumpweight'] = 715.0

        prob.run()

        assert np.isclose(prob['comp.n'], 2.13, rtol=0.01)
        assert np.isclose(prob['comp.cost'], 35859.51, rtol=0.01)
        assert np.isclose(prob['comp.weighttot'], 1521.25, rtol=0.01)

        # assert np.isclose(prob['comp.vol'], 190, rtol = 0.001)
        # assert np.isclose(prob['comp.etot'], 0, rtol = 0.001)
        # assert np.isclose(prob['comp.totpwr'], 101.686, rtol = 0.001)
