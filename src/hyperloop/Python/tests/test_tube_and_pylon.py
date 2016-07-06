import numpy as np
from openmdao.api import Group, Problem

from Python.tube import tube_and_pylon


def create_problem(component):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', component)
    return prob


class TestTubeAndPylon(object):
    def test_case1_vs_npss(self):

        component = tube_and_pylon.TubeAndPylon()

        prob = create_problem(component)

        prob.setup()

        prob['comp.rho_tube'] = 7820.0
        prob['comp.E_tube'] = 200.0e9
        prob['comp.v_tube'] = .3
        prob['comp.Su_tube'] = 152.0e6
        prob['comp.sf'] = 1.5
        prob['comp.g'] = 9.81
        prob['comp.unit_cost_tube'] = .3307
        prob['comp.p_tunnel'] = 100.0
        prob['comp.p_ambient'] = 101300.0
        prob['comp.alpha_tube'] = 0.0
        prob['comp.dT_tube'] = 0.0
        prob['comp.m_pod'] = 3100.0
        prob['comp.r'] = 1.1
        prob['comp.t'] = 5.0
        prob['comp.rho_pylon'] = 2400.0
        prob['comp.E_pylon'] = 41.0e9
        prob['comp.v_pylon'] = .2
        prob['comp.Su_pylon'] = 40.0e6
        prob['comp.sf'] = 1.5
        prob['comp.unit_cost_pylon'] = .05
        prob['comp.h'] = 10.0
        prob['comp.r_pylon'] = 1.1

        prob.run()

        assert np.isclose(prob['comp.m_prime'], 884421.16, rtol=0.1)
        assert np.isclose(prob['comp.R'], 101368720.0, rtol=0.1)
        assert np.isclose(prob['comp.dx'], 23.36, rtol=0.1)
