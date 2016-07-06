import numpy as np
from openmdao.api import Group, Problem

from Python.pod import propulsion_mechanics


def create_problem(component):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', component)
    return prob


class TestPropulsionMechanics(object):
    def test_case1_vs_npss(self):

        component = propulsion_mechanics.PropulsionMechanics()

        prob = create_problem(component)

        prob.setup()

        prob['comp.Cd'] = .2
        prob['comp.S'] = 1.4
        prob['comp.p_tube'] = 100.0
        prob['comp.T_ambient'] = 293.0
        prob['comp.R'] = 286.9
        prob['comp.D_magnetic'] = 150.0
        prob['comp.Thrust_pod'] = 3500.0
        prob['comp.vf'] = 335.0
        prob['comp.v0'] = 324.0
        prob['comp.rho_pm'] = 7400.0
        prob['comp.A'] = .0225
        prob['comp.t'] = .05
        prob['comp.g'] = 9.81
        prob['comp.m_pod'] = 3100.0
        prob['comp.eta'] = .8

        prob.run()

        assert np.isclose(prob['comp.pwr_req'], 373460.335731, rtol=0.1)
