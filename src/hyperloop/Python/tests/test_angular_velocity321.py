import pytest
from hyperloop.Python import angular_velocity321
import numpy as np
from openmdao.api import Group, Problem


def create_problem(component):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', component)
    return prob

class TestMissionDrag(object):


    def test_case1_vs_npss(self):


        component  =  angular_velocity321.AngularVelocity321()

        prob = create_problem(component)

        prob.setup()

        prob['comp.psi'] = 0.0
        prob['comp.theta'] = 0.0
        prob['comp.phi'] = 0.0
        prob['comp.psi_dot'] = 0.1
        prob['comp.theta_dot'] = 0.1
        prob['comp.phi_dot'] = 0.0

        prob.run()

        assert np.isclose(prob['comp.omega_b'][0], .1, rtol = 0.01)
        assert np.isclose(prob['comp.omega_b'][1], 0.1, rtol = 0.01)
        assert np.isclose(prob['comp.omega_b'][2], 0.0, rtol = 0.01)