import pytest
from src.hyperloop.Python import boundary_layer_sensitivity
import numpy as np
from openmdao.api import Group, Problem


def create_problem(component):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', component)
    return prob

class TestBoundaryLayerSensitivity(object):


    def test_case1_vs_npss(self):


        component  =  boundary_layer_sensitivity.BoundaryLayerSensitivity

        prob = create_problem(component)

        prob.setup()

        prob['comp.gam'] = 1.4
        prob['comp.R'] = 287.0
        prob['comp.BF'] = .9
        prob['comp.A_pod'] = 2.0
        prob['comp.L'] = 22.0
        prob['comp.prc'] = 12.5
        prob['comp.p_tube'] = 850.0
        prob['comp.T_ambient'] = 298.0
        prob['comp.mu'] = 1.846e-5
        prob['comp.M_duct'] = .95
        prob['comp.M_diff'] = .6
        prob['comp.cp'] = 1009.0
        prob['comp.delta_star'] = .02
        prob['comp.M_pod'] = .8
        prob['comp.length_calc'] = False

        prob.run()

        assert np.isclose(prob['comp.A_tube'], 16.7848131256, rtol=0.1)