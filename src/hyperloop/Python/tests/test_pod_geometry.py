import numpy as np
from openmdao.api import Group, Problem

from hyperloop.Python.pod import pod_geometry

def create_problem(component):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', component)
    return prob

class TestMissionDrag(object):


    def test_case1_vs_npss(self):


        component  =  pod_geometry.PodGeometry()

        prob = create_problem(component)

        prob.setup()

        prob['comp.L_comp'] = 1.0
        prob['comp.L_motor'] = 1.0
        prob['comp.L_bat'] = 1.0
        prob['comp.L_inverter'] = 1.0
        prob['comp.L_trans'] = 1.0
        prob['comp.L_p'] = 11.2
        prob['comp.L_conv'] = .3
        prob['comp.L_div'] = 1.5
        prob['comp.D_dif'] = 1.28
        prob['comp.BF'] = .9
        prob['comp.prc'] = 12.5
        prob['comp.A_inlet'] = 1.1
        prob['comp.gam'] = 1.4
        prob['comp.p_tunnel'] = 850.0
        prob['comp.R'] = 287.0
        prob['comp.T_tunnel'] = 298.0
        prob['comp.beta'] = .1
        prob['comp.M_dif'] = .6
        prob['comp.M_duct'] = .3
        prob['comp.A_payload'] = 1.4
        prob['comp.M_pod'] = .8


        prob.run()

        assert np.isclose(prob['comp.A_pod'], 2.032157, rtol = 0.01)
        assert np.isclose(prob['comp.D_pod'], 1.608547, rtol = 0.01)
        assert np.isclose(prob['comp.L_pod'], 18.921240, rtol = 0.01)
        assert np.isclose(prob['comp.S'], 30.435701, rtol = 0.01)

