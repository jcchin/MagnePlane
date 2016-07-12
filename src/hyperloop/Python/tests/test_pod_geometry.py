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
        prob['comp.L_inlet'] = 2.5
        prob['comp.p_tunnel'] = 850.0
        prob['comp.p_duct'] = 6800.0
        prob['comp.p_passenger'] = 101.0e3
        prob['comp.rho_pod'] = 2700.0
        prob['comp.n_passengers'] = 28.0
        prob['comp.dm_passenger'] = 166.0
        prob['comp.SF'] = 1.5
        prob['comp.Su'] = 50.0e6
        prob['comp.A_duct'] = .3
        prob['comp.dl_passenger'] = .8
        prob['comp.g'] = 9.81

        prob.run()

        assert np.isclose(prob['comp.A_pod'], 3.053648, rtol = 0.01)
        assert np.isclose(prob['comp.D_pod'], 1.971808, rtol = 0.01)
        assert np.isclose(prob['comp.L_pod'], 20.500000, rtol = 0.01)
        assert np.isclose(prob['comp.S'], 40.422060, rtol = 0.01)
        assert np.isclose(prob['comp.t_passenger'], 0.002630, rtol = 0.01)
        assert np.isclose(prob['comp.t_pod'], 0.002946, rtol = 0.01)
        assert np.isclose(prob['comp.BF'], 0.994033, rtol = 0.01)
        assert np.isclose(prob['comp.beta'], 0.005672, rtol = 0.01)

