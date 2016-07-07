import numpy as np
from openmdao.api import Group, Problem

from hyperloop.Python.mission import mission_thrust

def create_problem(component):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', component)
    return prob

class TestMissionThrust(object):
    def test_case1_vs_npss(self):

        component = mission_thrust.MissionThrust()

        prob = create_problem(component)

        prob.setup()

        prob['comp.Cd'] = .2
        prob['comp.S'] = 1.4
        prob['comp.p_tube'] = 850.0
        prob['comp.T_ambient'] = 298.0
        prob['comp.R'] = 287.0
        prob['comp.D_magnetic'] = 150.0
        prob['comp.Thrust_pod'] = 3500.0
        prob['comp.V'] = 335.0
        prob['comp.theta'] = 0.0
        prob['comp.g'] = 9.81
        prob['comp.m_pod'] = 3100.0

        prob.run()

        assert np.isclose(prob['comp.Thrust'], 30717.148715, rtol=0.1)
