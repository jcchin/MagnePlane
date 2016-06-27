import pytest
from src.hyperloop.Python import mission_drag
import numpy as np
from openmdao.api import Group, Problem


def create_problem(component):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', component)
    return prob

class TestMissionDrag(object):


    def test_case1_vs_npss(self):


        component  =  mission_drag.MissionDrag()

        prob = create_problem(component)

        prob.setup()

        prob['comp.Cd'] = .2
        prob['comp.S'] = 1.4
        prob['comp.p_tube'] = 850.0
        prob['comp.T_ambient'] = 298.0
        prob['comp.R'] = 287.0
        prob['comp.D_magnetic'] = 150.0
        prob['comp.V'] = 335.0

        prob.run()

        assert np.isclose(prob['comp.D'], 930.743575, rtol = 0.01)

