import pytest
from src.hyperloop.Python.mission import body_frame_acceleration
import numpy as np
from openmdao.api import Group, Problem


def create_problem(component):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', component)
    return prob

class TestMissionDrag(object):


    def test_case1_vs_npss(self):


        component  =  body_frame_acceleration.BodyFrameAcceleration()

        prob = create_problem(component)

        prob.setup()

        prob['comp.psi'] = 0.0
        prob['comp.theta'] = 0.0
        prob['comp.phi'] = 0.0
        prob['comp.omega'] = np.array([0.0, 0.0, 0.0])
        prob['comp.v'] = 0.0
        prob['comp.a_linear'] = 9.81

        prob.run()

        assert np.isclose(prob['comp.a_body'][0], 9.81, rtol = 0.01)
        assert np.isclose(prob['comp.a_body'][1], 0.0, rtol = 0.01)
        assert np.isclose(prob['comp.a_body'][2], 0.0, rtol = 0.01)