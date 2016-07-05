import pytest
from src.hyperloop.Python.mission import lat_long
import numpy as np
from openmdao.api import Group, Problem


def create_problem(component):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', component)
    return prob

class TestMissionDrag(object):


    def test_case1_vs_npss(self):


        component  =  lat_long.LatLong()

        prob = create_problem(component)

        prob.setup()

        prob['comp.x'] = 100.0
        prob['comp.y'] = 100.0
        prob['comp.lat_origin'] = 35.0
        prob['comp.long_origin'] = -121.0
        prob['comp.R_E'] = 6378.0

        prob.run()

        assert np.isclose(prob['comp.lat'], 35.898335, rtol = 0.01)
        assert np.isclose(prob['comp.long'], -119.891025, rtol = 0.01)