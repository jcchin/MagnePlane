import pytest
from src.hyperloop.Python.pod.cycle import comp_len
import numpy as np
from openmdao.api import Group, Problem

def create_problem(component):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', component)
    return prob

class TestCompressorLength(object):

    def test_case1(self):

        component  =  comp_len.CompressorLen()
        prob = create_problem(component)
        prob.setup()

        prob['comp.comp_inletTemp'] = 293.0
        prob['comp.h_in'] = 0.0
        prob['comp.h_out'] = 207.
        prob['comp.comp_inletArea'] = 1.287
        prob['comp.A_inlet'] = 1.1
        prob['comp.comp_mach'] = 0.6
        prob['comp.M_pod'] = 0.8
        prob['comp.T_tunnel'] = 298.0
        prob['comp.p_tunnel'] = 850.0
        prob['comp.h_stage'] = 58.2
        prob.run()

        assert np.isclose(prob['comp.comp_len'], 0.518 , rtol = 0.1)
