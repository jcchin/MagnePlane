import pytest
from hyperloop.Python import compressor_weight
import numpy as np
from openmdao.api import Group, Problem



def create_problem(compressor):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', compressor)
    return prob

class TestCompressor(object):
    def test_case1(self):
        compressor = compressor_weight.Compressor_weight()

        prob = create_problem(compressor)

        prob.setup()

        prob['comp.comp_eff'] = 0.7
        prob['comp.mass_flow'] = 1.5
        prob['comp.h_in'] = 1.5
        prob['comp.h_out'] = 1.5

        prob.run()
        assert np.isclose(prob['comp.comp_weight'], 1417.00, rtol=1.00)
