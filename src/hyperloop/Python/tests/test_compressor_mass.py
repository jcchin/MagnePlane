import numpy as np
from openmdao.api import Group, Problem
from src.hyperloop.Python.pod.cycle import compressor_mass



def create_problem(compressor):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', compressor)
    return prob


class TestCompressorMass(object):
    def test_case1(self):
        compressor = compressor_mass.CompressorMass()
        prob = create_problem(compressor)
        prob.setup()


        prob['comp.comp_eff'] = 91.0
        prob['comp.mass_flow'] = 317.52
        prob['comp.h_in'] = 0.
        prob['comp.h_out'] = 486.13
        prob['comp.comp_inletArea'] = 1.287

        prob.run()



        assert np.isclose(prob['comp.comp_mass'], 1680.5, rtol=1.00)
