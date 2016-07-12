import numpy as np
from openmdao.api import Group, Problem

from hyperloop.Python.pod.cycle.compressor_mass import CompressorMass
from hyperloop.Python.pod.pod_mass import PodMass

def create_problem(compressor, pod):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp2', compressor)
    prob.root.add('comp1', pod)
    #prob.root.connect('comp1.comp_mass', 'comp2.comp_mass')
    return prob

class TestCompressorMass(object):
    def test_case1(self):
        compressor = CompressorMass()
        pod = PodMass()
        prob = create_problem(pod, compressor)

        prob.setup()

        prob['comp1.comp_eff'] = 91.0
        prob['comp1.mass_flow'] = 317.52
        prob['comp1.h_in'] = 0.
        prob['comp1.h_out'] = 486.13
        #prob.root.connect('comp1.comp_mass', 'comp2.comp_mass')
        prob['comp2.mag_mass'] = 1.0
        prob['comp2.motor_mass'] = 1.0
        prob['comp2.battery_mass'] = 1.0

        prob.run()

        assert np.isclose(prob['comp1.comp_mass'], 1417.00, rtol=1.00)
        assert np.isclose(prob['comp2.pod_mass'], 1421.00, rtol=1.00)
        #assert np.isclose(prob['comp2.pod_mass'], 1421.00, rtol=1.00)

