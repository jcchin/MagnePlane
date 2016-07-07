import numpy as np
from openmdao.api import Group, Problem
from src.hyperloop.Python.pod import pod_mass

def create_problem(component):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', component)
    return prob

class Test1(object):
    def test_case1(self):


        component = pod_mass.PodMass()

        prob = create_problem(component)

        prob.setup()

        prob['comp.mag_mass'] = 1.0
        prob['comp.podgeo_r'] = 1.0
        prob['comp.al_rho'] = 2800.0
        prob['comp.motor_mass'] = 1.0
        prob['comp.battery_mass'] = 1.0
        prob['comp.comp_mass'] = 1.0
        prob['comp.pod_len'] = 1.0
        prob.run()

        assert np.isclose(prob['comp.pod_mass'], 10479.95, rtol = 1.)



