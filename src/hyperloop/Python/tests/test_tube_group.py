import pytest
from openmdao.api import Group, Problem

from hyperloop.Python.tube import tube_group
import numpy as np

def create_problem(GroupName):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', GroupName)
    return prob

class TestTube(object):
    def test_case1(self):

        TubeGroup = tube_group.TubeGroup()
        prob = create_problem(TubeGroup)
        prob.setup()
        # prob.root.list_connections()

        #Tube Inputs

        #prob['comp.temp_boundary'] = 322.0

        prob['comp.length_tube'] = 482803.0

        prob['comp.m_pod'] = 3100.0
        prob['comp.tube_area'] = 40.0
        prob['comp.h'] = 10.0

        prob['comp.Cd'] = 0.2
        prob['comp.S'] = 1.4
        prob['comp.mag_drag'] = 150.0
        prob['comp.nozzle_thrust'] = 3500.0
        prob['comp.ram_drag'] = 7237.6
        prob.run()



        # Print Statement for debugging
        """
        print('Vacuum.weight_tot:%f' % prob['comp.Vacuum.weight_tot'])
        print('Struct.vac_weight: %f' % prob['comp.Struct.vac_weight'])
        print('Vacuum.tot_pwr %f' % prob['comp.Vacuum.tot_pwr'])
        print('TubePower.vac_power: %f' % prob['comp.TubePower.vac_power'])
        print('PropMech.pwr_req: %f' % prob['comp.PropMech.pwr_req'])
        print('Total Tube Power: %f' % prob['comp.TubePower.tot_power'])
        """

        # Test Values
        assert np.isclose(prob['comp.TubePower.tot_power'],2379.206449, rtol=.01)
        assert np.isclose(prob['comp.temp_boundary'],322.00, rtol=.01)


        assert np.isclose(prob['comp.Vacuum.weight_tot'], 774.7674 , rtol=.01)
        assert np.isclose(prob['comp.Struct.vac_weight'], 774.7674 , rtol=.01)
        assert np.isclose(prob['comp.Vacuum.pwr_tot'], 20.0464, rtol=.01)
        assert np.isclose(prob['comp.TubePower.vac_power'], 20.0464, rtol=.01)
        assert np.isclose(prob['comp.PropMech.pwr_req'], 471832.00359, rtol=.01)
        assert np.isclose(prob['comp.TubePower.tot_power'], 471832.00359, rtol=.01)
