"""
Test for tube_group.py.
"""
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
        #Vacuum
        prob['comp.tube_radius'] = 5.0


        #Temp Balance
        prob['comp.temp_boundary'] = 322.0

        #Tube Wall Temp
        prob['comp.radius_outer_tube'] = 1.115
        prob['comp.length_tube'] = 482803.0


        #Tube and Pylon
        prob['comp.m_pod'] = 3100.0
        prob['comp.r'] = 1.1
        prob['comp.h'] = 10.0

        #Propulsion Mechanics
        prob['comp.Cd'] = .2
        prob['comp.S'] = 1.4
        prob['comp.D_magnetic'] = 150.0
        prob['comp.Thrust_pod'] = 3500.0
        prob['comp.A'] = .0225
        prob['comp.t'] = .05

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
        assert np.isclose(prob['comp.Vacuum.weight_tot'], 1521.2524 , rtol=.01)
        assert np.isclose(prob['comp.Struct.vac_weight'], 1521.2524 , rtol=.01)
        assert np.isclose(prob['comp.Vacuum.tot_pwr'], 1.000, rtol=.01)
        assert np.isclose(prob['comp.TubePower.vac_power'], 1000.000, rtol=.01)
        assert np.isclose(prob['comp.PropMech.pwr_req'], 373437.942, rtol=.01)
        assert np.isclose(prob['comp.TubePower.tot_power'], 374437.941, rtol=.01)
