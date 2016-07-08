"""
Test for tube_group.py.

import pytest
from hyperloop.Python.tube import tube_group
import numpy as np
from openmdao.api import Group, Problem

def create_problem(GroupName):
    root = Group()
    prob = Problem(root)
    prob.root.add('group', GroupName)
    return prob

class TestTube(object):
    def test_case1(self):

        TubeGroup = tube_group.TubeGroup()

        prob = create_problem(TubeGroup)

        prob.setup()
        # prob.root.list_connections()

        # Pod Inputs
        #prob['group.CompressionCycle.'] =

        prob['group.PodMach.'] =

        prob['group.Geometry.'] =

        prob['group.LevGroup.'] =

        prob['group.Weight.'] =


        prob.run()

        # Print Statement for debugging

        print('Vacuum: %f' %prob['group.Vacuum.'])
        print('Tube temperature: %f' % prob['group.Thermal.'])
        print('Structural: %f' %prob['group.Struct.'])
        print('Propulsion Mechanics: %f' %prob['group.PropMech.'])
        print('Total Tube Power required: %f' % prob['group.TubePower.'])

        # Test Values
        assert np.isclose(prob['group.Vacuum.'], , rtol=.01)
        assert np.isclose(prob['group.Thermal.'], , rtol=.01)
        assert np.isclose(prob['group.Struct.'], , rtol=.01)
        assert np.isclose(prob['group.PropMech.'], , rtol=.01)
        assert np.isclose(prob['group.TubePower.'], , rtol=.01)
"""