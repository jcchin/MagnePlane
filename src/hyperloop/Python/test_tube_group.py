"""
Test for tube_group.py.
"""
import pytest
from hyperloop.Python import tube_group
from math import pi
import numpy as np
from openmdao.api import Group, Problem


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

        # Pod Inputs
        prob['comp.

        prob.run()

        # Print Statement for debugging
        print('L is %12.12f' % prob['comp.Drag.L'])

        # Test Values
        assert np.isclose(prob['comp.Drag.LDratio'], 0.21618, rtol=.01)
        assert np.isclose(prob['comp.Drag.R'], 0.000707, rtol=.01)
        assert np.isclose(prob['comp.Drag.L'], 5.7619e-8, rtol=.01)
        assert np.isclose(prob['comp.Drag.B0'], 0.81281, rtol=.01)
