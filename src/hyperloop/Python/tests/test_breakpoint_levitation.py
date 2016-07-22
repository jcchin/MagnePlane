"""
Test for breakpointlev.py. Uses test values and outputs given by
the laminated sheet experiment in [1].
"""
from math import pi
import numpy as np
from openmdao.api import Group, Problem

from hyperloop.Python.pod.magnetic_levitation.breakpoint_levitation import BreakPointDrag
from hyperloop.Python.pod.magnetic_levitation.breakpoint_levitation import MagMass

def create_problem(GroupName):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', GroupName)
    return prob

class TestLev(object):
    def test_case1_vs_inductrack(self):

        LevGroup = levitation_group.LevGroup()
        prob = create_problem(LevGroup)
        prob.setup()
        # prob.root.list_connections()

        # Pod Inputs
        prob['comp.m_pod'] = .375
        prob['comp.l_pod'] = .06

        prob['comp.Drag.b_res'] = 1.21
        prob['comp.Drag.num_mag_hal'] = 4.0
        prob['comp.Drag.mag_thk'] = .012
        prob['comp.Drag.gamma'] = 1.0
        prob['comp.Drag.spacing'] = 0.007

        prob['comp.Mass.mag_thk'] = .012
        prob['comp.Mass.gamma'] = 1.0

        # Track Inputs
        prob['comp.d_pod'] = .11
        prob['comp.Drag.track_factor'] = 1.0
        prob['comp.Drag.num_sheets'] = 1.0
        prob['comp.Drag.w_strip'] = .005
        prob['comp.Drag.delta_c'] = .0005334
        prob['comp.Drag.strip_c'] = .0105
        prob['comp.Drag.rc'] = 1.713 * 10** -8
        prob['comp.Drag.MU0'] = 4. * pi * 10** -7

        # Pod/Track Relations
        prob['comp.Drag.h_lev'] = .01
        prob['comp.vel_b'] = 23.2038

        prob.run()

        # Print Statement for debugging
        # print('track_ind is %12.12f' % prob['comp.Drag.track_ind'])

        # Test Values
        assert np.isclose(prob['comp.Drag.ld_ratio'], 0.21618, rtol=.01)
        assert np.isclose(prob['comp.Drag.track_res'], 0.000707, rtol=.01)
        assert np.isclose(prob['comp.Drag.track_ind'], 5.7619e-8, rtol=.01)
        assert np.isclose(prob['comp.Drag.b0'], 0.81281, rtol=.01)
        assert np.isclose(prob['comp.total_pod_mass'], 3000.375, rtol=0.01)
