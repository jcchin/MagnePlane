"""
Test for breakpointlev.py. Uses test values and outputs given by
the laminated sheet experiment in [1].
"""
from math import pi

import numpy as np
from openmdao.api import Group, Problem

from hyperloop.Python.pod.magnetic_levitation import levitation_group

def create_problem(GroupName):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', GroupName)
    return prob

class TestLev(object):
    """
    Params
    ------

    m_pod : float
        Mass of the hyperloop pod. Test value is .375.
    b_res : float
        Strength of the Neodynium Magnets. Test value is 1.21.
    num_mag_hal : float
        Number of Magnets per Halbach Array. Test value is 4.0.
    mag_thk : float
        Thickness of Magnet. Test value is 0.012.
    g : float
        Gravitational Acceleration. Test value is 9.81.
    l_pod : float
        Length of the pod. Test value is .06.
    gamma : float
        Percent factor used in Area. Test value is 1.0.
    d_pod : float
        Diameter of pod. Test value is 0.11.
    w_mag : float
        Width of magnet array. Test value is 0.6.
    spacing : float
        Halbach Spacing Factor. Test value is .007.
    w_strip : float
        Width of conductive strip. Test value is .005.
    num_sheets : float
        Number of laminated sheets. Test value is 1.
    delta_c : float
        Single layer thickness. Test value is .005.
    strip_c : float
        Center strip spacing. Test value is .0105.
    rc : float
        Electric resistance. Test value is 1.713*10**-8.
    MU0 : float
        Permeability of Free Space. Test value is 4*pi*10^-7.
    vel_b : float
        Breakpoint velocity of the pod. Test value is 23.2038.
    h_lev : float
        Levitation height. Test value is .01.
    track_factor : float
        Factor to adjust track width. Test value is 1.

    Returns
    -------
    track_res : float
        Resistance of the track. Test value is 7.0e-3.
    track_ind : float
        Inductance of the track. Test value is 5.7619e-8.
    b0 : float
        Halbach Peak Strength. Test value is 0.81281.
    ld_ratio : float
        Lift to drag ratio. Default value is 0.21618.

    Notes
    -----
    [1] Friend, Paul. Magnetic Levitation Train Technology 1. Thesis.
    Bradley University, 2004. N.p.: n.p., n.d. Print.
    """

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
        prob['comp.Drag.w_mag'] = .06
        prob['comp.Drag.mag_thk'] = .012
        prob['comp.Drag.gamma'] = 1.0
        prob['comp.Drag.spacing'] = 0.007

        prob['comp.Mass.mag_thk'] = .012
        prob['comp.Mass.gamma'] = 1.0

        # Track Inputs
        prob['comp.d_pod'] = .11
        prob['comp.Drag.track_factor'] = 1
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
