"""
Test for breakpointlev.py. Uses test values and outputs given by
the laminated sheet experiment in [1].
"""
import pytest
from hyperloop.Python import levitation_group
from math import pi

import numpy as np
from openmdao.api import Group, Problem


def create_problem(GroupName):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', GroupName)
    return prob

class TestLev(object):
    """
    Params
    ------

    mpod : float
        Mass of the hyperloop pod. Test value is .375.
    Br : float
        Strength of the Neodynium Magnets. Test value is 1.21.
    M : float
        Number of Magnets per Halbach Array. Test value is 4.0.
    d : float
        Thickness of Magnet. Test value is 0.012.
    g : float
        Gravitational Acceleration. Test value is 9.81.
    lpod : float
        Length of the pod. Test value is .06.
    gamma : float
        Percent factor used in Area. Test value is 1.0.
    Pc : float
        Width of track. Test value is 0.11.
    w : float
        Width of magnet array. Test value is 0.6.
    spacing : float
        Halbach Spacing Factor. Test value is .007.
    Nt : float
        Width of conductive strip. Test value is .005.
    Ns : float
        Number of laminated sheets. Test value is 1.
    delta_c : float
        Single layer thickness. Test value is .005.
    strip_c : float
        Center strip spacing. Test value is .0105.
    rc : float
        Electric resistance. Test value is 1.713*10**-8.
    mu0 : float
        Permeability of Free Space. Test value is 4*pi*10^-7.
    vb : float
        Breakpoint velocity of the pod. Test value is 23.2038.
    y : float
        Levitation height. Test value is .01.

    Returns
    -------
    R : float
        Resistance of the track. Test value is 7.0e-3.
    L : float
        Inductance of the track. Test value is 5.7619e-8.
    B0 : float
        Halbach Peak Strength. Test value is 0.81281.
    LDratio : float
        Lift to drag ratio. Default value is 0.21618.

    Notes
    -----
    [1] Friend, Paul. Magnetic Levitation Train Technology 1. Thesis. Bradley University, 2004. N.p.: n.p., n.d. Print.
    """

    def test_case1_vs_inductrack(self):

        LevGroup = levitation_group.LevGroup()

        prob = create_problem(LevGroup)

        prob.setup()
        # prob.root.list_connections()

        # Pod Inputs
        prob['comp.Drag.mpod'] = .375
        prob['comp.Drag.Br'] = 1.21
        prob['comp.Drag.M'] = 4.0
        prob['comp.Drag.w'] = .06
        prob['comp.Drag.lpod'] = .06
        prob['comp.Drag.d'] = .012
        prob['comp.Drag.gamma'] = 1.0
        prob['comp.Drag.spacing'] = 0.007

        prob['comp.Mass.w'] = .06
        prob['comp.Mass.lpod'] = .06
        prob['comp.Mass.d'] = .012
        prob['comp.Mass.gamma'] = 1.0

        # Track Inputs
        prob['comp.Drag.Pc'] = .11
        prob['comp.Drag.Ns'] = 1.0
        prob['comp.Drag.Nt'] = .005
        prob['comp.Drag.delta_c'] = .0005334
        prob['comp.Drag.strip_c'] = .0105
        prob['comp.Drag.rc'] = 1.713*10**-8
        prob['comp.Drag.mu0'] = 4.*pi*10**-7

        # Pod/Track Relations
        prob['comp.Drag.y'] = .01
        prob['comp.Drag.vb'] = 23.2038

        prob.run()

        # Print Statement for debugging
        print('L is %12.12f' % prob['comp.Drag.L'])

        # Test Values
        assert np.isclose(prob['comp.Drag.LDratio'], 0.21618, rtol=.01)
        assert np.isclose(prob['comp.Drag.R'], 0.000707, rtol = .01)
        assert np.isclose(prob['comp.Drag.L'], 5.7619e-8, rtol = .01)
        assert np.isclose(prob['comp.Drag.B0'], 0.81281, rtol = .01)