"""
Test for breakpointlev.py. Uses test values and outputs given by
the laminated sheet experiment in [1].
"""
from math import pi

import numpy as np
from openmdao.api import Group, Problem, IndepVarComp

from hyperloop.Python.pod.magnetic_levitation import levitation_group

class TestLev(object):
    def test_case1_vs_inductrack(self):

        root = Group()
        root.add('lev', levitation_group.LevGroup())
        prob = Problem(root)

        params = (('m_pod', 3000.0, {'units': 'kg'}),
              ('l_pod', 22.0, {'units': 'm'}),
              ('d_pod', 1.0, {'units': 'm'}),
              ('vel_b', 23.0, {'units': 'm/s'}),
              ('h_lev', 0.01, {'unit': 'm'}),
              ('vel', 350.0, {'units': 'm/s'}))

        prob.root.add('input_vars', IndepVarComp(params))

        prob.root.connect('input_vars.m_pod', 'lev.m_pod')
        prob.root.connect('input_vars.l_pod', 'lev.l_pod')
        prob.root.connect('input_vars.d_pod', 'lev.d_pod')
        prob.root.connect('input_vars.vel_b', 'lev.vel_b')
        prob.root.connect('input_vars.h_lev', 'lev.h_lev')
        prob.root.connect('input_vars.vel', 'lev.vel')

        prob.setup()

        prob['lev.Drag.b_res'] = 1.48
        prob['lev.Drag.num_mag_hal'] = 4.0
        prob['lev.Drag.gamma'] = 1.0
        prob['lev.Drag.w_mag'] = 3.0
        prob['lev.Drag.spacing'] = 0.0
        prob['lev.Drag.w_strip'] = .005
        prob['lev.Drag.num_sheets'] = 1.0
        prob['lev.Drag.delta_c'] = .0005334
        prob['lev.Drag.strip_c'] = .0105
        prob['lev.Drag.rc'] = 1.713e-8
        prob['lev.Drag.MU0'] = 4.0*np.pi*(1.0e-7)
        prob['lev.Drag.track_factor'] = .75
        prob['lev.Drag.g'] = 9.81 
        prob['lev.Drag.mag_thk'] = .15

        prob['lev.Mass.mag_thk'] = .15
        prob['lev.Mass.rho_mag'] = 7500.0
        prob['lev.Mass.gamma'] = 1.0
        prob['lev.Mass.cost_per_kg'] = 44.0
        prob['lev.Mass.w_mag'] = 3.0
        prob['lev.Mass.track_factor'] = .75

        prob.run()

        # Print Statements for debugging
        # print('Mag Mass %f' % prob['lev.m_mag'])
        # print('Mag Drag is %f' % prob['lev.mag_drag'])

        # Test Values

        assert np.isclose(prob['lev.mag_drag'], 9025.39, rtol=.01)
        assert np.isclose(prob['lev.total_pod_mass'], 21562.50, rtol=.01)
