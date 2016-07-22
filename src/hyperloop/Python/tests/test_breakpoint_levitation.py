import numpy as np
from openmdao.api import Group, Problem

from hyperloop.Python.pod.magnetic_levitation import breakpoint_levitation

class BreakPointLevTest(object):
    def test_case1_vs_npss(self):

        root = Group()
        root.add('p', breakpoint_levitation.BreakPointDrag())
        root.add('q', breakpoint_levitation.MagMass())
        prob = Problem(root)
        prob.setup()

        prob['p.m_pod'] = 3000.0
        prob['p.b_res'] = 1.48 
        prob['p.num_mag_hal'] = 4.0
        prob['p.mag_thk'] = .15
        prob['p.l_pod'] = 22.0
        prob['p.gamma'] = 1.0
        prob['p.w_mag'] = 3.0
        prob['p.spacing'] = 0.0
        prob['p.d_pod'] = 1.0
        prob['p.w_strip'] = .005
        prob['p.num_sheets'] = 1.0
        prob['p.delta_c'] = .0005334
        prob['p.strip_c'] = .0105
        prob['p.rc'] = 1.713e-8
        prob['p.MU0'] = 4.0*np.pi*(1.0e-7)
        prob['p.track_factor'] = .75
        prob['p.vel_b'] = 23.0
        prob['p.h_lev'] = .01
        prob['p.g'] = 9.81 

        prob['q.m_pod'] = 3000.0
        prob['q.mag_thk'] = .15
        prob['q.rho_mag'] = 7500.0
        prob['q.l_pod'] = 22.0
        prob['q.gamma'] = 1.0
        prob['q.cost_per_kg'] = 44.0
        prob['q.w_mag'] = 3.0
        prob['q.d_pod'] = 1.0
        prob['q.track_factor'] = .75

        prob.run()

        # Print Statement for debugging
        # print('track_ind is %12.12f' % prob['comp.Drag.track_ind'])

        # Test Values

        assert np.isclose(prob['q.mag_area'], 16.500000, rtol = .01)
        assert np.isclose(prob['q.m_mag'], 18562.500000, rtol = .01)
        assert np.isclose(prob['q.cost'], 816750.000000, rtol = .01)
        assert np.isclose(prob['q.total_pod_mass'], 21562.500000, rtol = .01)
        assert np.isclose(prob['p.lam'], 0.600000, rtol = .01)
        assert np.isclose(prob['p.track_ind'], 4.28571e-6, rtol = .01)
        assert np.isclose(prob['p.b0'], 1.055475, rtol = .01)
        assert np.isclose(prob['p.mag_area'], 16.500000, rtol = .01)
        assert np.isclose(prob['p.omegab'], 240.855437, rtol = .01)
        assert np.isclose(prob['p.w_track'], .75, rtol = .01)
        assert np.isclose(prob['p.fyu'], 520814.278077, rtol = .01)
        assert np.isclose(prob['p.fxu'], 2430517.899848, rtol = .01)
        assert np.isclose(prob['p.ld_ratio'], 0.214281, rtol = .01)
        assert np.isclose(prob['p.pod_weight'], 29430.000000, rtol = .01)
        assert np.isclose(prob['p.track_res'], 0.004817, rtol = .01)
