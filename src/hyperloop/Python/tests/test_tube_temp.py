from __future__ import print_function
import numpy as np

from openmdao.api import Problem, Group
from openmdao.components.indep_var_comp import IndepVarComp

from hyperloop.Python.tube.tube_wall_temp import TubeTemp

def create_problem(tempGroup):
    root = Group()
    prob = Problem(root)
    prob.root.add('tt', tempGroup)
    return prob

class TestTubeWall(object):
    def test_tube_temp(self):
        prob = create_problem(TubeTemp())

        params = (('P', 0.304434211, {'units': 'psi'}),
                  ('T', 1710., {'units': 'degR'}))
        prob.root.add('des_vars', IndepVarComp(params), promotes=['*'])
        prob.root.connect('P', 'tt.nozzle_air.P')
        prob.root.connect('T', 'tt.nozzle_air.T')
        prob.root.connect('P', 'tt.bearing_air.P')
        prob.root.connect('T', 'tt.bearing_air.T')

        prob.setup(check=True)

        #tm.nozzle_air.setTotalTP(1710, 0.304434211)

        prob['tt.nozzle_air.W'] = 1.08
        prob['tt.bearing_air.W'] = 0.
        prob[
            'tt.radius_outer_tube'] = 2.22504 / 2.  #, units = 'm', iotype='in', desc='Tube out diameter') #7.3ft
        prob[
            'tt.tm.length_tube'] = 482803.  #, units = 'm', iotype='in', desc='Length of entire Hyperloop') #300 miles, 1584000ft
        prob[
            'tt.tm.num_pods'] = 34.  #, units = 'K', iotype='in', desc='Number of Pods in the Tube at a given time') #
        #prob['tt.temp_boundary'] = 322.361#, units = 'K', iotype='in', desc='Average Temperature of the tube') #
        prob[
            'tt.tm.temp_outside_ambient'] = 305.6  #, units = 'K', iotype='in', desc='Average Temperature of the outside air') #

        prob.run()

        rtol = 2.1e-2  # seems a little generous

        npss = 353244.
        pyc = prob['tt.tm.heat_rate_pod']
        rel_err = abs(npss - pyc) / npss
        print('heat_rate_pod:', npss, pyc, rel_err)
        #self.assertLessEqual(rel_err, tol)

        npss = 12010290.
        pyc = prob['tt.tm.total_heat_rate_pods']
        rel_err = abs(npss - pyc) / npss
        print('total_heat_rate_pods:', npss, pyc, rel_err)
        #self.assertLessEqual(rel_err, tol)

        npss = 123775609.
        pyc = prob['tt.tm.GrDelTL3']
        rel_err = abs(npss - pyc) / npss
        #print('GrDelTL3:', npss, pyc, rel_err)
        assert np.isclose(pyc, npss, rtol=rtol)

        npss = 0.707
        pyc = prob['tt.tm.Pr']
        rel_err = abs(npss - pyc) / npss
        #print('Pr:', npss, pyc, rel_err)
        assert np.isclose(pyc, npss, rtol=rtol)

        npss = 23163846280.
        pyc = prob['tt.tm.Gr']
        rel_err = abs(npss - pyc) / npss
        print('Gr:', npss, pyc, rel_err)
        #self.assertLessEqual(rel_err, tol)

        npss = 16369476896.
        pyc = prob['tt.tm.Ra']
        rel_err = abs(npss - pyc) / npss
        print('Ra:', npss, pyc, rel_err)
        #self.assertLessEqual(rel_err, tol)

        npss = 281.6714
        pyc = prob['tt.tm.Nu']
        rel_err = abs(npss - pyc) / npss
        #print('Nu:', npss, pyc, rel_err)
        assert np.isclose(pyc, npss, rtol=rtol)

        #self.assertLessEqual(rel_err, tol)
        #http://www.egr.msu.edu/~somerton/Nusselt/ii/ii_a/ii_a_3/ii_a_3_a.html
        npss = 0.02655
        pyc = prob['tt.tm.k']
        rel_err = abs(npss - pyc) / npss
        #print('k:', npss, pyc, rel_err)
        assert np.isclose(pyc, npss, rtol=rtol)

        npss = 3.3611
        pyc = prob['tt.tm.h']
        rel_err = abs(npss - pyc) / npss
        #print('h:', npss, pyc, rel_err)
        assert np.isclose(pyc, npss, rtol=rtol)

        npss = 3374876.
        pyc = prob['tt.tm.area_convection']
        rel_err = abs(npss - pyc) / npss
        #print('area_convection:', npss, pyc, rel_err)
        assert np.isclose(pyc, npss, rtol=rtol)

        npss = 57.10
        pyc = prob['tt.tm.q_per_area_nat_conv']
        rel_err = abs(npss - pyc) / npss
        print('q_per_area_nat_conv:', npss, pyc, rel_err)
        #self.assertLessEqual(rel_err, tol)

        npss = 192710349.
        pyc = prob['tt.tm.total_q_nat_conv']
        rel_err = abs(npss - pyc) / npss
        print('total_q_nat_conv:', npss, pyc, rel_err)
        #self.assertLessEqual(rel_err, tol)

        npss = 1074256.
        pyc = prob['tt.tm.area_viewing']
        rel_err = abs(npss - pyc) / npss
        #print('area_viewing:', npss, pyc, rel_err)
        assert np.isclose(pyc, npss, rtol=rtol)

        npss = 350.
        pyc = prob['tt.tm.q_per_area_solar']
        rel_err = abs(npss - pyc) / npss
        #print('q_per_area_solar:', npss, pyc, rel_err)
        assert np.isclose(pyc, npss, rtol=rtol)

        npss = 375989751.
        pyc = prob['tt.tm.q_total_solar']
        rel_err = abs(npss - pyc) / npss
        #print('q_total_solar:', npss, pyc, rel_err)
        assert np.isclose(pyc, npss, rtol=rtol)

        npss = 3374876.115
        pyc = prob['tt.tm.area_rad']
        rel_err = abs(npss - pyc) / npss
        #print('area_rad:', npss, pyc, rel_err)
        assert np.isclose(pyc, npss, rtol=rtol)

        npss = 59.7
        pyc = prob['tt.tm.q_rad_per_area']
        rel_err = abs(npss - pyc) / npss
        print('q_rad_per_area:', npss, pyc, rel_err)
        #self.assertLessEqual(rel_err, tol)

        npss = 201533208.
        pyc = prob['tt.tm.q_rad_tot']
        rel_err = abs(npss - pyc) / npss
        print('q_rad_tot:', npss, pyc, rel_err)
        #self.assertLessEqual(rel_err, tol)

        npss = 394673364.
        pyc = prob['tt.tm.q_total_out']
        rel_err = abs(npss - pyc) / npss
        print('q_total_out:', npss, pyc, rel_err)
        #self.assertLessEqual(rel_err, tol)


if __name__ == "__main__":
    unittest.main()
