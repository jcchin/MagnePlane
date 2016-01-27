import numpy as np
import unittest

from openmdao.api import Problem, Group
from openmdao.components.indep_var_comp import IndepVarComp

import os,sys,inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.insert(0,parentdir)

from tube_wall_temp import TubeTemp
#from hyperloop.tube_wall_temp import TubeWallTemp


# class TubeHeatBalance(Assembly):

#     def configure(self):

#         tm = self.add('tm', TubeWallTemp())
#         #tm.bearing_air.setTotalTP()
#         driver = self.add('driver',BroydenSolver())
#         driver.add_parameter('tm.temp_boundary',low=0.,high=10000.)
#         driver.add_constraint('tm.ss_temp_residual=0')
#         driver.workflow.add(['tm'])


class TubeWallTestCase(unittest.TestCase):

    def setUp(self):

        self.prob = Problem()
        self.prob.root = Group()

        self.prob.root.add('tt',TubeTemp())

        params = (
            ('P', 0.304434211, {'units':'psi'}),
            ('T', 1710., {'units':'degR'})
        )
        self.prob.root.add('des_vars', IndepVarComp(params), promotes=['*'])
        self.prob.root.connect('P', 'tt.nozzle_air.P')
        self.prob.root.connect('T', 'tt.nozzle_air.T')
        self.prob.root.connect('P', 'tt.bearing_air.P')
        self.prob.root.connect('T', 'tt.bearing_air.T')

        self.prob.setup(check=True)

    def test_tube_temp(self):

        # test = set_as_top(TubeHeatBalance())
        #set input values

        #tm.nozzle_air.setTotalTP(1710, 0.304434211)

        self.prob['tt.nozzle_air.W'] = 1.08
        self.prob['tt.bearing_air.W'] = 0.
        self.prob['tt.radius_outer_tube'] = 2.22504/2.#, units = 'm', iotype='in', desc='Tube out diameter') #7.3ft
        self.prob['tt.tm.length_tube'] = 482803.#, units = 'm', iotype='in', desc='Length of entire Hyperloop') #300 miles, 1584000ft
        self.prob['tt.tm.num_pods'] = 34.#, units = 'K', iotype='in', desc='Number of Pods in the Tube at a given time') #
        #self.prob['tt.temp_boundary'] = 322.361#, units = 'K', iotype='in', desc='Average Temperature of the tube') #
        self.prob['tt.tm.temp_outside_ambient'] = 305.6#, units = 'K', iotype='in', desc='Average Temperature of the outside air') #

        self.prob.run()

        tol = 2.1e-2 # seems a little generous

        npss = 353244.
        pyc = self.prob['tt.tm.heat_rate_pod']
        rel_err = abs(npss - pyc)/npss
        print('heat_rate_pod:', npss, pyc, rel_err)
        self.assertLessEqual(rel_err, tol)

        npss = 12010290.
        pyc = self.prob['tt.tm.total_heat_rate_pods']
        rel_err = abs(npss - pyc)/npss
        print('total_heat_rate_pods:', npss, pyc, rel_err)
        self.assertLessEqual(rel_err, tol)

        npss = 123775609.
        pyc = self.prob['tt.tm.GrDelTL3']
        rel_err = abs(npss - pyc)/npss
        print('GrDelTL3:', npss, pyc, rel_err)
        self.assertLessEqual(rel_err, tol)

        npss = 0.707
        pyc = self.prob['tt.tm.Pr']
        rel_err = abs(npss - pyc)/npss
        print('Pr:', npss, pyc, rel_err)
        self.assertLessEqual(rel_err, tol)

        npss = 23163846280.
        pyc = self.prob['tt.tm.Gr']
        rel_err = abs(npss - pyc)/npss
        print('Gr:', npss, pyc, rel_err)
        self.assertLessEqual(rel_err, tol)

        npss = 16369476896.
        pyc = self.prob['tt.tm.Ra']
        rel_err = abs(npss - pyc)/npss
        print('Ra:', npss, pyc, rel_err)
        self.assertLessEqual(rel_err, tol)

        npss = 281.6714
        pyc = self.prob['tt.tm.Nu']
        rel_err = abs(npss - pyc)/npss
        print('Nu:', npss, pyc, rel_err)
        self.assertLessEqual(rel_err, tol)
         #http://www.egr.msu.edu/~somerton/Nusselt/ii/ii_a/ii_a_3/ii_a_3_a.html
        npss = 0.02655
        pyc = self.prob['tt.tm.k']
        rel_err = abs(npss - pyc)/npss
        print('k:', npss, pyc, rel_err)
        self.assertLessEqual(rel_err, tol)

        npss = 3.3611
        pyc = self.prob['tt.tm.h']
        rel_err = abs(npss - pyc)/npss
        print('h:', npss, pyc, rel_err)
        self.assertLessEqual(rel_err, tol)

        npss = 3374876.
        pyc = self.prob['tt.tm.area_convection']
        rel_err = abs(npss - pyc)/npss
        print('area_convection:', npss, pyc, rel_err)
        self.assertLessEqual(rel_err, tol)

        npss = 57.10
        pyc = self.prob['tt.tm.q_per_area_nat_conv']
        rel_err = abs(npss - pyc)/npss
        print('q_per_area_nat_conv:', npss, pyc, rel_err)
        self.assertLessEqual(rel_err, tol)

        npss = 192710349.
        pyc = self.prob['tt.tm.total_q_nat_conv']
        rel_err = abs(npss - pyc)/npss
        print('total_q_nat_conv:', npss, pyc, rel_err)
        self.assertLessEqual(rel_err, tol)

        npss = 1074256.
        pyc = self.prob['tt.tm.area_viewing']
        rel_err = abs(npss - pyc)/npss
        print('area_viewing:', npss, pyc, rel_err)
        self.assertLessEqual(rel_err, tol)

        npss = 350.
        pyc = self.prob['tt.tm.q_per_area_solar']
        rel_err = abs(npss - pyc)/npss
        print('q_per_area_solar:', npss, pyc, rel_err)
        self.assertLessEqual(rel_err, tol)

        npss = 375989751.
        pyc = self.prob['tt.tm.q_total_solar']
        rel_err = abs(npss - pyc)/npss
        print('q_total_solar:', npss, pyc, rel_err)
        self.assertLessEqual(rel_err, tol)

        npss = 3374876.115
        pyc = self.prob['tt.tm.area_rad']
        rel_err = abs(npss - pyc)/npss
        print('area_rad:', npss, pyc, rel_err)
        self.assertLessEqual(rel_err, tol)

        npss = 59.7
        pyc = self.prob['tt.tm.q_rad_per_area']
        rel_err = abs(npss - pyc)/npss
        print('q_rad_per_area:', npss, pyc, rel_err)
        self.assertLessEqual(rel_err, tol)

        npss = 201533208.
        pyc = self.prob['tt.tm.q_rad_tot']
        rel_err = abs(npss - pyc)/npss
        print('q_rad_tot:', npss, pyc, rel_err)
        self.assertLessEqual(rel_err, tol)

        npss = 394673364.
        pyc = self.prob['tt.tm.q_total_out']
        rel_err = abs(npss - pyc)/npss
        print('q_total_out:', npss, pyc, rel_err)
        self.assertLessEqual(rel_err, tol)


if __name__ == "__main__":
    unittest.main()