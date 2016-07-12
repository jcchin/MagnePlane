from __future__ import division, print_function, absolute_import

import unittest
import numpy as np

try:
    from openmdao.api import pyOptSparseDriver
except:
    pyOptSparseDriver = None

from openmdao.api import ScipyOptimizer

from pointer.components import Problem, Trajectory, CollocationPhase

from hyperloop.Python.mission.rhs import MagnePlaneRHS


class MagneplaneTestStraightTrack(unittest.TestCase):

    def setUp(self):

        solver = 'SLSQP'
        num_seg = 10
        seg_ncn = 2
        rel_lengths = 'lgl'

        # Instantiate a problem and set it's root to an empty Trajectory
        prob = Problem()
        prob.add_traj(Trajectory("traj0"))

        if solver == 'SNOPT':
            driver = pyOptSparseDriver()
            driver.options['optimizer'] = solver
            driver.opt_settings['Major iterations limit'] = 1000
            driver.opt_settings['iSumm'] = 6
            driver.opt_settings['Major step limit'] = 0.5
            driver.opt_settings["Major feasibility tolerance"] = 1.0E-6
            driver.opt_settings["Major optimality tolerance"] = 1.0E-6
            driver.opt_settings["Minor feasibility tolerance"] = 1.0E-4
            driver.opt_settings['Verify level'] = 3
        else:
            driver = ScipyOptimizer()
            driver.options['tol'] = 1.0E-6
            driver.options['disp'] = True
            driver.options['maxiter'] = 500

        prob.trajectories["traj0"].add_objective(name="t", phase="phase0",
                                                 place="end", scaler=1.0)

        prob.driver = driver

        dynamic_controls = None

        static_controls = [{'name': 'mass', 'units': 'kg'},
                           {'name': 'g', 'units': 'm/s/s'},
                           {'name': 'theta', 'units': 'deg'},
                           {'name': 'psi', 'units': 'deg'},
                           {'name': 'Cd', 'units': 'unitless'},
                           {'name': 'S', 'units': 'm**2'},
                           {'name': 'p_tube', 'units': 'Pa'},
                           {'name': 'T_ambient', 'units': 'K'},
                           {'name': 'R', 'units': 'J/(kg*K)'},
                           {'name': 'D_magnetic', 'units': 'N'}]

        phase0 = CollocationPhase(name='phase0', rhs_class=MagnePlaneRHS,
                                  num_seg=num_seg, seg_ncn=seg_ncn,
                                  rel_lengths=rel_lengths,
                                  dynamic_controls=dynamic_controls,
                                  static_controls=static_controls)
        prob.trajectories["traj0"].add_phase(phase0)

        phase0.set_state_options('x', lower=0, upper=100000,
                                 ic_val=0, ic_fix=True,
                                 fc_val=1000, fc_fix=False, defect_scaler=0.1)

        phase0.set_state_options('y', lower=0, upper=0, ic_val=0, ic_fix=False,
                                 fc_val=0, fc_fix=False, defect_scaler=0.1)

        phase0.set_state_options('z', lower=0, upper=0, ic_val=0, ic_fix=False,
                                 fc_val=0, fc_fix=False, defect_scaler=0.1)

        phase0.set_state_options('v', lower=0, upper=np.inf, ic_val=0.0,
                                 ic_fix=True, fc_val=335.0,
                                 fc_fix=True, defect_scaler=0.1)

        phase0.set_static_control_options('theta', val=0.0, opt=False)
        phase0.set_static_control_options('psi', val=0.0, opt=False)
        phase0.set_static_control_options(name='g', val=9.80665, opt=False)
        phase0.set_static_control_options(name='mass', val=3100.0, opt=False)

        phase0.set_static_control_options(name='Cd', val=0.2, opt=False)
        phase0.set_static_control_options(name='S', val=1.4, opt=False)
        phase0.set_static_control_options(name='p_tube', val=850.0, opt=False)
        phase0.set_static_control_options(name='T_ambient', val=298.0,
                                          opt=False)
        phase0.set_static_control_options(name='R', val=287.0, opt=False)
        phase0.set_static_control_options(name='D_magnetic', val=150.0,
                                          opt=False)

        phase0.set_time_options(t0_val=0, t0_lower=0, t0_upper=0,
                                tp_val=30.0, tp_lower=0.5, tp_upper=1000.0)

        self.prob = prob

    def test_straight_track_time(self):
        self.prob.setup()
        self.prob.run()

        np.testing.assert_almost_equal(self.prob['traj0.phase0.rhs_c.t'][-1],
                                       35.09879341,
                                       decimal=3)

        # # SLSQP is naive about having more constraints than desvars.
        # Uncomment the following block if you get this error, and you
        # can see where these issues are coming from.
        # num_desvars = 0
        # for desvar in prob.driver._desvars:
        #     print(desvar, prob.driver._desvars[desvar]['size'])
        #     num_desvars = num_desvars + prob.driver._desvars[desvar]['size']
        # print('num_desvars = {0}'.format(num_desvars))
        #
        # num_cons = 0
        # for con in prob.driver._cons:
        #     print(con, prob.driver._cons[con]['size'])
        #     num_cons = num_cons + prob.driver._cons[con]['size']
        # print('num_cons = {0}'.format(num_cons))

        # # Uncomment the following to plot the trajectory
        # results = self.prob.trajectories['traj0'].phases['phase0'].\
        #     simulate(dt=0.05)
        # import matplotlib.pyplot as plt
        # plt.figure()
        # plt.plot(self.prob["traj0.phase0.rhs_c.t"],
        #          self.prob["traj0.phase0.rhs_c.x"],"ro")
        # #plt.plot(prob["phase0.rhs_c.x"],prob["phase0.rhs_c.y"],"bo")
        # plt.plot(self.prob["traj0.phase0.rhs_i.t"],
        #          self.prob["traj0.phase0.rhs_i.x"],"rx")
        # plt.plot(results["t"],results["x"],"b-")
        # plt.figure()
        # plt.plot(self.prob["traj0.phase0.rhs_c.t"],
        #          self.prob["traj0.phase0.rhs_c.v"],"ro")
        # #plt.plot(prob["phase0.rhs_c.x"],prob["phase0.rhs_c.y"],"bo")
        # plt.plot(self.prob["traj0.phase0.rhs_i.t"],
        #          self.prob["traj0.phase0.rhs_i.v"],"rx")
        # plt.plot(results["t"],results["v"],"b-")
        # plt.show()
