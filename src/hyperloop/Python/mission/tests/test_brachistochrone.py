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

class TestMagneplanBrachistochrone(unittest.TestCase):

    def setUp(self):
        solver = 'SLSQP'
        num_seg = 10
        seg_ncn = 2
        rel_lengths = 'lgl'

        # Instantiate a problem and set it's root to an empty Trajectory
        prob = Problem()
        prob.add_traj(Trajectory("traj0"))

        if solver == 'SNOPT' and pyOptSparseDriver is not None:
            driver = pyOptSparseDriver()
            driver.options['optimizer'] = solver
            driver.opt_settings['Major iterations limit'] = 1000
            driver.opt_settings['iSumm'] = 6
            driver.opt_settings['Major step limit'] = 0.5
            driver.opt_settings["Major feasibility tolerance"] = 1.0E-5
            driver.opt_settings["Major optimality tolerance"] = 1.0E-5
            driver.opt_settings["Minor feasibility tolerance"] = 1.0E-4
            driver.opt_settings['Verify level'] = 3
        else:
            driver = ScipyOptimizer()
            driver.options['tol'] = 1.0E-6
            driver.options['disp'] = True
            driver.options['maxiter'] = 500

        prob.driver = driver

        dynamic_controls = [{'name': 'theta', 'units': 'rad'},
                            {'name': 'psi', 'units': 'rad'}]

        static_controls = [{'name': 'mass', 'units': 'kg'},
                           {'name': 'g', 'units': 'm/s/s'},
                           {'name': 'Cd', 'units': 'unitless'},
                           {'name': 'D_magnetic', 'units': 'N'},
                           {'name': 'R', 'units': 'J/(kg*K)'},
                           {'name': 'S', 'units': 'm**2'},
                           {'name': 'T_ambient', 'units': 'K'},
                           {'name': 'p_tube', 'units': 'Pa'}]

        phase0 = CollocationPhase(name='phase0', rhs_class=MagnePlaneRHS,
                                  num_seg=num_seg, seg_ncn=seg_ncn,
                                  rel_lengths=rel_lengths,
                                  dynamic_controls=dynamic_controls,
                                  static_controls=static_controls)
        prob.trajectories["traj0"].add_phase(phase0)

        phase0.set_state_options('x', lower=0, upper=100,
                                 ic_val=0, ic_fix=True,
                                 fc_val=10, fc_fix=True,
                                 scaler=10.0, defect_scaler=0.1)

        phase0.set_state_options('y', lower=0, upper=0,
                                 ic_val=0, ic_fix=True,
                                 fc_val=0, fc_fix=True,
                                 scaler=10.0, defect_scaler=0.1)

        phase0.set_state_options('z', lower=-10, upper=10,
                                 ic_val=-10, ic_fix=True,
                                 fc_val=-5, fc_fix=True,
                                 scaler=10.0, defect_scaler=0.1)

        phase0.set_state_options('v', lower=0, upper=np.inf,
                                 ic_val=0.0, ic_fix=True,
                                 fc_val=10.0, fc_fix=False,
                                 scaler=10.0, defect_scaler=0.1)

        phase0.set_dynamic_control_options('theta', ic_val=-1.57, fc_val=0.2,
                                           opt=True, lower=-1.58, upper=1.58,
                                           scaler=0.01)

        phase0.set_dynamic_control_options('psi', ic_val=0.0, fc_val=0.0,
                                           opt=True, lower=0.0, upper=0.0,
                                           scaler=0.01)

        phase0.set_static_control_options(name='g', val=9.80665, opt=False)
        phase0.set_static_control_options(name='mass', val=1200, opt=False)
        phase0.set_static_control_options(name='Cd', val=0.0, opt=False)
        phase0.set_static_control_options(name='S', val=0.0, opt=False)
        phase0.set_static_control_options(name='p_tube', val=0.0, opt=False)
        phase0.set_static_control_options(name='T_ambient', val=298.0,
                                          opt=False)
        phase0.set_static_control_options(name='R', val=287.0, opt=False)

        # Set D_magnetic equal to the thrust value for now, no net forces
        # on the pod except gravity.
        phase0.set_static_control_options(name='D_magnetic', val=30000.0,
                                          opt=False)

        phase0.set_time_options(t0_val=0, t0_lower=0, t0_upper=0, tp_val=1.81,
                                tp_lower=0.5, tp_upper=10.0)

        prob.trajectories["traj0"].add_objective(name="t", phase="phase0",
                                                 place="end", scaler=1.0)

        # Do top-level FD
        prob.root.deriv_options['type'] = 'fd'

        self.prob = prob

    # @unittest.skipIf(True,'Test skipped with SLSQP until performance improves')
    def test_brachistochrone_solution(self):
        self.prob.setup()
        self.prob.run()
        # assert_almost_equal(self.prob['traj0.phase0.rhs_c.t'][-1],
        #                     1.8016,
        #                     decimal=4)

        phase0 = self.prob.trajectories['traj0'].phases['phase0']
        results = phase0.simulate(dt=0.05)

        import matplotlib.pyplot as plt
        plt.figure()
        plt.plot(self.prob["traj0.phase0.rhs_c.x"],
                 self.prob["traj0.phase0.rhs_c.z"], "ro")
        # plt.plot(prob["phase0.rhs_c.x"],prob["phase0.rhs_c.y"],"bo")
        plt.plot(self.prob["traj0.phase0.rhs_i.x"],
                 self.prob["traj0.phase0.rhs_i.z"], "rx")
        plt.plot(results["x"], results["z"], "b-")
        plt.gca().invert_yaxis()

        plt.figure()
        plt.plot(results["t"], results["theta"], "b-")

        plt.show()
