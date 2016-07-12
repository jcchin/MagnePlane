from __future__ import print_function

import numpy as np
from openmdao.api import Problem

from hyperloop.Python.pod.drivetrain.drivetrain import Drivetrain


def create_problem():
    prob = Problem()
    prob.root = Drivetrain()
    return prob


class TestDrivetrain(object):
    def test_case1_vs_npss(self):
        prob = create_problem()
        prob.setup()

        # setup ElectricMotor
        prob['motor.max_current'] = 42.0
        prob['motor.speed'] = 1900.0
        prob['motor.motor_size.L_D_ratio'] = 0.83
        # prob['max_rpm'] = 3500.0
        prob['motor.design_torque'] = 0.801933
        prob['motor.design_power'] = 0.394 * 746
        prob['motor.n_phases'] = 3.0
        prob['motor.motor_size.kappa'] = 1 / 1.75
        prob['motor.pole_pairs'] = 6.0
        prob['motor.motor_size.core_radius_ratio'] = 0.0

        # setup inverter
        prob['inverter.efficiency'] = 1.0

        # setup battery
        prob['des_time'] = 1.0
        prob['time_of_flight'] = 2.0
        prob['battery.q_l'] = 0.1
        prob['battery.e_full'] = 1.4
        prob['battery.e_nom'] = 1.2
        prob['battery.e_exp'] = 1.27
        prob['battery.q_n'] = 6.8
        prob['battery.t_exp'] = 1.0
        prob['battery.t_nom'] = 4.3
        prob['battery.r'] = 0.0046
        prob['battery.cell_mass'] = 170
        prob['battery.cell_height'] = 61.0
        prob['battery.cell_diameter'] = 33.0
        prob.root.list_connections()
        prob.run()


        assert np.isclose(prob['battery.mass'], 8.84, rtol=0.001)
        assert np.isclose(prob['battery.volume'], 2991.51743192, rtol=0.001)
        assert np.isclose(prob['battery.cost'], 673.4, rtol=0.001)
        assert np.isclose(prob['motor.power_input'], 194.63856993, rtol=0.001)
        assert np.isclose(prob['motor.volume'], 379903.090621, rtol=0.001)
        assert np.isclose(prob['motor.diameter'], 0.0770663714581, rtol=0.001)
        assert np.isclose(prob['motor.length'], 0.0639650883103, rtol=0.001)
        assert np.isclose(prob['motor.mass'], 1.22089240568, rtol=0.001)
