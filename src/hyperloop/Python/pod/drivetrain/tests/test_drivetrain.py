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
        prob['motor_max_current'] = 42.0
        prob['motor_LD_ratio'] = 0.83
        prob['design_torque'] = -0.801933
        prob['design_power'] = -0.394 * 746 / 1.844
        prob['motor.idp1.n_phases'] = 3.0
        prob['motor.motor_size.kappa'] = 1 / 1.75
        prob['motor.idp2.pole_pairs'] = 6.0
        prob['motor.motor_size.core_radius_ratio'] = 0.0
        prob['motor_oversize_factor'] = 1.0

        # setup inverter
        prob['inverter_efficiency'] = 1.0

        # setup battery
        prob['battery_cross_section_area'] = 2.0
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


        assert np.isclose(prob['battery_mass'], 8.84, rtol=0.001)
        assert np.isclose(prob['battery_volume'], 2991.51743192, rtol=0.001)
        assert np.isclose(prob['battery_cost'], 673.4, rtol=0.001)
        assert np.isclose(prob['motor_power_input'], 194.63856993, rtol=0.001)
        assert np.isclose(prob['motor_volume'], 379903.090621, rtol=0.001)
        assert np.isclose(prob['motor_diameter'], 0.0770663714581, rtol=0.001)
        assert np.isclose(prob['motor_length'], 0.0639650883103, rtol=0.001)
        assert np.isclose(prob['motor_mass'], 1.22089240568, rtol=0.001)
        assert np.isclose(prob['battery_length'], prob['battery_volume'] / 2.0, rtol=0.001)
