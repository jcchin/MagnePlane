import numpy as np
from openmdao.api import Group, Problem

from Python.pod.drivetrain.electric_motor import MotorGroup


def create_problem():
    prob = Problem()
    prob.root = MotorGroup()
    return prob


class TestMotor(object):
    def test_case1_vs_npss(self):

        prob = create_problem()

        prob.setup()

        prob['max_current'] = 42.0
        prob['speed'] = 1900.0
        prob['motor_size.L_D_ratio'] = 0.83
        prob['max_rpm'] = 3500.0
        prob['design_power'] = 0.394*746
        prob['n_phases'] = 3.0
        prob['motor_size.kappa'] = 1/1.75
        prob['pole_pairs'] = 6.0
        prob['motor_size.core_radius_ratio'] = 0.0

        prob.run()

        assert np.isclose(prob['motor.I0'], 2.83587716914, rtol = 0.001)
        assert np.isclose(prob['voltage'], 7.71904539448, rtol = 0.001)
        assert np.isclose(prob['current'], 25.2153759296, rtol = 0.001)


    def test_case2_vs_npss(self):

        prob = create_problem()

        prob.setup()

        prob['max_current'] = 450.0
        prob['speed'] = 1900.0
        prob['motor_size.L_D_ratio'] = 0.83
        prob['max_rpm'] = 2500.0
        prob['design_power'] = 110000
        prob['n_phases'] = 3.0
        prob['motor_size.kappa'] = 0.5
        prob['pole_pairs'] = 6.0
        prob['motor_size.core_radius_ratio'] = 0.7

        prob.run()

        assert np.isclose(prob['motor.I0'], 3.53497914176, rtol = 0.001)
        assert np.isclose(prob['voltage'], 387.053305981, rtol = 0.001)
        assert np.isclose(prob['current'], 226.767489571, rtol = 0.001)