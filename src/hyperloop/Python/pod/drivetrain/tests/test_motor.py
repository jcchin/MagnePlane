import numpy as np
from openmdao.api import Group, Problem

from hyperloop.Python.pod.drivetrain.electric_motor import MotorGroup


def create_problem():
    prob = Problem()
    prob.root = MotorGroup()
    return prob


class TestMotor(object):
    def test_case1_vs_npss(self):

        prob = create_problem()

        prob.setup()

        prob['motor_max_current'] = 42.0
        # prob['speed'] = 1900.0
        prob['motor_LD_ratio'] = 0.822727
        # prob['max_rpm'] = 3500.0
        prob['design_torque'] = -0.801933
        prob['design_power'] = -0.394*746 / 1.844
        prob['idp1.n_phases'] = 3.0
        prob['motor_size.kappa'] = 1/1.75
        prob['idp2.pole_pairs'] = 6.0
        prob['motor_size.core_radius_ratio'] = 0.0
        prob['motor_oversize_factor'] = 1.0
        prob.run()

        assert np.isclose(prob['motor.I0'], 2.83556, rtol = 0.001)
        assert np.isclose(prob['voltage'], 7.71165, rtol = 0.001)
        assert np.isclose(prob['current'], 25.21524, rtol = 0.001)
        assert np.isclose(prob['motor_volume'], 379903, rtol=0.001)
        assert np.isclose(prob['motor_volume'], 379903, rtol=0.001)
        assert np.isclose(prob['motor_length'], 0.0635909, rtol=0.001)
        assert np.isclose(prob['motor_diameter'], 0.0772928, rtol=0.001)
        assert np.isclose(prob['motor_mass'], 1.22089, rtol=0.001)

    def test_case2_vs_npss(self):

        prob = create_problem()

        prob.setup()

        prob['motor_max_current'] = 450.0
        # prob['speed'] = 1900.0
        prob['motor_LD_ratio'] = 0.83
        # prob['max_rpm'] = 2500.0
        prob['design_power'] = -110000
        prob['design_torque'] = -420.169
        prob['idp1.n_phases'] = 3.0
        prob['motor_size.kappa'] = 0.5
        prob['idp2.pole_pairs'] = 6.0
        prob['motor_size.core_radius_ratio'] = 0.7
        prob['motor_oversize_factor'] = 1.0

        prob.run()
        #
        assert np.isclose(prob['motor.I0'], 3.66357, rtol = 0.001)
        assert np.isclose(prob['voltage'], 505.4611, rtol = 0.001)
        assert np.isclose(prob['current'], 226.767489571, rtol = 0.001)