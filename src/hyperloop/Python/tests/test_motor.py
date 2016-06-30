import pytest
from hyperloop.Python import electric_motor
import numpy as np
from openmdao.api import Group, Problem


def create_problem(motor):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', motor)
    return prob


class TestMotor(object):
    def test_case1_vs_npss(self):

        motor = electric_motor.ElectricMotor()

        prob = create_problem(motor)

        prob.setup()

        prob['comp.torque'] = 310.35 * 0.737
        prob['comp.max_rpm'] = 2500.0
        prob['comp.design_power'] = 110000.0 / 746.0
        prob['comp.speed'] = 1900.0
        prob['comp.POLE_PAIRS'] = 6.0
        prob['comp.i0'] = 0.0
        prob['comp.i0_des'] = 0.0
        prob['comp.imax'] = 450.0
        prob['comp.NPHASE'] = 3.0
        prob['comp.KAPPA'] = 0.5
        prob['comp.LD_RATIO'] = 0.83
        prob['comp.rd'] = 0.4
        prob['comp.AS'] = 95000.0
        prob['comp.resistance'] = 0.0
        prob['comp.inductance'] = 0.0

        prob.run()

        # assert np.isclose(prob['comp.phase_current'], 75.000, rtol = 0.001)
        # assert np.isclose(prob['comp.phase_voltage'], 1277.478, rtol = 0.001)
        # assert np.isclose(prob['comp.frequency'], 190.000, rtol = 0.001)
        # assert np.isclose(prob['comp.phase'], 0.000, rtol = 0.001)
        # assert np.isclose(prob['comp.kv'], 5.11363, rtol = 0.001)
        # assert np.isclose(prob['comp.weight'], 116.39, rtol = 0.001)

    def test_case2_vs_npss(self):
        motor = electric_motor.ElectricMotor()

        prob = create_problem(motor)

        prob.setup()

        prob['comp.torque'] = 310.35 * 0.737
        prob['comp.max_rpm'] = 2500.0
        prob['comp.design_power'] = 110000.0 / 746.0
        prob['comp.speed'] = 1900.0
        prob['comp.POLE_PAIRS'] = 6.0
        prob['comp.i0'] = 0.0
        prob['comp.i0_des'] = 0.0
        prob['comp.imax'] = 450.0
        prob['comp.NPHASE'] = 3.0
        prob['comp.KAPPA'] = 0.5
        prob['comp.LD_RATIO'] = 0.83
        prob['comp.rd'] = 0.4
        prob['comp.AS'] = 9500.0
        prob['comp.resistance'] = 0.0
        prob['comp.inductance'] = 0.0


        prob.run()

        # assert np.isclose(prob['comp.phase_current'], 75.000, rtol = 0.001)
        # assert np.isclose(prob['comp.phase_voltage'], 1277.478, rtol = 0.001)
        # assert np.isclose(prob['comp.frequency'], 190.000, rtol = 0.001)
        # assert np.isclose(prob['comp.phase'], 0.000, rtol = 0.001)
        # assert np.isclose(prob['comp.kv'], 5.11363, rtol = 0.001)
        # assert np.isclose(prob['comp.weight'], 116.39, rtol = 0.001)