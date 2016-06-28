import pytest
from hyperloop.Python import ElectricMotor
import numpy as np
from openmdao.api import Group, Problem


def create_problem(motor):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', motor)
    return prob

class TestMotor(object):


    def test_case1_vs_npss(self):


        motor  =  ElectricMotor.ElectricMotor()

        prob = create_problem(motor)

        prob.setup()

        prob['comp.Torque'] = 310.35*0.737
        prob['comp.Max_RPM'] = 2500.0
        prob['comp.DesignPower'] = 110000.0/746.0
        prob['comp.Resistance'] = 0.0
        prob['comp.Inductance'] = 0.0,
        prob['comp.Kv']  =  0.1
        prob['comp.Speed'] = 1900.0
        prob['comp.imax'] = 450.0
        prob['comp.PolePairs'] = 6.0
        prob['comp.kappa'] = 0.5
        prob['comp.LDratio'] = 0.83
        prob['comp.R0'] = 0.004
        prob['comp.I0'] = 0.0
        prob['comp.I0_Des'] = 0.0
        prob['comp.nphase'] = 3.0
        prob['comp.CoreRadiusRatio'] = 0.7
        prob['comp.B_p'] = 1.5

        prob.run()

        assert np.isclose(prob['comp.phaseCurrent'], 101.686, rtol = 0.001)
        assert np.isclose(prob['comp.phaseVoltage'], 456.555, rtol = 0.001)
        assert np.isclose(prob['comp.Frequency'], 190, rtol = 0.001)
        assert np.isclose(prob['comp.Phase'], 0, rtol = 0.001)
        assert np.isclose(prob['comp.Kv'], 5.11363, rtol = 0.001)
        assert np.isclose(prob['comp.Mass'], 116.39, rtol = 0.001)


    def test_case2_vs_npss(self):
        motor  =  ElectricMotor.ElectricMotor()

        prob = create_problem(motor)

        prob.setup()

        prob['comp.Torque'] = 400.0*0.737
        prob['comp.Max_RPM'] = 4000.0
        prob['comp.DesignPower'] = 150000.0/746.0
        prob['comp.Resistance'] = 0.0
        prob['comp.Inductance'] = 0.0,
        prob['comp.Kv']  =  0.1
        prob['comp.Speed'] = 2500.0
        prob['comp.imax'] = 450.0
        prob['comp.PolePairs'] = 6.0
        prob['comp.kappa'] = 0.5
        prob['comp.LDratio'] = 1.2
        prob['comp.R0'] = 0.007
        prob['comp.I0'] = 0.0
        prob['comp.I0_Des'] = 0.0
        prob['comp.nphase'] = 3.0
        prob['comp.CoreRadiusRatio'] = 1.5
        prob['comp.B_p'] = 2.2

        prob.run()

        assert np.isclose(prob['comp.phaseCurrent'], 101.686, rtol = 0.001)
        assert np.isclose(prob['comp.phaseVoltage'], 512.926, rtol = 0.001)
        assert np.isclose(prob['comp.Frequency'], 250, rtol = 0.001)
        assert np.isclose(prob['comp.Phase'], 0, rtol = 0.001)
        assert np.isclose(prob['comp.Kv'], 5.99999, rtol = 0.001)
        assert np.isclose(prob['comp.Mass'], 103.859, rtol = 0.001)
