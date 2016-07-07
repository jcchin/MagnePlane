from __future__ import print_function

from openmdao.api import Group, Problem

from hyperloop.Python.pod.drivetrain.drivetrain import Drivetrain

def create_problem(comp):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', comp)
    return prob

class TestDrivetrain(object):
    def test_case1_vs_npss(self):
        prob = create_problem(Drivetrain())
        prob.setup()

        # # setup ElectricMotor
        # prob['comp.Motor.Torque'] = 310.35*0.737
        # prob['comp.Motor.Max_RPM'] = 2500.0
        # prob['comp.Motor.DesignPower'] = 110000.0/746.0
        # prob['comp.Motor.Resistance'] = 0.0
        # prob['comp.Motor.Inductance'] = 0.0,
        # prob['comp.Motor.Kv']  =  0.1
        # prob['comp.Motor.Speed'] = 1900.0
        # prob['comp.Motor.imax'] = 450.0
        # prob['comp.Motor.PolePairs'] = 6.0
        # prob['comp.Motor.kappa'] = 0.5
        # prob['comp.Motor.LDratio'] = 0.83
        # prob['comp.Motor.R0'] = 0.004
        # prob['comp.Motor.I0'] = 0.0
        # prob['comp.Motor.I0_Des'] = 0.0
        # prob['comp.Motor.nphase'] = 3.0
        # prob['comp.Motor.CoreRadiusRatio'] = 0.7
        # prob['comp.Motor.B_p'] = 1.5
        #
        # # setup Inverter
        # prob['comp.Inverter.Efficiency'] = 0.704995
        # prob['comp.Inverter.DesignPower'] = 8000
        # prob['comp.Inverter.OutputVoltage'] = 10.1528
        # prob['comp.Inverter.OutputCurrent'] = 11.0613
        # prob['comp.Inverter.InputVoltage'] = 24
        # prob['comp.Inverter.OutputFrequency'] = 200
