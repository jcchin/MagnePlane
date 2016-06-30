"""
Basic model of a Brushless DC (BLDC) motor
Basic model of a Brushless DC (BLDC) motor to aid with motor sizing.
Calculates Phase Current, Phase Voltage, Frequency, Motor Size, and Weight.
"""
from __future__ import print_function
import numpy
from openmdao.api import IndepVarComp, Component, Problem, Group


class ElectricMotor(Component):
    """
    Params
    ------
    Torque : float
        Output Torque from motor in N*m. Default value is 1000.0
    Max_RPM : float
        Maximum rotations per minute of motor in rpm. Default value is 4600.0
    DesignPower : float
        Desired design value for motor power in hp. Default value is 0.0
    Resistance : float
        Resistance of Stator in Ohms. Default value is 0.0
    Inductance : float
        Motor inductance in Henrys. Default value is 0.0
    Speed : float
        Output shaft mechanical speed in rpm. Default value is 50.0
    Kv : float
        Motor constant (Speed/volt) in rad/s/V. Default value is 0.1
    Kt : float
        Motor constant (Torque/amp) in ft-lb/A. Default value is 10.0
    PolePairs : float
        Number of pole pairs in motor. Default value is 6.0
    R0 : float
        Total Internal Resistance at 0degC in Ohms. Default value is 0.2
    I0 : float
        Motor No-load current in Amps. Default value is 0.0
    I0_Des : float
        Motor No-load Current at Nbase in Amps. Default value is 0.0
    imax : float
        Max motor phase current in Amps. Default value is 500.0
    nphase : float
        Number of motor phases. Default value is 3.0
    kappa : float
        Ratio of Base speed to max speed. Default value is 0.6
    Dbase : float
        Base 8000hp diameter for scaling purposes in m.  Default value is 0.48
    Lbase : float
        Base 8000hp length for scaling purposes in m. Default value is 0.4
    LDratio : float
        Length to diameter ratio of motor. Default value is 1.5
    CoreRadiusRatio : float
        Ratio of inner diameter of core to outer. Default value is 0.4
    k_Friction : float
        Friction coefficient calibration factor. Default value is 1.0
    Rd : float
        D-axis resistance per motor phase at very high speed (short circuit). Default value is
    efficiency : float
        Motor efficiency (Input Power/Mechanical Power output). Default value is 0.0
    Pmax : float
        Conversion of DesignPower in hp to Watts. Default value is 0.0

    Returns
    -------
    phaseCurrent : float
        Phase current for AC current in Amps. Default value is 0.0
    phaseVoltage : float
        AC voltage across motor in Volts. Default value is 500.0
    Phase : float
        phase offset between Current and Voltage. Default value is 0.0
    Frequency : float
        Frequency of Electric output waveform in Hz. Default value is 60.0
    Weight : float
        Weight of motor in kg. Default value is 0.0
    D2L : float
        D-squared*L parameter which is ~ to Torque in mm^3. Default value is 0.0
    D2L_ft : float
        D-squared*L parameter converted to ft^3. Default value is 0.0
    Volume : float
        Volume of motor modeled as a cylinder in m^3. Default value is 0.0
    Volume_ft : float
        Volume of motor modeled as a cylinder in ft^3. Default value is 0.0



    Notes
    -----
    [1] Main Source: Georgia Tech ASDL:
    "Conceptual Modeling of Electric and Hybrid-Electric Propulsion for UAS Applications" (Gladin, Ali, Collins)
    """

    def __init__(self):
        super(ElectricMotor, self).__init__()

        # Inputs/Params
        self.add_param('Resistance',
                       val=0.0,
                       desc='Resistance of Stator',
                       units='ohm')
        self.add_param('Inductance',
                       val=0.0,
                       desc='Motor inductance',
                       units='H')
        self.add_param('Speed',
                       val=1900.0,
                       desc='Output shaft mechanical speed',
                       units='rpm')
        self.add_param('Torque',
                       val=310.35 * 0.737,
                       desc='Output torque',
                       units='N*m')
        self.add_param('Kv', val=0.1, desc='Speed/volt', units='rad/s/V')
        self.add_param('Kt', val=10.0, desc='Torque/amp', units='N*m/A')
        self.add_param('PolePairs',
                       val=6.0,
                       desc='Number of pole pairs in motor',
                       units='none')  # f=w*PP/2*pi
        self.add_param('R0',
                       val=0.004,
                       desc='Motor Phase Internal Resistance at 0degC',
                       units='ohm')  # ohms  # total internal resistance
        self.add_param('I0', val=0.0, desc='Motor No-load current', units='A')
        self.add_param('I0_Des',
                       val=0.0,
                       desc='Motor No-load Current at Nbase',
                       units='A')
        self.add_param('imax',
                       val=450.0,
                       desc='Max motor phase current',
                       units='A')
        self.add_param('nphase',
                       val=3.0,
                       desc='Number of motor phases',
                       units='none')
        self.add_param('DesignPower',
                       val=110000.0 / 746.0,
                       desc='Design value of motor',
                       units='hp')
        self.add_param('Max_RPM',
                       val=2500.0,
                       desc='max rpm of motor',
                       units='rpm')
        self.add_param('kappa',
                       val=0.5,
                       desc='Base speed/max speed',
                       units='none')
        self.add_param('Dbase',
                       val=0.48,
                       desc='Base 8000hp diameter for scaling purposes',
                       units='m')
        self.add_param('Lbase',
                       val=0.4,
                       desc='Base 8000hp length for scaling purposes',
                       units='m')
        self.add_param('LDratio',
                       val=0.83,
                       desc='Length to diameter ratio of motor',
                       units='none')
        self.add_param('CoreRadiusRatio',
                       val=0.7,
                       desc='ratio of inner diameter of core to outer',
                       units='none')
        self.add_param(
            'Rd',
            val=0.0,
            desc='D-axis resistance per motor phase at very high speed (short circuit)',
            units='ohm')

        # Desired outputs
        self.add_output('Tmax', val=0.0, desc='MaxTorque', units='N*m')
        self.add_output('phaseCurrent',
                        val=0.0,
                        desc='Phase current',
                        units='A')
        self.add_output('phaseVoltage',
                        val=500.0,
                        desc='AC voltage across motor',
                        units='V')
        self.add_output('Frequency',
                        val=60.0,
                        desc='Frequency of Electric output waveform',
                        units='Hz')
        self.add_output('Phase',
                        val=0.0,
                        desc='phase offset between Current and Voltage',
                        units='rad')
        self.add_output('Weight', val=0.0, desc='Weight of motor', units='kg')
        self.add_output('D2L',
                        val=0.0,
                        desc='D-squared*L parameter which is ~ to Torque',
                        units='mm**3')
        self.add_output('D2L_ft',
                        val=0.0,
                        desc='D-squared*L parameter converted to ft^3',
                        units='ft**3')
        self.add_output('Volume',
                        val=0.0,
                        desc='Volume of motor modeled as a cylinder',
                        units='m**3')
        self.add_output('Volume_ft',
                        val=0.0,
                        desc='Volume of motor modeled as a cylinder in ft',
                        units='ft**3')

    def solve_nonlinear(self, params, unknowns, resids):
        Torque = params['Torque']
        Max_RPM = params['Max_RPM']
        DesignPower = params['DesignPower']

        Speed = params['Speed']
        imax = params['imax']
        PolePairs = params['PolePairs']
        Inductance = params['Inductance']
        kappa = params['kappa']
        LDratio = params['LDratio']
        R0 = params['R0']
        I0 = params['I0']
        I0_Des = params['I0_Des']
        nphase = params['nphase']
        CoreRadiusRatio = params['CoreRadiusRatio']
        Rd = params['Rd']
        Resistance = params['Resistance']

        unknowns['Tmax'] = self.Tmax_calc(DesignPower, Max_RPM, kappa)
        Tmax = unknowns['Tmax']

        params['Kv'] = ((imax - I0_Des) / Tmax) * (30.0 / numpy.pi)  #A/(N*m)
        Kv = params['Kv']
        params['Kt'] = (1.0 / Kv) * (30.0 / numpy.pi)  #
        Kt = params['Kt']

        unknowns['D2L'] = 293722.0 * (Tmax**0.7592)  # mm^3
        D2L = unknowns['D2L']
        unknowns['D2L_ft'] = D2L * 3.53147 * (10** -8)  #ft^3
        D2L_ft = unknowns['D2L_ft']

        params['Dbase'] = ((D2L / LDratio)**(1.0 / 3.0)) / 1000.0  #m
        Dbase = params['Dbase']

        unknowns['Volume'] = numpy.pi * (LDratio * Dbase) * (Dbase / 2 - (
            (Dbase / 2) * CoreRadiusRatio))**2
        Volume = unknowns['Volume']
        #Volume = pi*length*(total radius-core radius)^2
        unknowns['Volume_ft'] = Volume * 35.3147
        Volume_ft = unknowns['Volume_ft']

        unknowns['Weight'] = 0.0000070646 * (D2L**0.9386912061)
        Weight = unknowns['Weight']

        unknowns['phaseCurrent'] = self.phaseCurrent_calc(DesignPower, Max_RPM,
                                                          nphase, Kt, I0)
        phaseCurrent = unknowns['phaseCurrent']
        unknowns['phaseVoltage'] = self.phaseVoltage_calc(
            Speed, kappa, Max_RPM, phaseCurrent, nphase, R0, Rd, Kv)
        phaseVoltage = unknowns['phaseVoltage']

        unknowns['Frequency'] = Speed * PolePairs / 60.0
        Frequency = unknowns['Frequency']
        unknowns['Phase'] = self.Phase_calc(Kv, Speed, PolePairs, Torque,
                                            Resistance, Inductance, Frequency)
        Phase = unknowns['Phase']

    def Tmax_calc(self, DesignPower, Max_RPM, kappa):
        #Calculates max torque to later obtain D2L and Kt at max current
        Pmax = DesignPower * 746.0  #hp to Watts
        wmax = Max_RPM * 2 * numpy.pi / 60.0  #rpm to rad/s
        wbase = kappa * wmax  #rad/s
        Tmax = Pmax / wbase  #W/(rad/s) = (N*m/s)/(rad/s) = N*m
        return Tmax

    def phaseCurrent_calc(self, DesignPower, Max_RPM, nphase, Kt, I0):
        # Calculates Current from equation 5 in paper and converts to phase current
        Pmax = DesignPower * 746.0  #hp to W
        wmax = Max_RPM * 2 * numpy.pi / 60.0  #rpm to rad/s
        Torque = Pmax / wmax  #W/(rad/s) = (N*m/s)/(rad/s) = N*m
        Current = I0 + Torque / Kt  #A + (N*m)/(N*m/A)
        phaseCurrent = Current / nphase  #A
        return phaseCurrent

    def phaseVoltage_calc(self, Speed, kappa, Max_RPM, phaseCurrent, nphase,
                          R0, Rd, Kv):
        # Calculates Voltage from equation 6 in paper and converts to phase voltage
        N = Speed  #*2*numpy.pi/60.0 #RPM to rad/s
        Nbase = kappa * Max_RPM  #*2*numpy.pi/60.0 #rpm to rad/s
        if N > Nbase:
            Voltage = (phaseCurrent * nphase) * (R0 + Rd *
                                                 (1 - N / Nbase)**2) + N / Kv
        elif N < Nbase:
            Voltage = (phaseCurrent * nphase) * R0 + N / Kv
        phaseVoltage = Voltage * numpy.sqrt(3.0 / 2.0)
        return phaseVoltage

    def Phase_calc(self, Kv, Speed, PolePairs, Torque, Resistance, Inductance,
                   Frequency):
        Kt = .73756214837 / Kv
        Freq = Speed * PolePairs / (2 * numpy.pi)
        Current = Torque / Kt
        resistorVoltage = Current * Resistance
        inductorImpedance = Frequency * Inductance
        inductorVoltage = Current * inductorImpedance
        speedVoltage = Kv * Speed
        realVoltage = speedVoltage + resistorVoltage
        Phase = numpy.arctan2(inductorVoltage, realVoltage)
        return Phase


if __name__ == '__main__':
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', ElectricMotor())
    prob.setup()
    prob.run()

    # Printing various input parameters to check with C++ model/inputs
    print('kappa: %f' % prob['comp.kappa'])
    print('imax [A]: %f' % prob['comp.imax'])
    print('Max_RPM: %f' % prob['comp.Max_RPM'])
    print('DesignPower [hp]: %f' % prob['comp.DesignPower'])
    print('LDratio:%f' % prob['comp.LDratio'])
    print('R0: %f' % prob['comp.R0'])
    print('CoreRadiusRatio: %f' % prob['comp.CoreRadiusRatio'])
    print('I0 [A]: %f' % prob['comp.I0'])
    print('Torque [N*m]: %f' % prob['comp.Torque'])

    print('-----------------------------')

    # Outputs
    print('Phase Current [A]: %f' % prob['comp.phaseCurrent'])
    print('Phase Voltage [V]: %f' % prob['comp.phaseVoltage'])
    print('Frequency [Hz]: %f ' % prob['comp.Frequency'])
    print('Phase: %f' % prob['comp.Phase'])
    print('Tmax: %f' % prob['comp.Tmax'])
    print('Kv [rad/s/V]: %f' % prob['comp.Kv'])
    print('Kt [N-m/A]: %f' % prob['comp.Kt'])
    # print('Dbase: %f' %prob['comp.Dbase'])
    # print('Lbase: %f' % prob['comp.Lbase'])
    print('Motor Weight [kg]: %f ' % prob['comp.Weight'])
    print('Motor Size (D^2*L) [mm^3]: %f ' % prob['comp.D2L'])
    print('Motor Size (D^2*L) [ft^3]: %f ' % prob['comp.D2L_ft'])
