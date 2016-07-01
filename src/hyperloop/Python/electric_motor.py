"""
Models the equivalent circuit model of a brushless DC (BLDC) motor to perform motor sizing.
Calculates Phase Current, Phase Voltage, Frequency, Motor Size, and Weight.
"""
from __future__ import print_function

import numpy
from openmdao.api import Component, Problem, Group


class ElectricMotor(Component):
    """
    Params
    ------
    torque : float
        Output Torque from motor in N*m. Default value is 1000.0
    max_rpm : float
        Maximum rotational speed of motor in RPM. Default value is 4600.0
    design_power : float
        Desired design value for motor power in hp. Default value is 0.0
    speed : float
        Output shaft mechanical speed in RPM. Default value is 1000.0
    POLE_PAIRS : float
        Number of pole pairs in motor. Default value is 6.0
    i0 : float
        Motor No-load current in Amps. Default value is 0.0
    i0_des : float
        Motor No-load Current at Nbase in Amps. Default value is 0.0
    imax : float
        Max motor phase current in Amps. Default value is 500.0
    N_PHASE : float
        Number of motor phases. Default value is 3.0
    KAPPA : float
        Ratio of Base speed to max speed. Default value is 0.6
    L_D_RATIO : float
        Length to diameter ratio of motor. Default value is 1.5
    A_S : float
        Electrical loading of total stator current flowing per unit circumference in A/m.
        Default value is 95000.0
    r_d : float
        D-axis resistance per motor phase at very high speed (short circuit). Default value is 0.4
    resistance : float
        Resistance of Stator in Ohms. Default value is 0.0
    inductance : float
        Motor inductance in Henrys. Default value is 0.0

    Returns
    -------
    tmax : float
        Maximum possible torque for the motor in N*m. Default value is 0.0
    k_v : float
        Motor voltage/back emf constant (Voltage/speed) in V/(rad/s). Default value is 0.1
    k_t : float
        Motor torque constant (Torque/Current) in N*m/A. Default value is 10.0
    d2l : float
        D-squared*L parameter which is proportional to Torque in mm^3. Default value is 0.0
    d2l_ft : float
        D-squared*L parameter converted to ft^3. Default value is 0.0
    d_base : float
        Base 8000hp diameter for scaling purposes in m.  Default value is 0.48
    weight : float
        Weight of motor in kg. Default value is 0.0
    phase_current : float
        Phase current for AC current in Amps. Default value is 0.0
    phase_voltage : float
        AC voltage across motor in Volts. Default value is 500.0
    frequency : float
        Frequency of Electric output waveform in Hz. Default value is 60.0
    phase : float
        phase offset between Current and Voltage. Default value is 0.0

    Notes
    -----
    [1] Main Source: Georgia Tech ASDL:
    "Conceptual Modeling of Electric and Hybrid-Electric Propulsion for UAS Applications"
    (Gladin, Ali, Collins)
    """

    def __init__(self):
        super(ElectricMotor, self).__init__()

        # Inputs/Params
        self.add_param('torque',
                       val=310.35 * 0.737,
                       desc='Output torque',
                       units='N*m')
        self.add_param('max_rpm',
                       val=2500.0,
                       desc='max rpm of motor',
                       units='rpm')
        self.add_param('design_power',
                       val=110000.0 / 746.0,
                       desc='Design value of motor',
                       units='hp')
        self.add_param('speed',
                       val=1900.0,
                       desc='Output shaft mechanical speed',
                       units='rpm')
        self.add_param('POLE_PAIRS',
                       val=6.0,
                       desc='Number of pole pairs in motor',
                       units='none')
        self.add_param('i0', val=0.0, desc='Motor No-load current', units='A')
        self.add_param('i0_des',
                       val=0.0,
                       desc='Motor No-load Current at Nbase',
                       units='A')
        self.add_param('imax',
                       val=450.0,
                       desc='Max motor phase current',
                       units='A')
        self.add_param('N_PHASE',
                       val=3.0,
                       desc='Number of motor phases',
                       units='none')
        self.add_param('KAPPA',
                       val=0.5,
                       desc='Base speed/max speed',
                       units='none')
        self.add_param('L_D_RATIO',
                       val=0.83,
                       desc='Length to diameter ratio of motor',
                       units='none')
        self.add_param('r_d',
                       val=0.4,
                       desc='D-axis resistance per motor phase at very high speed (short circuit)',
                       units='ohm')
        self.add_param('A_S',
                       val=95000.0,
                       desc='Electrical loading of total stator current flowing per unit circumference',
                       units='A/m')
        self.add_param('resistance',
                       val=0.0,
                       desc='Resistance of Stator',
                       units='ohm')
        self.add_param('inductance',
                       val=0.0,
                       desc='Motor inductance',
                       units='H')
        # Desired outputs
        self.add_output('pmax',val=0.0,desc='MaxPower',units='W')
        self.add_output('wmax',val=0.0,desc='max rotational speed',units='rad/s')
        self.add_output('tmax', val=0.0, desc='MaxTorque', units='N*m')
        self.add_output('k_v',
                        val=0.1,
                        desc='Motor voltage/back emf constant (Speed/volt)',
                        units='rad/s/V')
        self.add_output('k_t', val=10.0, desc='Motor torque constant (Torque/amp)', units='N*m/A')
        self.add_output('d2l',
                        val=0.0,
                        desc='D-squared*L parameter which is ~ to Torque',
                        units='mm**3')
        self.add_output('d2l_ft',
                        val=0.0,
                        desc='D-squared*L parameter converted to ft^3',
                        units='ft**3')
        self.add_output('d_base',
                        val=0.48,
                        desc='Base 8000hp diameter for scaling purposes',
                        units='m')
        self.add_output('l_base',
                        val=0.0,
                        desc='motor length',
                        units='m')
        self.add_output('weight', val=0.0, desc='Weight of motor', units='kg')
        self.add_output('phase_current',
                        val=0.0,
                        desc='Phase current',
                        units='A')
        self.add_output('phase_voltage',
                        val=500.0,
                        desc='AC voltage across motor',
                        units='V')
        self.add_output('frequency',
                        val=60.0,
                        desc='Frequency of Electric output waveform',
                        units='Hz')
        self.add_output('phase',
                        val=0.0,
                        desc='phase offset between Current and Voltage',
                        units='rad')


    def solve_nonlinear(self, params, unknowns, resids):

        #Calculating tmax
        unknowns['pmax'] = params['design_power'] * 746.0
        unknowns['wmax'] = params['max_rpm'] * 2 * numpy.pi / 60.0
        unknowns['tmax'] = unknowns['pmax']/(params['KAPPA']*unknowns['wmax'])
        tmax = unknowns['tmax']

        #Calculating motor constants
        unknowns['k_v'] = ((params['imax'] - params['i0_des']) / tmax) * (30.0 / numpy.pi)  # A/(N*m)
        unknowns['k_t'] = (1.0 / unknowns['k_v']) * (30.0 / numpy.pi)  #

        #Calculating sizing
        L_D_RATIO = params['L_D_RATIO']
        unknowns['d2l'] = 293722.0 * (tmax ** 0.7592)  # mm^3
        d2l = unknowns['d2l']
        unknowns['d2l_ft'] = d2l * 3.53147 * (10 ** -8)  # ft^3
        unknowns['d_base'] = ((d2l / L_D_RATIO) ** (1.0 / 3.0)) / 1000.0  # m
        unknowns['l_base'] = unknowns['d_base']*L_D_RATIO

        #Calculating weight
        unknowns['weight'] = 0.0000070646 * (d2l ** 0.9386912061) #relation in GT paper (Figure 6)

        #Calculating phase current, phase voltage, frequency, and phase
        unknowns['phase_current'] = self.phase_current_calc(unknowns,params,resids)
        unknowns['phase_voltage'] = self.phase_voltage_calc(unknowns,params,resids)
        unknowns['frequency'] = params['speed'] * params['POLE_PAIRS'] / 60.0
        unknowns['phase'] = self.phase_calc(unknowns,params,resids)


    def phase_current_calc(self,unknowns,params,resids):
        # Calculates Current from equation 5 in paper and converts to phase current
        t = unknowns['pmax'] / unknowns['wmax']  # W/(rad/s) = (N*m/s)/(rad/s) = N*m
        current = params['i0'] + t / unknowns['k_t']  # A + (N*m)/(N*m/A)
        phase_current = current / params['N_PHASE']  # A
        return phase_current

    def phase_voltage_calc(self,unknowns,params,resids):
        # Calculates Voltage from equation 6 in paper and converts to phase voltage
        imax = params['imax']
        N_PHASE = params['N_PHASE']
        k_v = unknowns['k_v']
        d_base = unknowns['d_base']
        phase_current = unknowns['phase_current']

        w = params['speed']*2*numpy.pi/60.0 #RPM to rad/s
        w_base = params['KAPPA']*params['max_rpm']*2*numpy.pi/60.0 #rpm to rad/s
        turns = params['A_S']*numpy.pi*d_base/imax/N_PHASE/2.0 #number of coil turns required
        r_per_km = 48.8387296964863*imax**(-1.00112597971171)
        l_wind = d_base*numpy.pi
        r_turn = l_wind*r_per_km/1000.0 #total resistance per turn

        if w > w_base:
            r_eq = r_turn*turns + params['r_d']*(1-w/w_base)**2
            voltage = (phase_current*N_PHASE)*r_eq + w/k_v
        elif w < w_base:
            r_eq = r_turn*turns
            voltage = (phase_current*N_PHASE)*r_eq + w/k_v
        phase_voltage = voltage * numpy.sqrt(3.0 / 2.0)
        return phase_voltage

    def phase_calc(self, unknowns,params,resids):
        speed = params['speed']
        k_v = unknowns['k_v']

        k_t = .73756214837 / k_v
        frequency = speed * params['POLE_PAIRS'] / (2 * numpy.pi)
        current = params['torque'] / k_t
        resistor_voltage = current * params['resistance']
        inductor_impedance = frequency * params['inductance']
        inductor_voltage = current * inductor_impedance
        speed_voltage = k_v * speed
        real_voltage = speed_voltage + resistor_voltage
        phase = numpy.arctan2(inductor_voltage, real_voltage)
        return phase


if __name__ == '__main__':
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', ElectricMotor())
    prob.setup()
    prob.run()

    # Printing various input parameters
    print('kappa: %f' % prob['comp.KAPPA'])
    print('imax [A]: %f' % prob['comp.imax'])
    print('Max RPM: %f' % prob['comp.max_rpm'])
    print('Design Power [hp]: %f' % prob['comp.design_power'])
    print('LD Ratio:%f' % prob['comp.L_D_RATIO'])
    print('I0 [A]: %f' % prob['comp.i0'])
    print('Torque [N*m]: %f' % prob['comp.torque'])

    print('-----------------------------')

    # Outputs
    print('Max Torque [N*m]: %f' % prob['comp.tmax'])
    print('Kv [rad/s/V]: %f' % prob['comp.k_v'])
    print('Kt [N-m/A]: %f' % prob['comp.k_t'])
    print('Motor Size (D^2*L) [mm^3]: %f ' % prob['comp.d2l'])
    print('Motor Size (D^2*L) [ft^3]: %f ' % prob['comp.d2l_ft'])
    #print('Diameter [m]: %f ' %prob['comp.d_base'])
    #print('Length [m]: %f ' %prob['comp.l_base'])
    print('Motor Weight [kg]: %f ' % prob['comp.weight'])
    print('Phase Current [A]: %f' % prob['comp.phase_current'])
    print('Phase Voltage [V]: %f' % prob['comp.phase_voltage'])
    print('Frequency [Hz]: %f ' % prob['comp.frequency'])
    print('Phase: %f' % prob['comp.phase'])
