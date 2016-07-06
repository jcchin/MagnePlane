"""
Models the equivalent circuit model of a brushless DC (BLDC) motor to perform motor sizing.
Calculates Phase Current, Phase Voltage, Frequency, Motor Size, and Weight.
"""
from __future__ import print_function
from openmdao.devtools.partition_tree_n2 import view_tree

import numpy as np
from openmdao.api import Component, Problem, Group, ScipyOptimizer, IndepVarComp, ExecComp, ScipyGMRES, NLGaussSeidel
# from openmdao.api import pyOptSparseDriver
# from openmdao.drivers.pyoptsparse_driver import pyOptSparseDriver

class MotorGroup(Group):
    def __init__(self):
        super(MotorGroup, self).__init__()

        self.add('Indep_R0', IndepVarComp('R0', 0.004), promotes=['R0'])
        self.add('Indep_I0', IndepVarComp('I0', 0.0), promotes=['I0'])

        self.add('MotorLoss', MotorLoss(), promotes=['*'])
        self.add('Motor', Motor(), promotes=['*'])
        self.add('MotorSize', MotorSize(), promotes=['*'])
        self.add('init_vars', IndepVarComp('imax', 500.0), promotes=['imax'])

        # Finite Difference
        self.deriv_options['type'] = 'fd'
        self.deriv_options['form'] = 'forward'
        self.deriv_options['step_size'] = 1.0e-6

        alpha = 0.4
        self.add('obj_1', ExecComp('obj = alpha * (R0 - R_calc) + (1-alpha) * (current * voltage - power_input)'), promotes=['*'])
        # self.add('obj_2', ExecComp('obj = current * voltage - power_input'))

        self.add('con_1', ExecComp('con1 = current * voltage'), promotes=['*'])
        self.add('con_2', ExecComp('con2 = power_input'), promotes=['*'])
        # self.add('con_3', ExecComp('con1 = R0'), promotes=['*'])
        # self.add('con_4', ExecComp('con1 = R_calc'), promotes=['*'])

        self.nl_solver = NLGaussSeidel()
        self.ln_solver = ScipyGMRES()

        # self.add('MotorLoss', MotorLoss(), promotes=['d_base', 'wbase', 'l_base', 'woperating'])
        # self.add('Motor', Motor(), promotes=['tmax', 'torque'])
        # self.add('MotorSize', MotorSize, promotes=['d_base', 'wbase', 'l_base',
        #                                            'tmax', 'torque', 'mass', 'n_phases', 'design_power', 'max_rpm', 'L_D_RATIO',
        #                                            'KAPPA', 'efficiency'])

        # self.connect('MotorSize.d_base', 'MotorLoss.d_base')
        # self.connect('MotorSize.wbase', 'MotorLoss.wbase')
        # self.connect('MotorSize.l_base', 'MotorLoss.l_base')
        #
        # self.connect('MotorSize.tmax', 'Motor.tmax')
        # self.connect('MotorSize.torque', 'Motor.torque')
        #
        # self.connect('MotorLoss.woperating', 'Motor.woperating')



class MotorSize(Component):

    def __init__(self):
        super(MotorSize, self).__init__()
        self.add_param('L_D_RATIO',
                       val=0.83,
                       desc='Length to diameter ratio of motor',
                       units='none')
        self.add_param('max_rpm',
                       val=2500.0,
                       desc='max rpm of motor',
                       units='rpm')
        self.add_param('design_power',
                       val=110000,
                       desc='Design value of motor',
                       units='W')
        self.add_param('n_phases',
                       val=3.0,
                       desc='number of motor power phases',
                       units='unitless')
        self.add_param('KAPPA',
                       val=0.6,
                       desc='Base speed/max speed',
                       units='none')
        self.add_param('efficiency',
                       val=1.0,
                       desc='P input / p output',
                       units='unitless')


        self.add_output('d_base',
                        val=0.48,
                        desc='Base 8000hp diameter for scaling purposes',
                        units='m')
        self.add_output('l_base',
                        val=0.4,
                        desc='motor length',
                        units='m')
        self.add_output('mass', val=0.0, desc='Weight of motor', units='kg')
        self.add_output('tmax', val=0.0, desc='MaxTorque', units='N*m')
        self.add_output('torque', val=1000.0, desc='torque at max_rpm')
        self.add_output('wbase', val=3000.0, desc='base speed', units='rad/s')
        self.add_output('D2L', val=1.0, desc='motor volume', units='mm^3')



    def solve_nonlinear(self, params, unknowns, resids):
        # calc max torque, rotational velocity
        w_max = params['max_rpm'] * 2.0 * np.pi / 60.0 # rad/sec
        unknowns['wbase'] = params['KAPPA']*w_max
        unknowns['tmax'] = params['design_power']/unknowns['wbase']
        unknowns['torque'] = params['design_power']/w_max
        # calc sizing
        unknowns['D2L'] = 293722.0 * np.power(unknowns['tmax'], 0.7592)  # mm^3

        unknowns['d_base'] = np.power(unknowns['D2L'] / params['L_D_RATIO'], 1.0 / 3.0) / 1000.0  # m
        unknowns['l_base'] = unknowns['d_base']* params['L_D_RATIO'] # m

        # calc mass
        unknowns['mass'] = 0.0000070646 * np.power(unknowns['D2L'], 0.9386912061) # kg, relation in GT paper (Figure 6)


class MotorLoss(Component):
    def __init__(self):
        super(MotorLoss, self).__init__()
        self.add_param('imax',
                       val=250.0,
                       desc='Max motor phase current',
                       units='A')
        self.add_param('d_base',
                        val=0.48,
                        desc='Base 8000hp diameter for scaling purposes',
                        units='m')
        self.add_param('wbase', val=1.0, desc='base speed', units='rad/s')
        self.add_param('speed', val=1.0, desc='operating speed', units='RPM')

        self.add_param('l_base',
                        val=0.0,
                        desc='motor length',
                        units='m')
        self.add_param('current',
                       val=1.0,
                       desc='total current through motor',
                       units='A')
        self.add_param('n_phase',
                       val=3.0,
                       desc='number of phases of motor',
                       units='unitless')
        self.add_param('torque',
                       val=3.0,
                       desc='motor torque',
                       units='N*m')
        self.add_output('woperating',
                        1.0,
                        desc='operating speed of motor',
                        units='rad/s')
        self.add_output('R_calc',
                        0.0,
                        desc='total copper resistance of motor winding',
                        units='R')
        self.add_output('power_input',
                        0.0,
                        desc='total power input into motor',
                        units='W')

    def solve_nonlinear(self, params, unknowns, resids):
        pole_pairs = 6.0
        core_radius_ratio = 0.7
        core_density = 7650.0
        unknowns['woperating'] = params['speed'] * np.pi / 60.0
        Kh = 0.0275
        Kc = 0.0000183
        Ke = 0.0000277
        Bp = 1.5

        # D-axis resistance per motor phase at very high-speed (short-cruit)
        Rd = 0.0
        # calc static loading factor from GT paper
        As = 688.7 * params['imax']

        # number of coil turns for 3 phase motor
        params['n_phase'] = 3.0
        n_coil_turns = As * np.pi * params['d_base'] / params['imax'] / params['n_phase'] / 2.0
        resistance_per_km_per_turn = 48.8387296964863*np.power(params['imax'], -1.00112597971171)
        winding_len = params['d_base'] * 3.14159
        resistance_per_turn = resistance_per_km_per_turn * winding_len / 1000.0
        total_winding_resistance = resistance_per_turn * n_coil_turns * params['n_phase']

        unknowns['R_calc'] = total_winding_resistance

        # copper losses
        total_resistance = resistance_per_turn * n_coil_turns
        if (params['speed'] > params['wbase']):
            total_resistance += Rd*(1.0-params['wbase']/params['speed'])
        P_copper_loss = np.power(params['current'], 2.0) * total_resistance

        # iron losses
        freq = params['speed'] * pole_pairs / 60.0
        volume_iron = np.pi * params['l_base'] * np.power(params['d_base'] / 2.0, 2.0) * (1.0 - np.power(core_radius_ratio, 2.0))
        iron_core_mass = core_density * volume_iron
        P_iron_loss = (Kh * np.power(Bp, 2.0) * freq + Kc*np.power(Bp*freq, 2.0) + Ke*np.power(Bp*freq, 1.5)) * iron_core_mass

        # calculate windage losses
        unknowns['woperating'] = params['speed'] * np.pi / 60.0
        Re = np.power(params['d_base'],2.0) / 4.0 * unknowns['woperating'] / 2.075e-5*0.05
        c_friction = 0.01
        diff = 1.0

        # TODO no nested loop
        # while (abs(diff) > 0.001):
        #     Cd = 1.0 / np.power(2.04+1.768*np.log(Re*np.power(c_friction, 0.5)), 2.0)
        #     diff = (Cd - c_friction) / Cd
        #     c_friction = Cd

        # calculate cylinder loss
        P_windage_cylinder_loss = c_friction * np.pi * np.power(unknowns['woperating'], 3.0) * np.power(params['d_base'] / 2.0, 4.0) * params['l_base'] * 1.2041
        # calculate cylinder face loss
        Re_r = np.power(params['d_base'], 2.0)/4.0*unknowns['woperating']/2.075e-5
        c_disk_friction = 0.08 / np.power(0.05, 0.167) / np.power(Re, 0.25)
        P_windage_face_loss = 0.5 * c_friction * 1.2041 * np.power(unknowns['woperating'], 3.0) * np.power(params['d_base'] /2.0, 5.0)*(1.0-np.power(core_radius_ratio, 5.0))
        P_windage_total_loss = P_windage_face_loss + P_windage_face_loss

        # calculate total power input
        power_mech = unknowns['woperating'] * params['torque']
        unknowns['power_input'] = power_mech + P_windage_total_loss + P_copper_loss + P_iron_loss
        efficiency = unknowns['power_input'] / power_mech

class Motor(Component):
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
    pole_pairs : float
        Number of pole pairs in motor. Default value is 6.0
    i0 : float
        Motor No-load current in Amps. Default value is 0.0
    I0 : float
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
        Motor torque constant (Torque/params['Current']) in N*m/A. Default value is 10.0
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

    References
    ----------
    [1] "J. Gladin, K. Ali, K. Collins, "Conceptual Modeling of Electric and Hybrid-Electric
    Propulsion for UAS Applications," Georgia Tech, 2015.

    """

    def __init__(self):
        super(Motor, self).__init__()

        self.add_param('imax',
                       val=500.0,
                       desc='max operating current',
                       units='A')
        self.add_param('I0',
                       val=0.0,
                       desc='motor no load current',
                       units='A')
        self.add_param('R0',
                       val=0.004,
                       desc='motor resistance',
                       units='A')
        self.add_param('n_phases',
                       val=3.0,
                       desc='number of motor power phases',
                       units='unitless')
        self.add_param('pole_pairs',
                       val=6.0,
                       desc='number of motor pole_pairs',
                       units='unitless')
        self.add_param('torque', val=1.0, desc='torque at max_rpm')
        self.add_param('woperating',
                        1.0,
                        desc='operating speed of motor',
                        units='rad/s')
        self.add_param('tmax', val=0.0, desc='MaxTorque', units='N*m')
        self.add_output('current', val=2.0, desc='total current through motor', units='A')
        self.add_output('phase_current', val=1.0, desc='total phase current through motor', units='A')
        self.add_output('voltage', val=500.0, desc='total voltage through motor', units='V')
        self.add_output('phase_voltage', val=1.0, desc='total phase voltage through motor', units='V')
        self.add_output('frequency', val=60.0, desc='frequency of motor controller', units='Hz')

        # assert self.params['KAPPA'] * self.params['max_rpm'] < self.params['speed']


    def solve_nonlinear(self, params, unknowns, resids):
        print('Entered!')
        print()
        print(unknowns['current'])
        print(unknowns['voltage'])
        print(unknowns['frequency'])
        print(params['I0'])
        print(params['R0'])
        print()


        k_t =  params['tmax'] / (params['imax'] - params['I0'])
        k_v = (30.0 / np.pi) / k_t

        #Calculating phase current, phase voltage, frequency, and phase
        unknowns['current'] = params['I0'] + params['torque'] / k_t
        unknowns['phase_current'] = unknowns['current'] / params['n_phases']
        voltage = unknowns['current']*params['R0'] + params['woperating'] / k_v
        unknowns['phase_voltage'] = voltage * np.sqrt(3.0/2.0)
        unknowns['frequency'] = params['woperating'] / np.pi * params['pole_pairs'] / 60.0

    # def phase_calc(self, unknowns, params):
    #     speed = params['speed']
    #     k_v = unknowns['k_v']
    #
    #     k_t = .73756214837 / k_v
    #     frequency = speed * params['pole_pairs'] / (2.0 * np.pi)
    #     current = params['torque'] / k_t
    #     resistor_voltage = current * params['resistance']
    #     inductor_impedance = frequency * params['inductance']
    #     inductor_voltage = current * inductor_impedance
    #     speed_voltage = k_v * speed
    #     real_voltage = speed_voltage + resistor_voltage
    #     phase = np.arctan2(inductor_voltage, real_voltage)
    #     return phase


if __name__ == '__main__':

    prob = Problem()
    prob.root = MotorGroup()
    prob.driver = ScipyOptimizer()
    prob.driver.options['optimizer'] = 'SLSQP'
    prob.driver.options['tol'] = 1.0e-5


    prob.driver.add_desvar('I0', lower=0.0)
    prob.driver.add_desvar('R0', lower=0.0)

    prob.driver.add_objective('obj')
    # prob.driver.add_objective('obj_2')

    prob.driver.add_constraint('con1', lower=0.0)
    prob.driver.add_constraint('con2', lower=0.0)



    # prob.root.add('init_vars', IndepVarComp('imax', 500.0), promotes=['imax'])
    # prob.root.connect('imax', 'imax')

    prob.setup(check=True)
    # view_tree(prob)
    prob.run()


    # print(prob['comp.imax'])

    # # Printing various input parameters
    # print('kappa: %f' % prob['comp.KAPPA'])
    # print('imax [A]: %f' % prob['comp.imax'])
    # print('Max RPM: %f' % prob['comp.max_rpm'])
    # print('Design Power [hp]: %f' % prob['comp.design_power'])
    # print('LD Ratio:%f' % prob['comp.L_D_RATIO'])
    # print('I0 [A]: %f' % prob['comp.i0'])
    # print('Torque [N*m]: %f' % prob['comp.torque'])
    #
    # print('-----------------------------')
    #
    # # Outputs
    # print('Max Torque [N*m]: %f' % prob['comp.tmax'])
    # print('Kv [rad/s/V]: %f' % prob['comp.k_v'])
    # print('Kt [N-m/A]: %f' % prob['comp.k_t'])
    # print('Motor Size (D^2*L) [mm^3]: %f ' % prob['comp.d2l'])
    # print('Motor Size (D^2*L) [ft^3]: %f ' % prob['comp.d2l_ft'])
    # #print('Diameter [m]: %f ' %prob['comp.d_base'])
    # #print('Lengt
    # # h [m]: %f ' %prob['comp.l_base'])
    # print('Motor Weight [kg]: %f ' % prob['comp.weight'])
    # print('Phase Current [A]: %f' % prob['comp.phase_current'])
    # print('Phase Voltage [V]: %f' % prob['comp.phase_voltage'])
    # print('Frequency [Hz]: %f ' % prob['comp.frequency'])
    # print('Phase: %f' % prob['comp.phase'])


