"""
Models the equivalent circuit model of a brushless DC (BLDC) motor to perform motor sizing.
Calculates Phase Current, Phase Voltage, Frequency, Motor Size, and Weight.
"""
from __future__ import print_function

import numpy as np
from openmdao.api import Component, Problem, Group, Newton, IndepVarComp, ScipyGMRES


class MotorBalance(Component):
    """Creates an implicit connection between the
    """

    def __init__(self):
        super(MotorBalance, self).__init__()
        self.deriv_options['type'] = 'fd'
        self.add_state('I0',
                       val=40.0,
                       desc='motor no load current',
                       units='A',
                       lower=0.0,
                       upper=1000.0)
        self.add_param('power_input',
                       0.0,
                       desc='total power input into motor',
                       units='W')
        self.add_param('current', val=2.0, desc='total current through motor', units='A')
        self.add_param('voltage', val=500.0, desc='total voltage through motor', units='V')

    def solve_nonlinear(self, params, unknowns, resids):
        pass

    def apply_nonlinear(self, params, unknowns, resids):
        resids['I0'] = (params['current'] * params['voltage'] - params['power_input']) / 1000
        # print('current %f' % params['current'])
        # print('voltage %f' % params['voltage'])
        # print('I0: %f' % unknowns['I0'])
        # print('resid: %f' % resids['I0'])
        # print('power_input: %f' % params['power_input'])


class MotorGroup(Group):
    def __init__(self):
        super(MotorGroup, self).__init__()

        self.add('Motor', Motor(), promotes=['*'])
        self.add('MotorSize', MotorSize(), promotes=['*'])
        self.add('MotorBalance', MotorBalance(), promotes=['power_input', 'current', 'voltage'])
        self.connect('MotorSolver.I0', 'I0')

        self.nl_solver = Newton()
        self.nl_solver.options['maxiter'] = 1000
        self.nl_solver.options['atol'] = 5.0e-5

        self.ln_solver = ScipyGMRES()
        self.ln_solver.options['maxiter'] = 100


class MotorSize(Component):
    """
    Parameters
    ----------
    max_rpm : float
        maximum rotational speed of motor (RPM)
    design_power : float
        desired design value for motor power (W)
    max_current : float
        max motor phase current (A)
    n_phases : float
        number of motor phases (unitless)
    speed : float
        desired output shaft mechanical speed (RPM)
    kappa : float
        ratio of base speed to max speed (unitless)
    L_D_ratio : float
        length to diameter ratio of motor (unitless)

    Outputs
    -------
    D2L : float
        D-squared*L parameter which is proportional to Torque (mm^3)
    d_base : float
        base 8000hp diameter for scaling purposes (m)
    mass : float
        mass of motor (kg)
    l_base : float
        motor length (m)
    w_base : float
        base speed of motor (rad/s)
    max_torque : float
        maximum possible torque for motor (N*m)
    power_iron_loss : float
        total power loss due to iron core (W)
    power_mech : float
        mechanical power output of motor (W)
    power_windage_loss : float
        friction loss from motor operation (W)
    winding_resistance : float
        total resistance of copper winding (ohm)
    torque : float
        torque at max rpm (N*m)
    w_operating : float
        operating speed of motor (rad/s)
    """
    def __init__(self):
        super(MotorSize, self).__init__()
        self.deriv_options['type'] = 'fd'

        self.add_param('max_current',
                       val=42.0,
                       desc='max motor phase current',
                       units='A')
        self.add_param('speed', val=2000.0, desc='desired output shaft mechanical speed', units='RPM')
        self.add_param('L_D_ratio',
                       val=0.822727,
                       desc='length to diameter ratio of motor',
                       units='unitless')
        self.add_param('max_rpm',
                       val=3500.0,
                       desc='maximum rotational speed of motor',
                       units='RPM')
        self.add_param('design_power',
                       val=0.394 * 746,
                       desc='desired design value for motor power',
                       units='W')
        self.add_param('n_phases',
                       val=3.0,
                       desc='number of motor power phases',
                       units='unitless')
        self.add_param('kappa',
                       val=1 / 1.75,
                       desc='ratio of base speed to max speed',
                       units='unitless')
        self.add_output('d_base',
                        val=0.48,
                        desc='base 8000hp diameter for scaling purposes in m',
                        units='m')
        self.add_output('l_base',
                        val=0.4,
                        desc='motor length',
                        units='m')
        self.add_output('mass', val=0.0, desc='mass of motor', units='kg')
        self.add_output('max_torque', val=0.0, desc='maximum possible torque for motor', units='N*m')
        self.add_output('torque', val=1000.0, desc='torque at max_rpm', units='N*m')
        self.add_output('w_base', val=3000.0, desc=' base speed of motor ', units='rad/s')
        self.add_output('D2L', val=1.0, desc='D-squared*L parameter which is proportional to Torque', units='mm^3')
        self.add_output('power_mech', 0.0, desc='mechanical power output of motor',  units='W')
        self.add_output('w_operating', 0.0, desc='operating speed of motor', units='rad/s')
        self.add_output('power_iron_loss', 0.0, desc='total power loss due to iron core', units='W')
        self.add_output('winding_resistance', 0.0, desc='total resistance of copper winding', units='ohm')
        self.add_output('power_windage_loss', 0.0, desc='friction loss from motor operation', units='W')

    def solve_nonlinear(self, params, unknowns, resids):
        # calc max torque, rotational velocity, power
        w_max = params['max_rpm'] * 2.0 * np.pi / 60.0  # rad/sec
        unknowns['w_base'] = params['kappa'] * w_max
        unknowns['max_torque'] = params['design_power'] / unknowns['w_base']
        unknowns['torque'] = params['design_power'] / w_max
        unknowns['power_mech'] = unknowns['w_operating'] * unknowns['torque']
        unknowns['w_operating'] = params['speed'] * 2 * np.pi / 60.0

        # calc loss parameters
        unknowns['power_iron_loss'] = self.calculate_iron_loss(unknowns['d_base'], params['speed'], unknowns['l_base'])
        unknowns['winding_resistance'] = self.calculate_copper_loss(unknowns['d_base'], params['max_current'], params['n_phases'])
        unknowns['power_windage_loss'] = self.calculate_windage_loss(unknowns['w_operating'], unknowns['d_base'], unknowns['l_base'])
       
        # calc size
        unknowns['D2L'] = 293722.0 * np.power(unknowns['max_torque'], 0.7592)  # mm^3
        unknowns['d_base'] = np.power(unknowns['D2L'] / params['L_D_ratio'], 1.0 / 3.0) / 1000.0  # m
        unknowns['l_base'] = unknowns['d_base'] * params['L_D_ratio']  # m

        unknowns['mass'] = 0.0000070646 * np.power(unknowns['D2L'], 0.9386912061)  # kg, relation in GT paper (Figure 6)

    def calculate_windage_loss(self, w_operating, d_base, l_base):
        return 0
    #      # calc Reynolds number losses
    #      Re = np.power(d_base, 2.0) / 4.0 * w_operating / 2.075e-5 * 0.05
    #      c_friction = 0.01
    #      
    #      # TODO no nested loop
    #      # while (abs(diff) > 0.001):
    #      #     Cd = 1.0 / np.power(2.04+1.768*np.log(Re*np.power(c_friction, 0.5)), 2.0)
    #      #     diff = (Cd - c_friction) / Cd
    #      #     c_friction = Cd
    #     
    #      # calculate cylinder loss
    #      P_windage_cylinder_loss = c_friction * np.pi * np.power(w_operating, 3.0) * np.power(
    #          d_base / 2.0, 4.0) * l_base * 1.2041
    #      # calculate cylinder face loss
    #      Re_r = np.power(d_base, 2.0) / 4.0 * w_operating / 2.075e-5
    #      c_disk_friction = 0.08 / np.power(0.05, 0.167) / np.power(Re, 0.25)
    #      P_windage_face_loss = 0.5 * c_friction * 1.2041 * np.power(w_operating, 3.0) * np.power(
    #          d_base / 2.0, 5.0) * (1.0 - np.power(core_radius_ratio, 5.0))
    #      # P_windage_total_loss = P_windage_face_loss + P_windage_face_loss
    #      P_windage_total_loss = 0

    def calculate_copper_loss(self, d_base, max_current, n_phases):
        # D-axis resistance per motor phase at very high-speed (short-cruit)
        Rd = 0.0

        # calc static loading factor from GT paper
        As = 688.7 * max_current

        # calc total resistance in winding
        n_coil_turns = As * np.pi * d_base / max_current / n_phases / 2.0
        resistance_per_km_per_turn = 48.8387296964863 * np.power(max_current, -1.00112597971171)
        winding_len = d_base * 3.14159
        resistance_per_turn = resistance_per_km_per_turn * winding_len / 1000.
        return resistance_per_turn * n_coil_turns * n_phases

    def calculate_iron_loss(self, d_base, speed, l_base):
        pole_pairs = 6.0
        stator_core_density = 7650.0  # kg/m^3
        core_radius_ratio = 0  # r_inner / r_outter
        # hysteresis loss constant
        Kh = 0.0275  # W/(kg T^2 Hz)
        # iron eddy loss constant
        Kc = 1.83e-5  # W/(kg T^2 Hz^2)
        # correction factor
        Ke = 2.77e-5  # W/(kg T^1.5 Hz^1.5)
        # stator magnetic flux density
        Bp = 1.22  # T

        # iron losses
        freq = speed * pole_pairs / 60.0
        volume_iron = np.pi * l_base * np.power(d_base / 2.0, 2.0) * (
            1.0 - np.power(core_radius_ratio, 2.0))
        iron_core_mass = stator_core_density * volume_iron
        power_iron_loss = (Kh * np.power(Bp, 2.0) * freq + Kc * np.power(Bp * freq, 2.0) + Ke * np.power(Bp * freq, 1.5)) * iron_core_mass
        return power_iron_loss


class Motor(Component):
    """ Represents an electric motor which can calculate output current and voltage
    based on motor sizing parameters and losses. Used in conjunction with MotorBalance
    to find the correct no load current for this Motor

    Based on work done by Gladin et. al. ([1]_)

    Params
    ------
    w_operating : float
        operating speed of motor (rad/s)
    torque : float
        output Torque from motor (N*m)
    pole_pairs : float
        Number of pole pairs in motor
    I0 : float
        motor No-load Current (A)
    max_current : float
        max motor phase current (A)
    max_torque : float
        maximum possible torque for motor (N*m)
    power_iron_loss : float
        total power loss due to iron core (W)
    power_mech : float
        mechanical power output of motor (W)
    power_windage_loss : float
        friction loss from motor operation (W)
    winding_resistance : float
        total resistance of copper winding (ohm)
    torque : float
        torque at max_rpm (N*m)

    Outputs
    -------
    current : float
        current through motor (A)
    voltage : float
        voltage across motor in (V)
    phase_current : float
        phase current through motor (A)
    phase_voltage : float
        phase voltage across motor (V)
    frequency : float
        Frequency of electric output waveform (Hz)
    power_input : float
        total required power input into motor
    
    References
    ----------
    [1] "J. Gladin, K. Ali, K. Collins, "Conceptual Modeling of Electric and Hybrid-Electric
    Propulsion for UAS Applications," Georgia Tech, 2015.

    """

    def __init__(self):
        super(Motor, self).__init__()
        self.deriv_options['type'] = 'fd'
        self.add_param('max_current',
                       val=42.0,
                       desc='max operating current',
                       units='A')
        self.add_param('I0',
                       val=40.0,
                       desc='motor no load current',
                       units='A')
        self.add_param('n_phases',
                       val=3.0,
                       desc='number of motor power phases',
                       units='unitless')
        self.add_param('w_operating', 0.0, desc='operating speed of motor', units='rad/s')
        self.add_param('pole_pairs',
                       val=6.0,
                       desc='number of motor pole_pairs',
                       units='unitless')
        self.add_param('torque', val=1.0, desc='torque at max_rpm', units='N*m')
        self.add_param('winding_resistance', 0.0, desc='total resistance of copper winding', units='ohm')
        self.add_param('power_windage_loss', 0.0, desc='friction loss from motor operation', units='W')
        self.add_param('power_mech', 0.0, desc='mechanical power output of motor', units='W')
        self.add_param('power_iron_loss', 0.0, desc='total power loss due to iron core', units='W')
        self.add_param('max_torque', val=0.0, desc='maximum possible torque for the motor', units='N*m')
        
        self.add_output('current', val=2.0, desc='current through motor', units='A')
        self.add_output('phase_current', val=1.0, desc='phase current through motor', units='A')
        self.add_output('voltage', val=500.0, desc='voltage through motor', units='V')
        self.add_output('phase_voltage', val=1.0, desc='phase voltage through motor', units='V')
        self.add_output('frequency', val=60.0, desc='frequency of electric output waveform', units='Hz')
        self.add_output('power_input', 1.0, desc='total required power input into motor', units='W')

    def solve_nonlinear(self, params, unknowns, resids):
        # voltage constant
        k_v = (params['max_current'] - params['I0']) / params['max_torque'] * 30.0 / np.pi
        # torque constant
        k_t = 30.0 / np.pi * 1.0 / k_v

        # Calculating phase current, phase voltage, frequency, and phase
        unknowns['current'] = params['I0'] + params['torque'] / k_t
        power_copper_loss = np.power(unknowns['current'], 2.0) * params['winding_resistance']
        unknowns['power_input'] = params['power_mech'] + params['power_windage_loss'] + params['power_iron_loss'] + power_copper_loss
        
        unknowns['current'] = params['I0'] + params['torque'] / k_t
        unknowns['phase_current'] = unknowns['current'] / params['n_phases']
        
        unknowns['voltage'] = unknowns['current'] * params['winding_resistance'] + params['w_operating'] / (k_v * np.pi / 30.0)
        unknowns['phase_voltage'] = unknowns['voltage'] * np.sqrt(3.0 / 2.0)
        
        unknowns['frequency'] = params['w_operating'] / np.pi * params['pole_pairs'] / 60.0


if __name__ == '__main__':
    from openmdao.api import SqliteRecorder
    from os import remove
    from sqlitedict import SqliteDict

    prob = Problem()
    prob.root = MotorGroup()
    # prob.root.add('Indep_I0', IndepVarComp('I0_input', 0.0))
    prob.root.add('init_vars', IndepVarComp('max_current', 42.0, units='A'), promotes=['max_current'])

    rec = SqliteRecorder('drivetraindb')
    rec.options['record_params'] = True
    rec.options['record_metadata'] = True
    prob.driver.add_recorder(rec)

    prob.setup()
    prob.root.list_connections()
    prob.print_all_convergence()

    # view_tree(prob)
    prob.run()

    db = SqliteDict('drivetraindb', 'openmdao')
    # p# print(db.keys())
    # data = db['rank0:Driver/1']
    # p# print(data['Parameters'])
    # p# print(data['Unknowns'])



    prob.cleanup()
    remove('drivetraindb')
