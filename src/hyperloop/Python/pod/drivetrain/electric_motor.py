"""
Models the equivalent circuit model of a brushless DC (BLDC) motor to perform motor sizing.
Calculates Phase Current, Phase Voltage, Frequency, Motor Size, and Weight.
"""
from __future__ import print_function

import numpy as np
from openmdao.api import Component, Problem, Group, Newton, LinearGaussSeidel, IndepVarComp, ScipyGMRES, NLGaussSeidel


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
        # print('-------------------')
        # print("HEREEEE")
        # print('-------------------')

        resids['I0'] = (params['current'] * params['voltage'] - params['power_input']) / 1000
        # print('current %f' % params['current'])
        # print('voltage %f' % params['voltage'])
        print('I0: %f' % unknowns['I0'])
        # print('resid: %f' % resids['I0'])
        # print('power_input: %f' % params['power_input'])


class MotorGroup(Group):
    def __init__(self):
        super(MotorGroup, self).__init__()

        self.add('Motor', Motor(), promotes=['*'])
        self.add('MotorSize', MotorSize(), promotes=['*'])
        self.add('MotorSolver', MotorBalance(), promotes=['power_input', 'current', 'voltage'])
        self.connect('MotorSolver.I0', 'I0')

        self.nl_solver = Newton()
        self.nl_solver.options['maxiter'] = 1000
        self.nl_solver.options['atol'] = 5.0e-5

        self.ln_solver = ScipyGMRES()
        self.ln_solver.options['maxiter'] = 100


class MotorSize(Component):
    def __init__(self):
        super(MotorSize, self).__init__()
        self.deriv_options['type'] = 'fd'

        self.add_param('speed', val=2000.0, desc='operating speed', units='RPM')
        self.add_param('L_D_RATIO',
                       val=0.822727,
                       desc='Length to diameter ratio of motor',
                       units='unitless')
        self.add_param('max_rpm',
                       val=3500.0,
                       desc='max rpm of motor',
                       units='RPM')
        self.add_param('design_power',
                       val=0.394 * 746,
                       desc='Design value of motor',
                       units='W')
        self.add_param('n_phases',
                       val=3.0,
                       desc='number of motor power phases',
                       units='unitless')
        self.add_param('KAPPA',
                       val=1 / 1.75,
                       desc='Base speed/max speed',
                       units='unitless')
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
        self.add_output('torque', val=1000.0, desc='torque at max_rpm', units='N*m')
        self.add_output('wbase', val=3000.0, desc='base speed', units='rad/s')
        self.add_output('D2L', val=1.0, desc='motor volume', units='mm^3')
        self.add_output('power_mech', 0.0, units='W')
        self.add_output('w_operating', 0.0, units='rad/s')
        self.add_output('P_iron_loss', 0.0, units='W')
        self.add_output('R_calc', 0.0, units='ohm')


    def solve_nonlinear(self, params, unknowns, resids):
        # calc max torque, rotational velocity
        w_max = params['max_rpm'] * 2.0 * np.pi / 60.0  # rad/sec
        unknowns['wbase'] = params['KAPPA'] * w_max
        unknowns['tmax'] = params['design_power'] / unknowns['wbase']
        unknowns['torque'] = params['design_power'] / w_max
        # calc sizing
        unknowns['D2L'] = 293722.0 * np.power(unknowns['tmax'], 0.7592)  # mm^3

        unknowns['d_base'] = np.power(unknowns['D2L'] / params['L_D_RATIO'], 1.0 / 3.0) / 1000.0  # m
        unknowns['l_base'] = unknowns['d_base'] * params['L_D_RATIO']  # m

        # calc mass
        unknowns['mass'] = 0.0000070646 * np.power(unknowns['D2L'], 0.9386912061)  # kg, relation in GT paper (Figure 6)
        unknowns['w_operating'] = params['speed'] * 2 * np.pi / 60.0
        unknowns['power_mech'] = unknowns['w_operating'] * unknowns['torque']
        unknowns['P_iron_loss'] = self.calculate_iron_loss(unknowns['d_base'], params['speed'], unknowns['l_base'])
        unknowns['R_calc'] = self.calculate_copper_loss(unknowns['d_base'], params['imax'], params['n_phases'])

    def calculate_copper_loss(self, d_base, imax, n_phase):
        # D-axis resistance per motor phase at very high-speed (short-cruit)
        Rd = 0.0
        # calc static loading factor from GT paper
        As = 688.7 * imax

        # number of coil turns for 3 phase motor
        n_coil_turns = As * np.pi * d_base / imax / n_phase / 2.0
        resistance_per_km_per_turn = 48.8387296964863 * np.power(imax, -1.00112597971171)
        winding_len = d_base * 3.14159
        resistance_per_turn = resistance_per_km_per_turn * winding_len / 1000.
        return resistance_per_turn * n_coil_turns * n_phase

    def calculate_iron_loss(self, d_base, speed, l_base):
        pole_pairs = 6.0
        # core_radius_ratio = 0.7
        core_density = 7650.0
        core_radius_ratio = 0
        Kh = 0.0275
        Kc = 0.0000183
        Ke = 0.0000277
        # Bp = 1.5
        Bp = 1.22

        # iron losses
        freq = speed * pole_pairs / 60.0
        volume_iron = np.pi * l_base * np.power(d_base / 2.0, 2.0) * (
            1.0 - np.power(core_radius_ratio, 2.0))
        iron_core_mass = core_density * volume_iron
        P_iron_loss = (Kh * np.power(Bp, 2.0) * freq + Kc*np.power(Bp*freq, 2.0) + Ke*np.power(Bp*freq, 1.5)) * iron_core_mass
        return P_iron_loss

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
        Resistance of Stator in ohm. Default value is 0.0
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
        self.deriv_options['type'] = 'fd'
        self.add_param('imax',
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
        self.add_param('l_base',
                       val=0.0,
                       desc='motor length',
                       units='m')
        self.add_param('d_base',
                       val=0.48,
                       desc='Base 8000hp diameter for scaling purposes',
                       units='m')
        self.add_param('pole_pairs',
                       val=6.0,
                       desc='number of motor pole_pairs',
                       units='unitless')
        self.add_param('torque', val=1.0, desc='torque at max_rpm', units='N*m')
        self.add_param('w_operating',
                       1.0,
                       desc='operating speed of motor',
                       units='rad/s')
        self.add_param('R_calc', 0.0, units='ohm')

        self.add_param('power_mech', 0.0, units='W')
        self.add_param('P_iron_loss', 0.0, units='W')
        self.add_param('tmax', val=0.0, desc='MaxTorque', units='N*m')
        self.add_output('current', val=2.0, desc='total current through motor', units='A')
        self.add_output('phase_current', val=1.0, desc='total phase current through motor', units='A')
        self.add_output('voltage', val=500.0, desc='total voltage through motor', units='V')
        self.add_output('phase_voltage', val=1.0, desc='total phase voltage through motor', units='V')
        self.add_output('frequency', val=60.0, desc='frequency of motor controller', units='Hz')
        self.add_output('power_input', 1.0, desc='total power input into motor', units='W')

    def calculate_windage_loss(self, params):
        # # calculate windage losses
        # w_operating = speed * 2 * np.pi / 60.0
        # Re = np.power(d_base, 2.0) / 4.0 * woperating / 2.075e-5 * 0.05
        # c_friction = 0.01
        # diff = 1.0
        pass
        # # TODO no nested loop
        # # while (abs(diff) > 0.001):
        # #     Cd = 1.0 / np.power(2.04+1.768*np.log(Re*np.power(c_friction, 0.5)), 2.0)
        # #     diff = (Cd - c_friction) / Cd
        # #     c_friction = Cd
        #
        # # calculate cylinder loss
        # P_windage_cylinder_loss = c_friction * np.pi * np.power(unknowns['woperating'], 3.0) * np.power(
        #     d_base / 2.0, 4.0) * l_base * 1.2041
        # # calculate cylinder face loss
        # Re_r = np.power(d_base, 2.0) / 4.0 * unknowns['woperating'] / 2.075e-5
        # c_disk_friction = 0.08 / np.power(0.05, 0.167) / np.power(Re, 0.25)
        # P_windage_face_loss = 0.5 * c_friction * 1.2041 * np.power(unknowns['woperating'], 3.0) * np.power(
        #     d_base / 2.0, 5.0) * (1.0 - np.power(core_radius_ratio, 5.0))
        # # P_windage_total_loss = P_windage_face_loss + P_windage_face_loss
        # P_windage_total_loss = 0

    def solve_nonlinear(self, params, unknowns, resids):

        # P_windage_loss = self.calculate_windage_loss()
        P_windage_loss = 0

        k_v = (params['imax'] - params['I0']) / params['tmax'] * 30.0 / np.pi
        k_t = 30.0 / np.pi * 1.0 / k_v


        # Calculating phase current, phase voltage, frequency, and phase
        unknowns['current'] = params['I0'] + params['torque'] / k_t
        P_copper_loss = np.power(unknowns['current'], 2.0) * params['R_calc']
        unknowns['power_input'] = params['power_mech'] + P_windage_loss + params['P_iron_loss'] + P_copper_loss
        unknowns['current'] = params['I0'] + params['torque'] / k_t

        unknowns['phase_current'] = unknowns['current'] / params['n_phases']
        unknowns['voltage'] = unknowns['current']*params['R_calc'] + params['w_operating'] / (k_v * np.pi / 30.0)

        # print('voltage % f' % unknowns['voltage'])
        # print()
        unknowns['phase_voltage'] = unknowns['voltage'] * np.sqrt(3.0 / 2.0)
        unknowns['frequency'] = params['w_operating'] / np.pi * params['pole_pairs'] / 60.0

        # print('copper_loss %f' % P_copper_loss)
        # print("R_calc %f" % R_calc)
        # print("woper %f" % params['w_operating'])
        # print('k_t %f' % k_t)
        # print('k_v %f' % k_v)

        # print("frequency: %f " % unknowns['frequency'])
        # # print('I0: %f' % params['I0'])

if __name__ == '__main__':
    from openmdao.api import SqliteRecorder
    from os import remove
    from sqlitedict import SqliteDict
    from openmdao.api import view_tree

    from pprint import pprint

    prob = Problem()
    prob.root = MotorGroup()
    # prob.root.add('Indep_I0', IndepVarComp('I0_input', 0.0))
    prob.root.add('init_vars', IndepVarComp('imax', 42.0, units='A'), promotes=['imax'])

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
