"""
Models the equivalent circuit model of a brushless DC (BLDC) motor to perform motor sizing.
Calculates Phase Current, Phase Voltage, Frequency, motor size, and Weight.
"""
from __future__ import print_function

import numpy as np
from openmdao.api import Component, Problem, Group, Newton, IndepVarComp, ScipyGMRES


class MotorBalance(Component):
    """Creates an implicit connection used to balance conservation of energy
    across the motor by computing the residual of power_in - power_out.

    Params
    ------
    motor_power_input : float
        total required power input into motor (W)
    current : float
        current through motor (A)
    voltage : float
        voltage across motor (V)

    Unknowns
    --------
    I0 : float
        motor No-load Current (A)
    """

    def __init__(self):
        super(MotorBalance, self).__init__()
        self.deriv_options['type'] = 'cs'

        self.add_state('I0', val=40.0, desc='motor no load current', units='A')
        self.add_param('motor_power_input',
                       0.0,
                       desc='total power input into motor',
                       units='W')
        self.add_param('current',
                       val=2.0,
                       desc='total current through motor',
                       units='A')
        self.add_param('voltage',
                       val=500.0,
                       desc='total voltage through motor',
                       units='V')

    def solve_nonlinear(self, params, unknowns, resids):
        pass

    def apply_nonlinear(self, params, unknowns, resids):
        resids['I0'] = (
            params['current'] * params['voltage'] - params['motor_power_input'])

class MotorGroup(Group):
    def __init__(self):
        """MotorGroup represents a BLDC motor in an OpenMDAO model which can calculate
        size, mass, and various performance characteristics of a BLDC motor based
        on input paramters

        Components
        ----------
        motor : Motor
            Calculates the electrical characteristics of the motor
        motor_size : MotorSize
            Calculates the size, mass, and performance characteristics of the motor
        motor_balance : MotorBalance
            Calculates the residual in the conservation of energy equation between input
            power and total power used by the motor from mechanical output and additional
            losses

        Params
        ------
        design_power : float
            desired design value for motor power (W)
        design_torque : float
            desired torque at max rpm (N*m)
        motor_max_current : float
            max motor phase current (A)
        motor_LD_ratio : float
            length to diameter ratio of motor (unitless)
        motor_oversize_factor : float
            scales peak motor power by this figure (unitless)

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
        motor_power_input : float
            total required power input into motor (W)
        motor_volume : float
            D^2*L parameter which is proportional to Torque (mm^3)
        motor_diameter : float
            diameter of motor (m)
        motor_mass : float
            mass of motor (kg)
        motor_length : float
            motor length (m)
        motor_power_input : float
            total required power input into motor (W)
        """
        super(MotorGroup, self).__init__()

        self.add('motor',
                 Motor(),
                 promotes=['design_torque', 'motor_max_current',
                           'phase_current', 'phase_voltage', 'current',
                           'voltage', 'frequency', 'motor_power_input'])
        self.add('motor_size',
                 MotorSize(),
                 promotes=['motor_mass', 'design_torque', 'design_power',
                            'motor_max_current', 'motor_length', 'motor_diameter', 'motor_volume',
                           'motor_LD_ratio', 'motor_oversize_factor'])
        self.add('motor_balance',
                 MotorBalance(),
                 promotes=['current', 'voltage', 'motor_power_input'])

        self.connect('motor_size.max_torque', 'motor.max_torque')

        self.add('idp1',
                 IndepVarComp('n_phases', 3.0,
                              units='unitless'))
        self.connect('idp1.n_phases', ['motor_size.n_phases', 'motor.n_phases'])

        self.add('idp2',
                 IndepVarComp('pole_pairs', 6.0,
                              units='unitless'))

        self.connect('idp2.pole_pairs',
                     ['motor_size.pole_pairs', 'motor.pole_pairs'])

        self.add('idp3',
                 IndepVarComp('motor_max_current', 42.0,
                              units='A'),
                 promotes=['motor_max_current'])

        # self.add('idp4',
        #          IndepVarComp('design_torque', 1.0,
        #                       units='N*m'),
        #          promotes=['design_torque'])

        self.connect('motor_size.power_iron_loss', 'motor.power_iron_loss')
        self.connect('motor_size.power_mech', 'motor.power_mech')
        self.connect('motor_size.power_windage_loss',
                     'motor.power_windage_loss')
        self.connect('motor_size.w_operating', 'motor.w_operating')
        self.connect('motor_size.winding_resistance',
                     'motor.winding_resistance')

        self.connect('motor_balance.I0', 'motor.I0')

        self.nl_solver = Newton()
        self.nl_solver.options['maxiter'] = 1000
        self.nl_solver.options['atol'] = 0.0001

        self.ln_solver = ScipyGMRES()
        self.ln_solver.options['maxiter'] = 100


class MotorSize(Component):
    """MotorSize models the size of a BLDC motor based on a set of input paramters
    using data from existing commerical BLDC motors and work done by ([1]_)


    Parameters
    ----------
    design_power : float
        desired design value for motor power (W)
    design_torque : float
        desired torque at max rpm (N*m)
    motor_max_current : float
        max motor phase current (A)
    n_phases : float
        number of motor phases (unitless)
    motor_oversize_factor : float
        scales peak motor power by this figure (unitless)
    kappa : float
        ratio of base speed to max speed (unitless)
    pole_pairs : float
        number of motor pole pairs (unitless)
    motor_LD_ratio : float
        length to diameter ratio of motor (unitless)
    core_radius_ratio : float
        ratio of inner diamter to outer diameter of core (unitless)

    Outputs
    -------
    motor_volume : float
        D^2*L parameter which is proportional to Torque (mm^3)
    motor_diameter : float
        diameter of motor winding (m)
    motor_mass :float
        mass of motor (kg)
    motor_length : float
        length of motor (m)
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
    w_operating : float
        operating speed of motor (rad/s)

    References
    ----------
    .. [1] "J. Gladin, K. Ali, K. Collins, "Conceptual Modeling of Electric and Hybrid-Electric
           Propulsion for UAS Applications," Georgia Tech, 2015.
    """

    def __init__(self):
        """Initalizes the motor_size component to its default values"""

        super(MotorSize, self).__init__()
        self.deriv_options['type'] = 'cs'

        self.add_param('motor_max_current',
                       val=42.0,
                       desc='max motor phase current',
                       units='A')
        self.add_param('motor_LD_ratio',
                       val=0.822727,
                       desc='length to diameter ratio of motor',
                       units='unitless')
        self.add_param('core_radius_ratio',
                       0.0,
                       desc='ratio of inner diamter to outer diameter of core',
                       units='unitless')
        self.add_param('pole_pairs',
                       val=6.0,
                       desc='number of motor pole pairs',
                       units='unitless')
        self.add_param('design_power',
                       val=0.394 * 746,
                       desc='desired design value for motor power',
                       units='W')
        self.add_param('motor_oversize_factor',
                       1.0,
                       desc='scales peak motor power by this figure',
                       units='unitless')
        self.add_param('design_torque',
                        val=1.0,
                        desc='torque at max rpm',
                        units='N*m')
        self.add_param('n_phases',
                       val=3.0,
                       desc='number of motor power phases',
                       units='unitless')
        self.add_param('kappa',
                       val=1 / 1.75,
                       desc='ratio of base speed to max speed',
                       units='unitless')
        self.add_output('motor_diameter',
                        val=0.48,
                        desc='diameter of motor winding',
                        units='m')
        self.add_output('motor_length', val=0.4, desc='motor length', units='m')
        self.add_output('motor_mass', val=0.0, desc='mass of motor', units='kg')
        self.add_output('max_torque',
                        val=0.0,
                        desc='maximum possible torque for motor',
                        units='N*m')
        self.add_output('w_base',
                        val=3000.0,
                        desc=' base speed of motor ',
                        units='rad/s')
        self.add_output(
            'motor_volume',
            val=1.0,
            desc='D-squared*L parameter which is proportional to Torque',
            units='mm^3')
        self.add_output('power_mech',
                        0.0,
                        desc='mechanical power output of motor',
                        units='W')
        self.add_output('w_operating',
                        0.0,
                        desc='operating speed of motor',
                        units='rad/s')
        self.add_output('power_iron_loss',
                        0.0,
                        desc='total power loss due to iron core',
                        units='W')
        self.add_output('winding_resistance',
                        0.0,
                        desc='total resistance of copper winding',
                        units='ohm')
        self.add_output('power_windage_loss',
                        0.0,
                        desc='friction loss from motor operation',
                        units='W')

    def solve_nonlinear(self, params, unknowns, resids):
        """Runs the MotorSize component and sets its respective outputs to their calculated results
        in the unknowns `VecWrapper`.

        Args
        ----------
        params : `VecWrapper`
            `VecWrapper` containing parameters

        unknowns : `VecWrapper`
            `VecWrapper` containing outputs and states

        resids : `VecWrapper`
            `VecWrapper` containing residuals

        """
        design_power = params['design_power'] * params['motor_oversize_factor']
        # calc max torque, rotational velocity, power
        # w_max = params['max_rpm'] * 2.0 * np.pi / 60.0  # rad/sec
        unknowns['w_operating'] = design_power / params['design_torque'] # operating at maximum speed
        unknowns['w_base'] = params['kappa'] * unknowns['w_operating']
        unknowns['max_torque'] = design_power / unknowns['w_base']
        # unknowns['torque'] = design_power / w_max

        # unknowns['w_operating'] = params['speed'] * 2 * np.pi / 60.0
        unknowns['power_mech'] = unknowns['w_operating'] * params['design_torque']

        # calc size
        # print('max_torque %f' % unknowns['max_torque'])
        unknowns['motor_volume'] = 293722.0 * np.power(unknowns['max_torque'],
                                              0.7592)  # mm^3
        unknowns['motor_diameter'] = np.power(unknowns['motor_volume'] / params['motor_LD_ratio'],
                                      1.0 / 3.0) / 1000.0  # m
        unknowns['motor_length'] = unknowns['motor_diameter'] * params['motor_LD_ratio']  # m

        unknowns['motor_mass'] = 0.0000070646 * np.power(
            unknowns['motor_volume'],
            0.9386912061)  # kg, relation in GT paper (Figure 6)

        # calc loss parameters
        unknowns['power_iron_loss'] = self.calculate_iron_loss(
            unknowns['motor_diameter'], unknowns['w_operating'], unknowns['motor_length'],
            params['core_radius_ratio'], params['pole_pairs'])
        unknowns['winding_resistance'] = self.calculate_copper_loss(
            unknowns['motor_diameter'], params['motor_max_current'], params['n_phases'])
        unknowns['power_windage_loss'] = self.calculate_windage_loss(
            unknowns['w_operating'], unknowns['motor_diameter'], unknowns['motor_length'])


    def calculate_windage_loss(self, w_operating, motor_diameter, motor_length):
        """Calculates the windage or frictional losses of a BLDC motor with
        dimensions given by `motor_length` and `motor_diameter` operating at motor_speed `w_operating`.

        Args
        ----
        w_operating : float
            operating speed of motor (rad/s)
        motor_diameter : float
            diameter of motor winding (m)
        motor_length : float
            motor length (m)

        Returns
        -------
        float
            the total windage losses of the motor (W)
        """
        return 0

    #      # calc Reynolds number losses
    #      Re = np.power(diameter, 2.0) / 4.0 * w_operating / 2.075e-5 * 0.05
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
    #          diameter / 2.0, 4.0) * motor_length * 1.2041
    #      # calculate cylinder face loss
    #      Re_r = np.power(diameter, 2.0) / 4.0 * w_operating / 2.075e-5
    #      c_disk_friction = 0.08 / np.power(0.05, 0.167) / np.power(Re, 0.25)
    #      P_windage_face_loss = 0.5 * c_friction * 1.2041 * np.power(w_operating, 3.0) * np.power(
    #          diameter / 2.0, 5.0) * (1.0 - np.power(core_radius_ratio, 5.0))
    #      # P_windage_total_loss = P_windage_face_loss + P_windage_face_loss
    #      P_windage_total_loss = 0

    def calculate_copper_loss(self, motor_diameter, motor_max_current, n_phases):
        """Calculates the resistive losses in the copper winding of a BLDC motor
        operating at `motor_max_current` and `n_phases` with dimension specified by
        `motor_diameter`.

        Parameters
        ----------
        self
        motor_diameter : float
            dameter of motor winding (m)
        motor_max_current : float
            max motor phase current (A)
        n_phases : float
            number of motor phases (unitless)

        Returns
        -------
        float
            the total resistive losses of the copper winding (W)

        """
        # D-axis resistance per motor phase at very high-speed (short-cruit)
        Rd = 0.0

        # calc static loading factor from GT paper
        As = 688.7 * motor_max_current

        # calc total resistance in winding
        n_coil_turns = As * np.pi * motor_diameter / motor_max_current / n_phases / 2.0
        resistance_per_km_per_turn = 48.8387296964863 * np.power(
            motor_max_current, -1.00112597971171)
        winding_len = motor_diameter * 3.14159
        resistance_per_turn = resistance_per_km_per_turn * winding_len / 1000.
        return resistance_per_turn * n_coil_turns * n_phases

    def calculate_iron_loss(self, motor_diameter, motor_speed, motor_length, core_radius_ratio,
                            pole_pairs):
        """Calculates the iron core magnetic losses of a BLDC motor with
        dimensions given by `motor_length` and `motor_diameter` operating at speed `motor_speed`.

        Args
        ----
        motor_diameter : float
            diameter of motor winding (m)
        speed : float
            desired output shaft mechanical motor_speed (RPM)
        motor_length : float
            motor length (m)
        core_radius_ratio : float
            ratio of inner diameter to outer diameter of core (unitless)

        Returns
        -------
        float
            the total iron core losses of the motor (W)
        """

        stator_core_density = 7650.0  # kg/m^3
        # hysteresis loss constant
        Kh = 0.0275  # W/(kg T^2 Hz)
        # iron eddy loss constant
        Kc = 1.83e-5  # W/(kg T^2 Hz^2)
        # correction factor
        Ke = 2.77e-5  # W/(kg T^1.5 Hz^1.5)
        # stator magnetic flux density
        Bp = 1.22  # T
        # Bp = 1.5
        # iron losses
        freq = motor_speed / (2.0 * np.pi) * pole_pairs
        volume_iron = np.pi * motor_length * np.power(motor_diameter / 2.0, 2.0) * (
            1.0 - np.power(core_radius_ratio, 2.0))
        iron_core_mass = stator_core_density * volume_iron
        power_iron_loss = (Kh * np.power(Bp, 2.0) * freq + Kc * np.power(
            Bp * freq, 2.0) + Ke * np.power(Bp * freq, 1.5)) * iron_core_mass
        return power_iron_loss


class Motor(Component):
    """ Represents an electric motor which can calculate output current and voltage
    based on motor sizing parameters and losses. Used in conjunction with motor_balance
    to find the correct no load current for this motor

    Based on work done by Gladin et. al. ([1]_)

    Params
    ------
    w_operating : float
        operating speed of motor (rad/s)
    design_torque : float
        torque at max rpm (N*m)
    I0 : float
        motor No-load Current (A)
    motor_max_current : float
        max motor phase current (A)
    pole_pairs : float
        number of motor pole pairs (unitless)
    n_phases : float
        number of motor phases (unitless)
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

    Outputs
    -------
    current : float
        current through motor (A)
    voltage : float
        voltage across motor (V)
    phase_current : float
        phase current through motor (A)
    phase_voltage : float
        phase voltage across motor (V)
    frequency : float
        Frequency of electric output waveform (Hz)
    motor_power_input : float
        total required power input into motor (W)

    References
    ----------
    .. [1] "J. Gladin, K. Ali, K. Collins, "Conceptual Modeling of Electric and Hybrid-Electric
            Propulsion for UAS Applications," Georgia Tech, 2015.

    """

    def __init__(self):
        """Creates an instance of motor_size and initializes it to the default values below"""

        super(Motor, self).__init__()
        self.deriv_options['type'] = 'cs'
        self.add_param('motor_max_current',
                       val=42.0,
                       desc='max operating current',
                       units='A')
        self.add_param('I0', val=40.0, desc='motor no load current', units='A')
        self.add_param('n_phases',
                       val=3.0,
                       desc='number of motor power phases',
                       units='unitless')
        self.add_param('w_operating',
                       0.0,
                       desc='operating speed of motor',
                       units='rad/s')
        self.add_param('pole_pairs',
                       val=6.0,
                       desc='number of motor pole_pairs',
                       units='unitless')
        self.add_param('design_torque',
                       val=1.0,
                       desc='torque at max_rpm',
                       units='N*m')
        self.add_param('winding_resistance',
                       0.0,
                       desc='total resistance of copper winding',
                       units='ohm')
        self.add_param('power_windage_loss',
                       0.0,
                       desc='friction loss from motor operation',
                       units='W')
        self.add_param('power_mech',
                       0.0,
                       desc='mechanical power output of motor',
                       units='W')
        self.add_param('power_iron_loss',
                       0.0,
                       desc='total power loss due to iron core',
                       units='W')
        self.add_param('max_torque',
                       val=0.0,
                       desc='maximum possible torque for the motor',
                       units='N*m')

        self.add_output('current',
                        val=2.0,
                        desc='current through motor',
                        units='A')
        self.add_output('phase_current',
                        val=1.0,
                        desc='phase current through motor',
                        units='A')
        self.add_output('voltage',
                        val=500.0,
                        desc='voltage across motor',
                        units='V')
        self.add_output('phase_voltage',
                        val=1.0,
                        desc='phase voltage across motor',
                        units='V')
        self.add_output('frequency',
                        val=60.0,
                        desc='frequency of electric output waveform',
                        units='Hz')
        self.add_output('motor_power_input',
                        1.0,
                        desc='total required power input into motor',
                        units='W')

    def solve_nonlinear(self, params, unknowns, resids):
        # voltage constant
        k_v = (params['motor_max_current'] - params['I0']
               ) / params['max_torque'] * 30.0 / np.pi
        # torque constant
        k_t = 30.0 / np.pi * 1.0 / k_v

        # Calculating phase current, phase voltage, frequency, and phase
        unknowns['current'] = params['I0'] + params['design_torque'] / k_t
        power_copper_loss = np.power(unknowns['current'],
                                     2.0) * params['winding_resistance']
        unknowns['motor_power_input'] = params['power_mech'] + params[
            'power_windage_loss'] + params[
                'power_iron_loss'] + power_copper_loss

        unknowns['current'] = params['I0'] + params['design_torque'] / k_t
        unknowns['phase_current'] = unknowns['current'] / params['n_phases']

        unknowns['voltage'] = unknowns['current'] * params[
            'winding_resistance'] + params['w_operating'] / (k_v * np.pi / 30.0
                                                             )
        unknowns['phase_voltage'] = unknowns['voltage'] * np.sqrt(3.0 / 2.0)

        unknowns['frequency'] = params['w_operating'] / np.pi * params[
            'pole_pairs'] / 60.0


if __name__ == '__main__':

    prob = Problem()
    prob.root = MotorGroup()

    # rec = SqliteRecorder('drivetraindb')
    # rec.options['record_params'] = True
    # rec.options['record_metadata'] = True
    # prob.driver.add_recorder(rec)

    prob.setup(check=True)



    prob['motor_max_current'] = 450.0
    prob['motor_LD_ratio'] = 0.83
    prob['design_power'] = 110000
    prob['design_torque'] = 420.169
    prob['motor_oversize_factor'] = 1.0
    prob['idp1.n_phases'] = 3.0
    prob['motor_size.kappa'] = 0.5
    prob['idp2.pole_pairs'] = 6.0
    prob['motor_size.core_radius_ratio'] = 0.7

    prob.root.list_connections()
    prob.run()
    # prob.print_all_convergence()

    print("FINAL")
    print(prob['motor.I0'])
    print(prob['voltage'])
    print(prob['current'])
    print(prob['motor_mass'])
    print(prob['motor_length'])
    print(prob['motor_volume'])
    print(prob['motor_size.max_torque'])

    prob.cleanup()
