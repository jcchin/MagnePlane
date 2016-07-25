from openmdao.api import Group, Problem
from sqlitedict import SqliteDict

from hyperloop.Python.pod.drivetrain.battery import Battery
from hyperloop.Python.pod.drivetrain.electric_motor import MotorGroup
from hyperloop.Python.pod.drivetrain.inverter import Inverter

import numpy as np
import matplotlib.pylab as plt

class Drivetrain(Group):

    # TODO improve these comments...
    """The `Drivetrain` group represents a `Group` of an electric motor, inverter and battery
    in an OpenMDAO model.

    Models the performance and behavior of a combined electric motor, battery, and inverter
    following previous work from [1]_

    Components
    ----------
    Motor : ElectricMotor
        Represents a BLDC electric motor
    Inverter : Inverter
        Represents an Inverter
    Battery : Battery
        Represents a Battery

    Params
    ------
    design_torque : float
        design torque at max rpm (N*m)
    design_power : float
        desired design value for motor power (W)
    motor_max_current : float
        max motor phase current (A)
    motor_LD_ratio : float
        length to diameter ratio of motor (unitless)
    motor_oversize_factor : float
        scales peak motor power by this figure
    inverter_efficiency : float
        power out / power in (W)
    des_time : float
        time until design power point (h)
    time_of_flight : float
        total mission time (h)
    battery_cross_section_area : float
        cross_sectional area of battery used to compute length (cm^2)

    Outputs
    -------
    battery_mass : float
        total mass of cells in battery configuration (kg)
    battery_volume : float
        total volume of cells in battery configuration (cm^3)
    battery_cost : float
        total cost of battery cells in (USD)
    battery_length : float
        length of battery (cm)
    motor_volume : float
        D^2*L parameter which is proportional to Torque (mm^3)
    motor_diameter : float
        motor diameter (m)
    motor_mass : float
        mass of motor (kg)
    motor_length : float
        motor length (m)
    motor_power_input : float
        total required power input into motor (W)

    References
    ----------
    .. [1] Gladin, Ali, Collins, "Conceptual Modeling of Electric and Hybrid-Electric Propulsion for UAS Applications"
       Georgia Tech, 2015

    """

    def __init__(self):
        super(Drivetrain, self).__init__()

        self.deriv_options['type'] = 'fd'

        self.add('motor', MotorGroup(), promotes=['motor_power_input', 'motor_volume',
                                                  'motor_diameter', 'motor_mass', 'motor_length',
                                                  'design_torque', 'design_power',
                                                  'motor_max_current', 'motor_LD_ratio', 'motor_oversize_factor'])
        self.add('inverter', Inverter(), promotes=['inverter_efficiency'])
        self.add('battery', Battery(), promotes=['des_time', 'time_of_flight', 'battery_volume', 'battery_mass',
                                                 'battery_cost', 'battery_cross_section_area', 'battery_length'])

        # connect motor outputs to inverter inputs
        self.connect('motor.frequency', 'inverter.output_frequency')
        self.connect('motor.phase_voltage', 'inverter.output_voltage')
        self.connect('motor.phase_voltage', 'inverter.input_voltage')
        # TODO VERIFY THIS, does this make sense to force inverter to have
        # equal input/output
        self.connect('motor.phase_current', 'inverter.output_current')

        # connect inverter outputs to Battery inputs
        self.connect('inverter.input_current', 'battery.des_current')
        self.connect('inverter.input_power', 'battery.des_power')


if __name__ == '__main__':
    from openmdao.api import SqliteRecorder
    from pprint import pprint
    from os import remove

    prob = Problem()
    prob.root = Drivetrain()

    prob.setup()

    # setup ElectricMotor
    prob['motor_max_current'] = 800.0
    prob['motor_LD_ratio'] = 0.83
    prob['design_power'] = -100 * 746
    prob['design_torque'] = -10.0
    prob['motor_oversize_factor'] = 1.0
    prob['motor.idp1.n_phases'] = 3.0
    prob['motor.motor_size.kappa'] = 1 / 1.75
    prob['motor.idp2.pole_pairs'] = 6.0
    prob['motor.motor_size.core_radius_ratio'] = 0.0

    # setup inverter
    prob['inverter_efficiency'] = 1.0

    # setup battery
    prob['des_time'] = 1.0
    prob['time_of_flight'] = 2.0
    prob['battery.q_l'] = 0.1
    prob['battery.e_full'] = 1.4
    prob['battery.e_nom'] = 1.2
    prob['battery.e_exp'] = 1.27
    prob['battery.q_n'] = 6.8
    prob['battery.t_exp'] = 1.0
    prob['battery.t_nom'] = 4.3
    prob['battery.r'] = 0.0046
    prob['battery.cell_mass'] = 170.0
    prob['battery.cell_height'] = 61.0
    prob['battery.cell_diameter'] = 33.0

    prob.root.list_connections()

    power = np.linspace(-100,-50,num = 100)*746
    mass = np.zeros((1, len(power)))

    #for i in range(len(power)):
    #    prob['design_power'] = power[i]
    #    prob.run()
    #    mass[0, i] = prob['battery_mass']

    prob.run()
    plt.plot(power, mass[0, :])
    plt.show()
    #prob.run()

    print('battery_mass %f' % prob['battery_mass'])
    print(' %f' % prob['battery_volume'])
    print(' %f' % prob['battery_cost'])
    print('battery length %f' % prob['battery_length'])
    print(' %f' % prob['motor_volume'])
    print(' %f' % prob['motor_diameter'])
    print('motor mass %f' % prob['motor_mass'])
    print('motor length %f' % prob['motor_length'])
    print(' %f' % prob['motor_power_input'])
    