from openmdao.api import Group, Problem
from sqlitedict import SqliteDict

from hyperloop.Python.pod.drivetrain.battery import Battery
from hyperloop.Python.pod.drivetrain.electric_motor import MotorGroup
from hyperloop.Python.pod.drivetrain.inverter import Inverter


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
    motor.design_torque : float
    motor.design_power : float
        desired design value for motor power (W)
    motor.max_current : float
        max motor phase current (A)
    motor.speed : float
        desired output shaft mechanical speed (RPM)
    inverter.efficiency : float
        power out / power in (W)
    des_time : float
        time until design power point (h)
    time_of_flight : float
        total mission time (h)

    Outputs
    -------
    battery.mass : float
        total mass of cells in battery configuration (kg)
    battery.volume : float
        total volume of cells in battery configuration (cm^3)
    battery.cost : float
        total cost of battery cells in (USD)
    motor.power_input : float
        total required power input into motor
    motor.volume : float
        D^2*L parameter which is proportional to Torque (mm^3)
    motor.diameter : float
        motor diameter (m)
    motor.mass : float
        mass of motor (kg)
    motor.length : float
        motor length (m)
    motor.power_input : float
        total required power input into motor (W)

    References
    ----------
    .. [1] Gladin, Ali, Collins, "Conceptual Modeling of Electric and Hybrid-Electric Propulsion for UAS Applications"
       Georgia Tech, 2015

    """

    def __init__(self):
        super(Drivetrain, self).__init__()

        self.deriv_options['type'] = 'fd'

        self.add('motor', MotorGroup())
        self.add('inverter', Inverter())
        self.add('battery', Battery(), promotes=['des_time', 'time_of_flight'])

        # connect motor outputs to inverter inputs
        self.connect('motor.frequency', 'inverter.output_frequency')
        self.connect('motor.phase_voltage', 'inverter.output_voltage')
        self.connect('motor.phase_voltage', 'inverter.input_voltage')
        # TODO VERIFY THIS, does this make sense to force inverter to have
        # equal input/output  import pprintvoltage?
        self.connect('motor.phase_current', 'inverter.output_current')

        # connect inverter outputs to Battery inputs
        self.connect('inverter.input_current', 'battery.des_current')
        self.connect('inverter.input_power', 'battery.des_power')

        # self.nl_solver = NLGaussSeidel()
        # self.nl_solver.options['atol'] = 0.1
        # self.nl_solver.options['maxiter'] = 200
        # self.ln_solver = ScipyGMRES()
        # self.ln_solver = PetscKSP()


if __name__ == '__main__':
    from openmdao.api import SqliteRecorder
    from pprint import pprint
    from os import remove

    prob = Problem()
    prob.root = Drivetrain()
    rec = SqliteRecorder('drivetraindb')
    rec.options['record_params'] = True
    rec.options['record_metadata'] = True
    prob.driver.add_recorder(rec)

    rec = SqliteRecorder('drivetraindb')
    rec.options['record_params'] = True
    rec.options['record_metadata'] = True
    prob.driver.add_recorder(rec)

    prob.setup()

    # setup ElectricMotor
    prob['max_current'] = 42.0
    prob['speed'] = 1900.0
    prob['motor.motor_size.L_D_ratio'] = 0.83
    prob['max_rpm'] = 3500.0
    prob['design_power'] = 0.394 * 746
    prob['motor.n_phases'] = 3.0
    prob['motor.motor_size.kappa'] = 1 / 1.75
    prob['motor.pole_pairs'] = 6.0
    prob['motor.motor_size.core_radius_ratio'] = 0.0

    # setup inverter
    prob['inverter.efficiency'] = 1.0

    # setup battery
    prob['des_time'] = 1.0
    prob['time_of_flight'] = 2.0
    # prob['des_power'] = 7.0
    # prob['des_current'] =1.0
    prob['battery.q_l'] = 0.1
    prob['battery.e_full'] = 1.4
    prob['battery.e_nom'] = 1.2
    prob['battery.e_exp'] = 1.27
    prob['battery.q_n'] = 6.8
    prob['battery.t_exp'] = 1.0
    prob['battery.t_nom'] = 4.3
    prob['battery.r'] = 0.0046
    prob['battery.cell_mass'] = 170
    prob['battery.cell_height'] = 61.0
    prob['battery.cell_diameter'] = 33.0

    prob.root.list_connections()
    prob.run()

    print('ncells %f' % top['Battery'])

    db = SqliteDict('drivetraindb', 'openmdao')
    pprint(db.keys())
    data = db['rank0:Driver/1']
    pprint(data['Parameters'])
    print
    print
    pprint(data['Unknowns'])
    prob.cleanup()
    remove('drivetraindb')
