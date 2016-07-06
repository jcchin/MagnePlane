import numpy as np
from hyperloop.Python.pod.drivetrain.battery import Battery
from hyperloop.Python.pod.drivetrain.inverter import Inverter
from openmdao.api import Group, NLGaussSeidel, \
    ScipyGMRES, Problem, SqliteRecorder

from Python.pod.drivetrain.electric_motor import ElectricMotor
from sqlitedict import SqliteDict


# from openmdao.api.PP import PetscKSP.


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

    References
    ----------
    .. [1] Gladin, Ali, Collins, "Conceptual Modeling of Electric and Hybrid-Electric Propulsion for UAS Applications"
       Georgia Tech, 2015

    """

    def __init__(self):
        super(Drivetrain, self).__init__()

        self.add('motor', ElectricMotor(), promotes=['speed', 'design_power', 'max_rpm'])
        self.add('inverter', Inverter())
        self.add('battery', Battery(), promotes=['des_time', 'time_of_flight'])

        # connect ElectricMotor outputs to Inverter inputs
        self.connect('motor.frequency', 'inverter.output_frequency')
        self.connect('motor.phase_voltage', 'inverter.output_voltage')
        self.connect('motor.phase_voltage', 'inverter.input_voltage')
        # TODO VERIFY THIS, does this make sense to force inverter to have
        # equal input/output  import pprintvoltage?
        self.connect('motor.phase_current', 'inverter.output_current')

        # connect Inverter outputs to Battery inputs
        self.connect('inverter.input_current', 'battery.des_current')
        self.connect('inverter.input_power', 'battery.des_power')

        # TODO remove this, represented previous non-converging cycle
        # connect Battery outputs to Inverter inputs
        # self.connect('battery.output_voltage', 'inverter.input_voltage')

        self.nl_solver = NLGaussSeidel()
        self.nl_solver.options['atol'] = 0.1
        self.nl_solver.options['maxiter'] = 200
        self.ln_solver = ScipyGMRES()
        # self.ln_solver = PetscKSP()


if __name__ == '__main__':
    from openmdao.api import SqliteRecorder
    from pprint import pprint
    from os import remove

    top = Problem()
    top.root = Drivetrain()

    rec = SqliteRecorder('drivetraindb')
    rec.options['record_params'] = True
    rec.options['record_metadata'] = True
    top.driver.add_recorder(rec)

    top.setup()

    # top['battery.n_parallel'] = 0
    # # Setting initial values for design variables
    # top['InputVoltage.Voltage'] = 200.0

    top.run()
    print
    print('bat out v: %f ' % top['battery.output_voltage'])
    print('invert in v: %f' % top['inverter.input_voltage'])
    print('mot in volt: %f' % top['motor.phase_voltage'])
    print('invert out volt: %f' % top['inverter.output_voltage'])
    print
    print('invert in pow %f' % top['inverter.input_power'])
    in_pow = top['inverter.input_voltage'] * top['inverter.input_current']
    print('calc inverter in pow %f' % in_pow)
    output_power = top['inverter.output_voltage'] * top[
        'inverter.output_current'] * 3.0 * np.sqrt(2.0 / 3.0)
    print('calc output pow %f' % output_power)
    print('bat des pow: %f' % top['battery.des_power'])
    print
    print('Inv in cur %f' % top['inverter.input_current'])
    print('mot des pow %f' % top['design_power'])
    print('mot input cur %f' % top['motor.phase_current'])
    print('mot input volt %f' % top['motor.phase_voltage'])

    # print('ncells %f' % top['Battery'])

    db = SqliteDict('drivetraindb', 'openmdao')
    pprint(db.keys())
    data = db['rank0:Driver/1']
    pprint(data['Parameters'])
    print
    print
    pprint(data['Unknowns'])
    top.cleanup()
    remove('drivetraindb')
