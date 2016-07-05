import numpy as np
from hyperloop.Python.pod.drivetrain.battery import Battery
from hyperloop.Python.pod.drivetrain.inverter import Inverter
from openmdao.api import Group, NLGaussSeidel, \
    ScipyGMRES, Problem

from Python.pod.drivetrain.electric_motor import ElectricMotor


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

        self.add('Motor', ElectricMotor())
        self.add('Inverter', Inverter())
        self.add('Battery', Battery(), promotes=['des_time', 'time_of_flight'])

        # connect ElectricMotor outputs to Inverter inputs
        self.connect('Motor.frequency', 'Inverter.output_frequency')
        self.connect('Motor.phase_voltage', 'Inverter.output_voltage')
        self.connect('Motor.phase_voltage', 'Inverter.input_voltage')
        # TODO VERIFY THIS, does this make sense to force inverter to have
        # equal input/output voltage?
        self.connect('Motor.phase_current', 'Inverter.output_current')

        # connect Inverter outputs to Battery inputs
        self.connect('Inverter.input_current', 'Battery.des_current')
        self.connect('Inverter.input_power', 'Battery.des_power')

        # TODO remove this, represented previous non-converging cycle
        # connect Battery outputs to Inverter inputs
        # self.connect('Battery.output_voltage', 'Inverter.input_voltage')

        self.nl_solver = NLGaussSeidel()
        self.nl_solver.options['atol'] = 0.1
        self.nl_solver.options['maxiter'] = 200
        self.ln_solver = ScipyGMRES()
        # self.ln_solver = PetscKSP()


if __name__ == '__main__':
    from openmdao.api import SqliteRecorder

    top = Problem()
    top.root = Drivetrain()

    rec = SqliteRecorder('drivetraindb')
    rec.options['record_params'] = True
    rec.options['record_metadata'] = True
    top.driver.add_recorder(rec)

    top.setup()

    # top['Battery.n_parallel'] = 0
    # # Setting initial values for design variables
    # top['InputVoltage.Voltage'] = 200.0

    top.run()
    print
    print('bat out v: %f ' % top['Battery.output_voltage'])
    print('invert in v: %f' % top['Inverter.input_voltage'])
    print('mot in volt: %f' % top['Motor.phase_voltage'])
    print('invert out volt: %f' % top['Inverter.output_voltage'])
    print
    print('invert in pow %f' % top['Inverter.input_power'])
    in_pow = top['Inverter.input_voltage'] * top['Inverter.input_current']
    print('calc inverter in pow %f' % in_pow)
    output_power = top['Inverter.output_voltage'] * top[
        'Inverter.output_current'] * 3.0 * np.sqrt(2.0 / 3.0)
    print('calc output pow %f' % output_power)
    print('bat des pow: %f' % top['Battery.des_power'])
    print
    print('Inv in cur %f' % top['Inverter.input_current'])
    # print('ncells %f' % top['Battery'])
    #
    # db = sqlitedict.SqliteDict('drivetraindb', 'openmdao')
    # pprint(db.keys())
    # data = db['rank0:Driver/1']
    # pprint(data['Parameters'])
    # print
    # print
    # pprint(data['Unknowns'])
    # top.cleanup()
    # remove('drivetraindb')
