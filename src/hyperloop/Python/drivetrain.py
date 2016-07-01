from openmdao.api import ExecComp, IndepVarComp, Group, NLGaussSeidel, \
    ScipyGMRES, Problem, ScipyOptimizer
from hyperloop.Python.electric_motor import ElectricMotor
from hyperloop.Python.inverter import Inverter
from hyperloop.Python.dc_transformer import DCTransformer
from hyperloop.Python.battery import Battery


class Drivetrain(Group):
    def __init__(self):
        super(Drivetrain, self).__init__()

        # self.add('InputVoltage', IndepVarComp('Voltage', 500.0))

        self.add('Motor', ElectricMotor())
        self.add('Inverter', Inverter())
        self.add('Battery', Battery(), promotes=['des_time', 'time_of_flight'])

        # connect ElectricMotor outputs to Inverter inputs
        self.connect('Motor.frequency', 'Inverter.output_frequency')
        self.connect('Motor.phase_voltage', 'Inverter.output_voltage')
        self.connect('Motor.phase_current', 'Inverter.output_current')

        # connect Inverter outputs to Battery inputs
        self.connect('Inverter.input_current', 'Battery.des_current')
        self.connect('Inverter.input_power', 'Battery.des_power')

        # connect Battery outputs to Inverter inputs
        # self.connect('Battery.output_voltage', 'Inverter.input_voltage')

        self.nl_solver = NLGaussSeidel()
        self.nl_solver.options['atol'] = 1.0e-12
        self.ln_solver = ScipyGMRES()


if __name__ == '__main__':
    from os import remove
    import sqlitedict
    from openmdao.api import SqliteRecorder
    from pprint import pprint



    top = Problem()
    top.root = Drivetrain()

    rec = SqliteRecorder('drivetraindb')
    rec.options['record_params'] = True
    rec.options['record_metadata'] = True
    top.driver.add_recorder(rec)


    top.setup()

    # # Setting initial values for design variables
    # top['InputVoltage.Voltage'] = 200.0

    top.run()

    db = sqlitedict.SqliteDict('drivetraindb', 'openmdao')
    data = db['rank0:Driver/1']

    pprint(data['Parameters'])
    print
    print
    pprint(data['Unknowns'])
    top.cleanup()
    remove('drivetraindb')

