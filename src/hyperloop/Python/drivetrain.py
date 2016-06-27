from openmdao.api import ExecComp, IndepVarComp, Group, NLGaussSeidel, \
    ScipyGMRES, Problem, ScipyOptimizer
from hyperloop.Python.ElectricMotor import ElectricMotor
from hyperloop.Python.Inverter import Inverter
from hyperloop.Python.dc_transformer import DCTransformer
from hyperloop.Python.battery import Battery

class PMAD(Group):
    """ Group containing the Sellar MDA. This version uses the disciplines
    with derivatives."""

    def __init__(self):
        super(PMAD, self).__init__()

        self.add('InputVoltage', IndepVarComp('Voltage', 500.0))

        self.add('Motor', ElectricMotor())
        self.add('Inverter', Inverter())
        self.add('DCTransformer', DCTransformer())
        self.add('Battery', Battery())

        # connect ElectricMotor outputs to Inverter inputs
        self.connect('Motor.InputFrequency', 'Inverter.OutputFrequency')
        self.connect('Motor.PhaseVoltage', 'Inverter.OutputVoltage')
        self.connect('Motor.PhaseCurrent', 'Inverter.OutputCurrent')

        # connect Inverter outputs to DCTransformer inputs
        self.connect('Inverter.InputVoltage', 'DCTransformer.OutputVoltage')
        self.connect('Inverter.InputCurrent', 'DCTransformer.OutputCurrent')

        # connect Battery outputs to DCTransformer inputs
        self.connect('Battery.Voltage', 'DCTransformer.InputVoltage')
        self.connect('Battery.Current', 'DCTransformer.InputCurrent')






        # self.nl_solver = NLGaussSeidel()
        # self.nl_solver.options['atol'] = 1.0e-12
        #
        # self.ln_solver = ScipyGMRES()

if __name__ == '__main__':

top = Problem()
top.root = PMAD()

top.driver = ScipyOptimizer()
top.driver.options['optimizer'] = 'SLSQP'
top.driver.options['tol'] = 1.0e-8

top.driver.add_desvar('InputVoltage.Voltage', lower=0.0, upper=1000.0)

top.setup()

# Setting initial values for design variables
top['InputVoltage.Voltage'] = 200.0

top.run()
