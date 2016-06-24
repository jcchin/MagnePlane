from openmdao.api import Component

class DCTransformer(Component):

    def __init__(self):
        super(DCTransformer, self).__init__()

        self.add_param('Efficiency', 1.0, desc='power out / power in')
        self.add_param('OutputVoltage', 5000, desc='DC output voltage', units='Volts')
        self.add_param('OutputCurrent', 298, desc='DC output current', units='Amps')
        self.add_param('InputVoltage', 298, desc='DC input voltage', units='Volts')
        self.add_param('InputFreq', 60, desc='DC input frequency', units='Hz')
        self.add_param('OutputFreq', 60, desc='DC output frequency', units='Hz')
        self.add_param('DesignPower', 12000000, 'design output power')


        self.add_output('InputCurrent', 2, desc='DC input current', units='Amps')

    def solve_nonlinear(self, params, unknowns, resids):
