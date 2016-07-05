from openmdao.api import Component


class DCTransformer(Component):
    def __init__(self):
        super(DCTransformer, self).__init__()

        self.add_param('efficiency', 1.0, desc='power out / power in')
        self.add_param('output_voltage',
                       5000,
                       desc='DC output voltage',
                       units='Volts')
        self.add_param('output_current',
                       298,
                       desc='DC output current',
                       units='Amps')
        self.add_param('input_voltage',
                       298,
                       desc='DC input voltage',
                       units='Volts')
        # self.add_param('DesignPower', 12000000, units='design output power')
        self.add_param('des_current',
                       2,
                       desc='DC input current',
                       units='Amps')

    def solve_nonlinear(self, params, unknowns, resids):
        input_power = params['input_voltage'] * params['input_current']
        output_power = params['output_voltage'] * params['output_current']

        # TODO perform efficiency calculation
