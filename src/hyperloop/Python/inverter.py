import numpy as np
from openmdao.api import Component


class Inverter(Component):
    def __init__(self):
        super(Inverter, self).__init__()

        self.add_param('efficiency', 1.0, desc='power out / power in')
        self.add_param('output_voltage',
                       120.0,
                       desc='Amplitude of AC output voltage',
                       units='V')
        self.add_param('output_current',
                       2.0,
                       desc='Amplitude of AC output current',
                       units='A')
        self.add_param('output_frequency',
                       60.0,
                       desc='Frequency of AC output',
                       units='Hz')
        self.add_param('input_voltage',
                       20.0,
                       desc='Amplitude of DC input voltage',
                       units='V')


        self.add_output('input_current',
                        0.48,
                        desc='Amplitude of DC input current',
                        units='A')
        self.add_output('input_power', 10.0, units='W')

    def solve_nonlinear(self, params, unknowns, resids):
        output_power = params['output_voltage'] * params[
            'output_current'] * 3.0 * np.sqrt(2.0 / 3.0)

        # TODO perform efficiency lookup
        unknowns['input_power'] = output_power / params['efficiency']

        unknowns['input_current'] = unknowns['input_power'] / params['input_voltage']
