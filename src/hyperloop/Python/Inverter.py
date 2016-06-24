


import math, scipy
import numpy as np
from openmdao.core.component import Component
from openmdao.api import IndepVarComp, Component, Problem, Group, ScipyOptimizer, ExecComp, SqliteRecorder

class Inverter(Component):


    def __init__(self):
        self.add_param('Efficiency', 1.0, desc='power out / power in')
        self.add_param('OutputVoltage', 120.0, desc='Amplitude of AC output voltage', units='Volts')
        self.add_param('OutputCurrent', 2, desc='Amplitude of AC output current', units='Amps')
        self.add_param('InputVoltage', 500, desc='Amplitude of AC input voltage', units='Volts')
        self.add_param('OutputFrequency', 60, desc='Frequency of AC output', units='Hz')
        self.add_param('DesignPower', 8000, desc='Design output power', units='hp')

        self.add_output('InputCurrent', 0,48, desc='Amplitude of AC input current', units='Amps')
        self.add_output('OutputPower', value=12000000, units='Watts')

    def solve_nonlinear(self, params, unknowns, resids):
        output_power = params['OutputVoltage'] * params['OutputCurrent'] * 3.0 * np.sqrt(2.0/3.0)
        unknowns['OutputPower'] = output_power

        input_power = output_power / params['Efficiency']

        # negative sign because drawing current from cable
        # needed to keep bus voltage higher than inverter rather than lower
        unknowns['InputCurrent'] = -input_power / params['InputVoltage']