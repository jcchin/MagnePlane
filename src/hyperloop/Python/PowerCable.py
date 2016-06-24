


import math, scipy
import numpy as np
from openmdao.core.component import Component
from openmdao.api import IndepVarComp, Component, Problem, Group, ScipyOptimizer, ExecComp, SqliteRecorder

class PowerCable(Component):


    def __init__(self):
        self.add_param('Resistance', 0.0, desc='resistance of Cable', units='Ohms')
        self.add_param('InputVoltage', 500.0, desc='voltage at side which is fixed (bus voltage)')
        self.add_param('Current', 2.0, desc='current through cable', units='Amps')

        self.add_output('OutputVoltage', 500.0, desc='Voltage at side which floats', units='Volts')


    def solve_nonlinear(self, params, unknowns, resids):
        unknowns['OutputVoltage'] = params['InputVoltage'] - params['Current'] * params['Resistance']
