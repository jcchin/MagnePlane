import math, numpy, scipy
from openmdao.core.component import Component
from openmdao.api import IndepVarComp, Component, Problem, Group, ScipyOptimizer, ExecComp, SqliteRecorder
"""Notes
    ----
   Calculates parameters like Nparallel(number of cells in parallel), Nseries(Number of cells in series), Ncells(total no of cells) and C_Max(max rating)
   The calculated parameters are then use to estimate battery weight in BatteryWeight.py
Params
    ----
    comp_eff: float
        Compressor Efficiency in percent. Default value is 91.0.
    mass_flow: float
        Mass Flow in kJ/kg. Default value is 317.52
    h_in: float
        Heat in in kJ/kg. Default value is 0.0
    h_out: float
        Heat out in kJ/kg. Default value is 486.13
    comp_inlet_R: float
        Compressor Inlet Radius in m. Default value is 0.64


Outputs
    ----
    comp_weight : float
        Compressor Weight in kg. Default value is 1.0.

References
        ---- Micheal Tong Correlation used. NPSS data used. """


class Compressor_weight(Component):
    def __init__(self):
        super(Compressor_weight, self).__init__()

        self.add_param('comp_eff',
                       val=91.,
                       desc='Compressor Efficiency',
                       units='percent')
        self.add_param('mass_flow',
                       val=317.52,
                       desc='Mass Flow Rate',
                       units='kg/s')
        self.add_param('h_in', val=0., desc='Heat-in', units='kJ/kg')
        self.add_param('h_out', val=486.13, desc='Heat-out', units='kJ/kg')
        self.add_param('comp_inlet_R',
                       val=0.64,
                       desc='Compressor Inlet Radius',
                       units='m')

        self.add_output('comp_weight',
                        val=0.1,
                        desc='Compressor Weight',
                        units='kg')

    def solve_nonlinear(self, params, unknowns, resids):
        comp_eff = params['comp_eff']
        mass_flow = params['mass_flow']
        h_in = params['h_in']
        h_out = params['h_out']
        comp_inlet_R = params['comp_inlet_R']

        unknowns['comp_weight'] = 299.2167 * (comp_inlet_R**2) + 0.007418 * (
            (mass_flow * (h_out - h_in)) / (comp_eff / 100)) + 37.15


if __name__ == '__main__':
    # set up problem.
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', Compressor_weight())
    prob.setup()
    prob.root.list_connections()
    prob.run()

    print('Compressor Weight(kg) : %f' % prob['comp.comp_weight'])
