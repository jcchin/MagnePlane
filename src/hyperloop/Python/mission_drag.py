from __future__ import print_function

from math import pi, sqrt
from openmdao.api import IndepVarComp, Component, Group, Problem, ExecComp

class MissionDrag(Component):
    '''...'''
    def __init__(self):
        super(MissionDrag, self).__init__()

        self.add_param('Cd', val = .2, desc = 'Drag Coefficient')
        self.add_param('S', val = 1.4, units = 'm^2', desc = 'Frontal Area')
        self.add_param('p_ambient', val = 850.0, units = 'Pa', desc = 'Ambient Pressure')
        self.add_param('T_ambient', val = 298.0, units = 'K', desc = 'Ambient Temperture')
        self.add_param('R', val = 287.0, units = 'J/(kg*K)', desc = 'Ideal Gas Constant')

        self.add_param('V', val = 335.0, units = 'm/s', desc = 'Velocity')

        self.add_output('D', val = 0.0, units = 'N', desc = 'Drag Force')

    def solve_nonlinear(self, params, unknowns, resids):

        #Calculate air density and drag force
        rho = params['p_ambient']/(params['R']*params['T_ambient'])
        unknowns['D'] = .5*rho*(params['V']**2)*params['S']

if __name__ == '__main__':

    top = Problem()
    root = top.root = Group()

    params = (
        ('Cd', .2),
        ('V', 335.0)
    )

    root.add('input_vars', IndepVarComp(params))
    root.add('p', MissionDrag())

    root.connect('input_vars.Cd', 'p.Cd')
    root.connect('input_vars.V', 'p.V')

    top.setup()
    top.run()

    print('\n')
    print('Drag Force = %f N' % top['p.D'])

