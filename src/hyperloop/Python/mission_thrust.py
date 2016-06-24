from __future__ import print_function

from math import pi, sqrt, sin
from openmdao.api import IndepVarComp, Component, Group, Problem, ExecComp

class MissionThrust(Component):
    '''...'''
    def __init__(self):
        super(MissionThrust, self).__init__()

        self.add_param('Cd', val = .2, desc = 'Drag Coefficient')
        self.add_param('S', val = 1.4, units = 'm**2', desc = 'Frontal Area')
        self.add_param('p_tube', val = 850.0, units = 'Pa', desc = 'Ambient Pressure')
        self.add_param('T_ambient', val = 298.0, units = 'K', desc = 'Ambient Temperture')
        self.add_param('R', val = 287.0, units = 'J/(kg*K)', desc = 'Ideal Gas Constant')
        self.add_param('D_magnetic', val = 150.0, units = 'N', desc = 'Drag from magnetic levitation')
        self.add_param('Thrust_pod', val = 3500.0, units = 'N', desc = 'Thrust from Compressor')
        self.add_param('theta', val = 0.0, units = 'rad', desc = 'elevation angle ')
        self.add_param('g', val = 9.81, units = 'm/s**2', desc = 'gravity')
        self.add_param('m_pod', val = 3100.0, units = 'kg', desc = 'mass of pod')

        self.add_param('V', val = 335.0, units = 'm/s', desc = 'Velocity')

        self.add_output('Thrust', val = 0.0, units = 'N', desc = 'Thrust Force')

    def solve_nonlinear(self, params, unknowns, resids):

        m_pod = params['m_pod']
        g = params['g']
        theta = params['theta']
        Cd = params['Cd']
        S = params['S']
        V = params['V']
        D_magnetic = params['D_magnetic']

        #Calculate air density and drag force
        rho = params['p_tube']/(params['R']*params['T_ambient'])
        unknowns['Thrust'] = (m_pod*g)*(1+sin(theta)) + .5*Cd*rho*S*(V**2) + D_magnetic

if __name__ == '__main__':
    top = Problem()
    root = top.root = Group()

    params = (
        ('V', 335.0, {'units' : 'm/s'}),
        ('Thrust_pod', 3500.0, {'units': 'N'}),
        ('D_magnetic', 150.0, {'units' : 'N'}),
        ('theta', 0.0, {'units' : 'rad'}),
        ('Cd', .2)
    )

    root.add('input_vars', IndepVarComp(params), promotes = ['V','Thrust_pod','D_magnetic', 'theta', 'Cd'])
    root.add('p', MissionThrust(), promotes = ['V','Thrust_pod','D_magnetic', 'theta', 'Cd', 'Thrust'])

    top.setup()
    top.run()

    print('\n')
    print('Total drag force = %f N' % top['Thrust'])




