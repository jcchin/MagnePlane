from __future__ import  print_function
from math import pi, log

from openmdao.api import IndepVarComp, Component, Group, Problem

class PropulsionMechanics(Component):
    """Evaluates the required power a motor would have to generate to accelerate pod by 1g
    Input values can be used for LIM or LSM models
    Can later be modified to include current constraints, factor in cost, and optimize several variables"""


    def __init__(self):
        """Establish inputs to equation.  Values initialized as practical values for LSM motors
        Output: Power required"""

        super(PropulsionMechanics, self).__init__()

        self.add_param('p_ambient', val = 100.0, desc = 'Ambient Pressure', units = 'Pa')
        self.add_param('R', val = 286.9, desc = 'Ideal gas constant of air', units = 'J/(kg * K)' )
        self.add_param('T_ambient', val = 293.0, desc = 'Ambient Temperature', units = 'K')
        self.add_param('g', val = 9.81, desc = 'Gavity', units = 'm/s^2')
        self.add_param('vf', val = 335.0, desc = 'Top Speed', units = 'm/s')
        self.add_param('v0', val = 324.0, desc = 'Entrance Speed', units = 'm/s')
        self.add_param('rho_pm', val = 7400.0, desc = 'Density of PM', units = 'kg/m^3')
        self.add_param('A', val = .0225, desc = 'Area of magnets', units = 'm^2')
        self.add_param('t', val = .05, desc = 'Thickness of magnets', units = 'm')
        self.add_param('m_pod', val = 3100.0, desc = 'mass of the pod without the magnets', units = 'kg')
        self.add_param('eta', val = .8, desc = 'LSM efficiency')
        self.add_param('Cd', val = .2, desc = 'Aerodynamic drag coefficient')
        self.add_param('S', val = 1.4, desc = 'Frontal Area', units = 'm^2')

        self.add_output('pwr_req', val = 0.0)             #Define power as output
        self.add_output('Fg_dP', val = 0.0)               #Define Thrust per unit Power output
        self.add_output('m_dP', val = 0.0)                #Define mass per unit power as output

    def solve_nonlinear(self, params, unknowns, resids):
        """Evaluate function Preq = (1/eta)*(mg*(vf-vo)+(1/6)*(Cd*rho*S*(vf^3 - vo^3)))
        Can be optimized in the future.  Friction and magnetic drag are neglected for now."""

        eta = params['eta']
        g = params['g']
        vf = params['vf']
        v0 = params['v0']
        Cd = params['Cd']
        S = params['S']

        #Calculate intermediate variables
        rho = params['p_ambient']/(params['R']*params['T_ambient'])         #Calculate air density, rho = P/(RT)
        m = params['m_pod'] + params['rho_pm']*params['A']*params['t']      #Calculate total mass, m = m_pod + rho*A*t
        L = ((vf**2)-(v0**2))/(2*g)                                         #Calculate necessary track length

        #Evaluate equation
        unknowns['pwr_req'] = (1.0/eta)*((m*g*(vf-v0))+(1.0/6.0)*(Cd*rho*S*((vf**3.0)-(v0**3.0))))
        unknowns['Fg_dP'] = (m*g)/unknowns['pwr_req']
        unknowns['m_dP'] = m/unknowns['pwr_req']


if __name__ == '__main__':

    #Set up problem to accept outside inputs for efficiency, eta, and magnet density, rho_pm
    root = Group()
    root.add('p', PropulsionMechanics())
    root.add('p1', IndepVarComp('eta',.8))
    root.add('p2', IndepVarComp('rho_pm', 7400.0))

    root.connect('p1.eta','p.eta')
    root.connect('p2.rho_pm','p.rho_pm')

    #Set up problem
    top = Problem()
    top.root = root

    #Will not specify driver since only default driver is required at this time

    top.setup()

    top['p1.eta'] = .8
    top['p2.rho_pm'] = 7400.0

    top.run()

    print('\n')
    print('Power Required is %f W' % top['p.pwr_req'])
    print('Thrust per unit power = %f N/W' % top['p.Fg_dP'])
    print('Mass per unit power = %f kg/W' % top['p.m_dP'])
