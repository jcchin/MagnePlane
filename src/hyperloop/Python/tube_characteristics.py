from __future__ import print_function

from math import pi, sqrt
from openmdao.api import IndepVarComp, Component, Group, Problem, ExecComp
from openmdao.api import ScipyOptimizer

class TubeCharacteristics(Component):
    """Optimize the tube diameter, thickness, and distance between pylons
    using a basic sturctural analysis of a pressure cylinder supported at two ends
    Tube is assumed open at the ends in between each support, end effects are neglected for now"""

    def __init__(self):
        super(TubeCharacteristics, self).__init__()

        #Define material properties of steel
        self.add_param('rho', val = 7820.0, units = 'kg/m^3', desc = 'density of steel')
        self.add_param('E', val = 200.0*(10**9), units = 'Pa', desc = 'Young\'s Modulus')
        self.add_param('v', val = .3, desc = 'Poisson\'s ratio')
        self.add_param('Su', val = 400*(10**6), units = 'Pa', desc = 'ultimate strength')
        self.add_param('sf', val = 1.5, desc = 'safety factor')
        self.add_param('g', val = 9.81, units = 'm/s^2', desc = 'gravity')
        self.add_param('unit_cost', val = .1322, units = 'USD/kg', desc = 'cost of materials per unit mass')
        self.add_param('p_tunnel', val = 100.0, units = 'Pa', desc = 'Tunnel Pressure')
        self.add_param('p_ambient', val = 101300.0, units = 'Pa', desc = 'Ambient Pressure')

        self.add_param('r', val = 1.1, units = 'm', desc = 'inner tube radius')
        self.add_param('t', val = .05, units = 'm', desc = 'tube thickness')
        self.add_param('dx', val = 500.0, units = 'm', desc = 'distance between tubes')

        #Define outputs
        self.add_output('m_tube', val = 0.0, units = 'kg', desc = 'total mass of the tube')
        self.add_output('VonMises', val = 0.0, units = 'Pa', desc = 'max Von Mises Stress')
        self.add_output('materials_cost', val = 0.0, units = 'USD', desc = 'cost of tube materials')
        self.add_output('R', val = 0.0, units = 'N', desc = 'Force on pylon')
        self.add_output('delta', val = 0.0, units = 'm', desc = 'max deflection inbetween pylons')

    def solve_nonlinear(self, params, unknowns, resids):

        rho = params['rho']
        E = params['E']
        g = params['g']
        r = params['r']
        t = params['t']
        dx = params['dx']
        p_tunnel = params['p_tunnel']
        p_ambient = params['p_ambient']


        #Calculate intermeiate variables
        q = rho*pi*(((r+t)**2)-(r**2))*g
        I = (pi/4)*(((r+t)**2)-(r**2))
        Z = ((pi/4)*(((r+t)**4)-(r**4)))/(r+t)
        sig_theta = ((p_tunnel-p_ambient)*r)/t
        sig_axial = (((p_tunnel - p_ambient)*r)/(2*t)) + ((q*dx)/(8*Z))

        unknowns['VonMises'] = (.5*((sig_axial**2)+(sig_theta**2)+((sig_axial-sig_theta)**2)))**.5
        unknowns['m_tube'] = rho*pi*(((r+t)**2)-(r**2))
        unknowns['R'] = .5*q*dx
        unknowns['delta'] = (5*q*(dx**4))/(384*E*I)
        unknowns['materials_cost'] = params['unit_cost'] * unknowns['m_tube']

    def linearize(self, params, unknowns, resids):

        rho = params['rho']
        r = params['r']
        t = params['t']
        dx = params['dx']
        J = {}

        J['m_tube', 'dx'] = rho*pi*(((r+t)**2)-(r**2))
        J['m_tube', 'r'] = 2*rho*pi*dx*t
        J['m_tube', 't'] = 2*rho*pi*dx*(r+t)

        return J


class PylonCharacteristics(Component):
    """Optimize the pylon diameter and distance between pylons
    using a basic sturctural analysis of a cylindrical beam supported at two ends
    Tube is assumed open at the ends in between each support, end effects are neglected for now"""

    def __init__(self):
        super(PylonCharacteristics, self).__init__()

        #Define material properties of steel
        self.add_param('rho_tube', val = 7820.0, units = 'kg/m^3', desc = 'density of tube material')
        self.add_param('rho_pylon', val = 2400, units='kg/m^3', desc='density of pylon material')
        self.add_param('E', val = 200.0*(10**9), units = 'Pa', desc = 'Young\'s Modulus')
        self.add_param('v', val = .3, desc = 'Poisson\'s ratio')
        self.add_param('Su', val = 400*(10**6), units = 'Pa', desc = 'ultimate strength')
        self.add_param('sf', val = 1.5, desc = 'safety factor')
        self.add_param('g', val = 9.81, units = 'm/s^2', desc = 'gravity')
        self.add_param('unit_cost', val = .1322, units = 'USD/kg', desc = 'cost of materials per unit mass')
        self.add_param('R', val=0.0, units='N', desc = 'Force on pylon')
        self.add_param('h', val = 10.0, units = 'm', desc = 'height of pylon')
        self.add_param('r', val = 1.1, units = 'm', desc = 'tube radius')
        self.add_param('t', val = .05, units = 'm', desc = 'tub thickness')

        self.add_param('r_pylon', val = 1.1, units = 'm', desc = 'inner tube radius')
        self.add_param('t', val = .05, units = 'm', desc = 'tube thickness')

        #Define outputs
        self.add_output('dx', val=500.0, units='m', desc='distance between tubes')
        self.add_output('m_pylon', val = 0.0, units = 'kg', desc = 'total mass of the pylon')
        #self.add_output('materials_cost', val = 0.0, units = 'USD', desc = 'cost of tube materials')

    def solve_nonlinear(self, params, unknowns, resids):

        rho_tube = params['rho_tube']
        rho_pylon = params['rho_pylon']
        E = params['E']
        r_pylon = params['r_pylon']
        r = params['r']
        t = params['t']
        h = params['h']
        sf = params['sf']


        #Calculate intermeiate variables
        Vol = pi*(r_pylon**2)*h

        unknowns['dx'] = (((pi**3)*E*(r_pylon**4))/(8*(h**2)*sf))/(rho_tube*pi*(((r+t)**2)-(r**2))*g)
        unknowns['m_pylon'] = rho_pylon*Vol



    def linearize(self, params, unknowns, resids):

        rho_tube = params['rho_tube']
        rho_pylon = params['rho_pylon']
        E = params['E']
        r_pylon = params['r_pylon']
        r = params['r']
        t = params['t']
        h = params['h']
        sf = params['sf']
        J = {}

        J['dx, r_pylon'] = (((pi**3)*E*(r_pylon**3))/(2*(h**2)*sf))/(rho_tube*pi*(((r+t)**2)-(r**2))*g)

        return J

if __name__ == '__main__':
     top = Problem()

     root = Group()
     root.add('p1', IndepVarComp('r', 1.1))
     root.add('p2', IndepVarComp('t', .05))
     root.add('p3', IndepVarComp('dx', 500.0))
     root.add('p', TubeCharacteristics)

     root.add('con1', ExecComp('c1 = (Su/sf) - VonMises'))                                      #Impose yield stress constraint
     root.add('con2', ExecComp('c2 = (p_ambient - p_tunnel) - (E/(4*(1-v**2)))*((t/r)**3)'))    #Impose buckling constraint

     root.connect('p1.r', 'p.r')
     root.connect('p2.t', 'p.t')
     root.connect('p3.dx', 'p.dx')
     root.connect('p.Su', 'con1.Su')
     root.connect('p.sf', 'con1.sf')
     root.connect('p.VonMises', 'con1.VonMises')
     root.connect('p.E', 'con2.E')
     root.connect('p.p_tunnel', 'con2.p_tunnel')
     root.connect('p.p_ambient', 'con2.p_ambient')
     root.connect('p.v', 'con2.v')
     root.connect('p.t', 'con2.t')
     root.connect('p.r', 'con2.r')

     top.root = root

     top.driver = ScipyOptimizer()
     top.driver.options['optimizer'] = 'SLSQP'

     top.driver.add_desvar('p1.r')
     top.driver.add_desvar('p1.t')
     top.driver.add_desvar('p1.dx')
     top.driver.add_objective('p.m_tube')
     top.driver.add_constraint('con.c1', lower = 0.0)
     top.driver.add_constraint('con.c2', lower = 0.0)

     top.setup()

     top['p1.r'] = 1.1
     top['p2.t'] = .05
     top['p3.dx'] = 500.0

     top.run()

     print('\n')
     print('Minimum tube mass is %f with a radius of %f and a thickness of %f' % (top['p.m_tube'], top['p.r'], top['p.t']))




