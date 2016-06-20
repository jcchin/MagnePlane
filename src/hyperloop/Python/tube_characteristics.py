from __future__ import print_function

from math import pi, sqrt
from openmdao.api import IndepVarComp, Component, Group, Problem, ExecComp
from openmdao.api import ScipyOptimizer, NLGaussSeidel, Newton

class TubeCharacteristics(Component):
    """Optimize the tube diameter, thickness, and distance between pylons
    using a basic sturctural analysis of a pressure cylinder supported at two ends
    Tube is assumed open at the ends in between each support, end effects are neglected for now"""

    def __init__(self):
        super(TubeCharacteristics, self).__init__()

        #Define material properties of steel
        self.add_param('rho_tube', val = 7820.0, units = 'kg/m^3', desc = 'density of steel')
        self.add_param('E', val = 200.0*(10**9), units = 'Pa', desc = 'Young\'s Modulus')
        self.add_param('v', val = .3, desc = 'Poisson\'s ratio')
        self.add_param('Su', val = 400.0e6, units = 'Pa', desc = 'ultimate strength')
        self.add_param('sf', val = 1.5, desc = 'safety factor')
        self.add_param('g', val = 9.81, units = 'm/s^2', desc = 'gravity')
        self.add_param('unit_cost', val = .1322, units = 'USD/kg', desc = 'cost of materials per unit mass')
        self.add_param('p_tunnel', val = 100.0, units = 'Pa', desc = 'Tunnel Pressure')
        self.add_param('p_ambient', val = 101300.0, units = 'Pa', desc = 'Ambient Pressure')

        self.add_param('r', val = 1.1, units = 'm', desc = 'inner tube radius')
        self.add_param('t', val = .05, units = 'm', desc = 'tube thickness')
        self.add_param('dx', val = 500.0, units = 'm', desc = 'distance between tubes')

        self.deriv_options['type'] = 'user'

        #Define outputs
        self.add_output('m_tube', val = 100.0, units = 'kg', desc = 'total mass of the tube')
        self.add_output('VonMises', val = 0.0, units = 'Pa', desc = 'max Von Mises Stress')
        self.add_output('materials_cost', val = 0.0, units = 'USD', desc = 'cost of tube materials')
        self.add_output('R', val = 0.0, units = 'N', desc = 'Force on pylon')
        self.add_output('delta', val = 0.0, units = 'm', desc = 'max deflection inbetween pylons')

    def solve_nonlinear(self, params, unknowns, resids):
        """Evaluate equation for mass of the tube (explicit).
        Inputs: material properties, tube pressure, initialized values for tunnel radius and wall thickness
        Outputs: tube mass, load on pylons, max deflection of ube between pylons, max Von Mises stress in tube, material cost

        Equation: m = """

        rho_tube = params['rho_tube']
        E = params['E']
        g = params['g']
        r = params['r']
        t = params['t']
        dx = params['dx']
        p_tunnel = params['p_tunnel']
        p_ambient = params['p_ambient']


        #Calculate intermeiate variables
        q = rho_tube*pi*(((r+t)**2)-(r**2))*g
        I = (pi/4)*(((r+t)**2)-(r**2))
        Z = ((pi/4)*(((r+t)**4)-(r**4)))/(r+t)
        sig_theta = ((p_tunnel-p_ambient)*r)/t
        sig_axial = (((p_tunnel - p_ambient)*r)/(2*t)) + ((q*dx)/(8*Z))

        unknowns['VonMises'] = (.5*((sig_axial**2)+(sig_theta**2)+((sig_axial-sig_theta)**2)))**.5
        unknowns['m_tube'] = rho_tube*pi*(((r+t)**2)-(r**2))*dx
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
        self.add_param('m_tube', val = 0.0, units = 'kg', desc = 'mass of the tube')

        #Define outputs
        self.add_output('dx', val=500.0, units='m', desc='distance between tubes')
        self.add_output('m_pylon', val = 0.0, units = 'kg', desc = 'total mass of the pylon')
        #self.add_output('materials_cost', val = 0.0, units = 'USD', desc = 'cost of tube materials')

    def solve_nonlinear(self, params, unknowns, resids):
        """Evaluate equaion for distance in between pylons (explicit)
        Inputs: materil properties, tube radius (r), tube thickness (t)
        Outputs: distance between pylons (dx)"""

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
        E = params['E']
        r_pylon = params['r_pylon']
        r = params['r']
        t = params['t']
        h = params['h']
        sf = params['sf']
        J = {}

        J['dx, r_pylon'] = (((pi**3)*E*(r_pylon**3))/(2*(h**2)*sf))/(rho_tube*pi*(((r+t)**2)-(r**2))*g)

        return J

if False: #__name__ == '__main__':
    top = Problem()

    root = top.root = Group()

    params = (
        ('r', 1.1, {'units':'m'}),
        ('t', 0.05, {'units':'m'}),
        ('dx', 500., {'units':'m'}),
        ('Su', 400.0e6, {'units':'Pa'}),
        ('sf', 1.5),
        ('p_ambient', 101300.0, {'units': 'Pa'}),
        ('p_tunnel', 100.0, {'units': 'Pa'}),
        ('E', 210.0e9, {'units': 'Pa'}),
        ('v', 0.3)
    )
    top.root.add('des_vars', IndepVarComp(params))

    # root.add('p1', IndepVarComp('r', 1.1, units = 'm'))
    # root.add('p2', IndepVarComp('t', .05, units = 'm'))
    # root.add('p3', IndepVarComp('dx', 500.0, units = 'm'))
    # root.add('p4', IndepVarComp('Su', 400.0e6, units = 'Pa'))
    # root.add('p5', IndepVarComp('sf', 1.5, units=''))
    # root.add('p6', IndepVarComp('p_ambient', 101300.0, units='Pa'))
    # root.add('p7', IndepVarComp('p_tunnel', 100.0, units = 'Pa'))
    # root.add('p8', IndepVarComp('E', 210.0e9, units = 'Pa'))
    # root.add('p9', IndepVarComp('v', .3, units = ''))
    root.add('p', TubeCharacteristics())

    root.add('con1', ExecComp('c1 = (Su/sf) - VonMises'))                                      #Impose yield stress constraint
    root.add('con2', ExecComp('c2 = (p_ambient - p_tunnel) - (E/(4*(1-v**2)))*((t/r)**3)'))    #Impose buckling constraint

    root.connect('des_vars.r', 'p.r')
    root.connect('des_vars.t', 'p.t')
    root.connect('des_vars.dx', 'p.dx')

    root.connect('des_vars.Su', 'con1.Su')
    root.connect('des_vars.sf', 'con1.sf')
    root.connect('des_vars.E', 'con2.E')
    root.connect('des_vars.p_tunnel', 'con2.p_tunnel')
    root.connect('des_vars.p_ambient', 'con2.p_ambient')
    root.connect('des_vars.v', 'con2.v')

    root.connect('p.VonMises', 'con1.VonMises')
    root.connect('p.t', 'con2.t')
    root.connect('p.r', 'con2.r')

    top.root = root

    top.driver = ScipyOptimizer()
    top.driver.options['optimizer'] = 'SLSQP'
    root.nl_solver = Newton()
    root.nl_solver.options['iprint'] = 1
    root.nl_solver.options['maxiter'] = 10

    top.driver.add_desvar('des_vars.r', lower = 0.5)
    top.driver.add_desvar('des_vars.t', lower = 0.001)
    top.driver.add_desvar('des_vars.dx', lower = 1.0)
    top.driver.add_objective('p.m_tube')
    top.driver.add_constraint('con1.c1', lower = 0.0)
    top.driver.add_constraint('con2.c2', lower = 0.0)

    top.setup()

    top.run()

    print(top['des_vars.t'])

    print('\n')
    print('Minimum tube mass is %f kg with a radius of %f m and a thickness of %f m' % (top['p.m_tube'], top['p.r'], top['p.t']))



if __name__ == '__main__':


    tube_comp = TubeCharacteristics()
    pylon_comp = PylonCharacteristics()

    g1 = Group()
    g1.add('comp1', tube_comp, promotes = ['r', 't'])
    g1.add('comp2', pylon_comp, promotes = ['r', 't'])

    g1.r = 1.1
    g1.t = .05
    g1.rho_tube = 7820.0

    tube_params = (
        ('Su', 400.0e6, {'units':'Pa'}),
        ('sf', 1.5),
        ('p_ambient', 101300.0, {'units': 'Pa'}),
        ('p_tunnel', 100.0, {'units': 'Pa'}),
        ('E', 210.0e9, {'units': 'Pa'}),
        ('v', 0.3)
    )
    g1.add('tube_vars', IndepVarComp(tube_params))

    pylon_params = (
        ('Su', 40.0e6, {'units' : 'Pa'}),
        ('sf',1.5),
        ('h', 10, {'units' : 'm'}),
        ('E', 14.0e9, {'units' : 'Pa'}),
        ('g', 9.81, {'units':'m/s^2'}),
        ('r_pylon', .1, {'units' : 'm'})
    )

    g1.add('pylon_vars', IndepVarComp(pylon_params))
    g1.add('con1', ExecComp('c1 = (Su/sf) - VonMises'))                                      #Impose yield stress constraint
    g1.add('con2', ExecComp('c2 = (p_ambient - p_tunnel) - (E/(4*(1-v**2)))*((t/r)**3)'))    #Impose buckling constraint
    g1.add('con3', ExecComp('c3 = (Su/sf)-(R/(pi*(r_pylon**2)))'))                           #Impose yield stress constraint on pylon

    g1.connect('comp2.dx', 'comp1.dx')

    g1.connect('tube_vars.Su', 'con1.Su')
    g1.connect('tube_vars.sf', 'con1.sf')
    g1.connect('comp1.VonMises', 'con1.VonMises')
    g1.connect('tube_vars.p_ambient', 'con2.p_ambient')
    g1.connect('tube_vars.p_tunnel', 'con2.p_tunnel')
    g1.connect('tube_vars.E', 'con2.E')
    g1.connect('tube_vars.v', 'con2.v')

    g1.connect('pylon_vars.Su', 'con3.Su')
    g1.connect('pylon_vars.sf', 'con3.sf')
    g1.connect('comp1.R', 'con3.R')
    g1.connect('pylon_vars.r_pylon','con3.r_pylon')

    top = Problem()
    top.root = g1









