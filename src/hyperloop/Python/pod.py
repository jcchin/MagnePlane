from __future__ import print_function

from math import pi, sqrt
from openmdao.api import IndepVarComp, Component, Group, Problem, ExecComp
from openmdao.api import ScipyOptimizer, NLGaussSeidel, Newton

class TubeandPylon(Component):
    """Optimize the tube diameter, thickness, and distance between pylons
    using a basic sturctural analysis of a pressure cylinder supported at two ends
    Tube is assumed open at the ends in between each support, end effects are neglected for now"""

    def __init__(self):
        super(TubeandPylon, self).__init__()

        #Define material properties of tube
        self.add_param('rho_tube', val = 7820.0, units = 'kg/m^3', desc = 'density of steel')
        self.add_param('E_tube', val = 200.0*(10**9), units = 'Pa', desc = 'Young\'s Modulus of tube')
        self.add_param('v_tube', val = .3, desc = 'Poisson\'s ratio of tube')
        self.add_param('Su_tube', val = 152.0e6, units = 'Pa', desc = 'ultimate strength of tube')
        self.add_param('sf', val = 1.5, desc = 'safety factor')
        self.add_param('g', val = 9.81, units = 'm/s^2', desc = 'gravity')
        self.add_param('unit_cost_tube', val = .3307, units = 'USD/kg', desc = 'cost of tube materials per unit mass')
        self.add_param('p_tunnel', val = 100.0, units = 'Pa', desc = 'Tunnel Pressure')
        self.add_param('p_ambient', val = 101300.0, units = 'Pa', desc = 'Ambient Pressure')
        self.add_param('alpha_tube', val = 0.0, desc = 'Coefficient of Thermal Expansion of tube')
        self.add_param('dT_tube', val = 0.0, units = 'K', desc = 'Temperature change')
        self.add_param('m_pod', val = 3100.0, units = 'kg', desc = 'mass of pod')

        self.add_param('r', val = 1.1, units = 'm', desc = 'inner tube radius')
        self.add_param('t', val = .05, units = 'm', desc = 'tube thickness')
        #self.add_param('dx', val = 500.0, units = 'm', desc = 'distance between pylons')

        self.deriv_options['type'] = 'fd'

        #Define pylon material properties
        self.add_param('rho_pylon', val = 2400.0, units='kg/m^3', desc='density of pylon material')
        self.add_param('E_pylon', val = 41.0*(10**9), units = 'Pa', desc = 'Young\'s Modulus of pylon')
        self.add_param('v_pylon', val = .2, desc = 'Poisson\'s ratio of pylon')
        self.add_param('Su_pylon', val = 40.0*(10**6), units = 'Pa', desc = 'ultimate strength_pylon')
        self.add_param('unit_cost_pylon', val = .05, units = 'USD/kg', desc = 'cost of pylon materials per unit mass')
        self.add_param('h', val = 10.0, units = 'm', desc = 'height of pylon')

        self.add_param('r_pylon', val = 1.1, units = 'm', desc = 'inner tube radius')

        #Define outputs
        self.add_output('m_pylon', val = 0.0, units = 'kg', desc = 'total mass of the pylon')
        self.add_output('m_prime', val = 100.0, units = 'kg/m', desc = 'total mass of the tube per unit length')
        self.add_output('VonMises', val = 0.0, units = 'Pa', desc = 'max Von Mises Stress')
        self.add_output('total_material_cost', val = 0.0, units = 'USD', desc = 'cost of materials')
        self.add_output('R', val = 0.0, units = 'N', desc = 'Force on pylon')
        self.add_output('delta', val = 0.0, units = 'm', desc = 'max deflection inbetween pylons')
        self.add_output('dx', val=500.0, units='m', desc='distance between pylons')

    def solve_nonlinear(self, params, unknowns, resids):

        rho_tube = params['rho_tube']
        E_tube = params['E_tube']
        alpha_tube = params['alpha_tube']
        dT_tube = params['dT_tube']
        unit_cost_tube = params['unit_cost_tube']
        g = params['g']
        r = params['r']
        t = params['t']
        m_pod = params['m_pod']
        p_tunnel = params['p_tunnel']
        p_ambient = params['p_ambient']
        Su_pylon = params['Su_pylon']
        sf = params['sf']
        rho_pylon = params['rho_pylon']
        E_pylon = params['E_pylon']
        r_pylon = params['r_pylon']
        unit_cost_pylon = params['unit_cost_pylon']
        h = params['h']

        #Compute intermediate variable
        q = rho_tube*pi*(((r+t)**2)-(r**2))*g                                               #Calculate distributed load
        dp = p_ambient - p_tunnel                                                           #Calculate delta pressure
        I_tube = (pi/4.0)*(((r+t)**4)-(r**4))                                               #Calculate moment of inertia of tube

        m_prime = rho_tube*pi*(((r+t)**2)-(r**2))                                           #Calculate mass per unit length
        #dx = ((((pi**3)*E_pylon*(r_pylon**4))/(16*(h**2)))-.5*m_pod*g)/(.5*m_prime*g)       #Calculate distance between pylons
        dx = ((2*(Su_pylon/sf)*pi*(r_pylon**2))-m_pod*g)/(m_prime*g)                        #Calculate dx
        M = (q*((dx**2)/8.0))+(m_pod*g*(dx/2.0))                                            #Calculate max moment
        sig_theta = (dp*r)/t                                                                #Calculate hoop stress
        sig_axial = ((dp*r)/(2*t)) + ((M*r)/I_tube) + alpha_tube*E_tube*dT_tube             #Calculate axial stress
        VonMises = (((sig_theta**2)+(sig_axial**2)+((sig_axial-sig_theta)**2))/2.0)**.5     #Calculate Von Mises stress
        m_pylon = rho_pylon*pi*(r_pylon**2)*h                                               #Calculate mass of single pylon

        unknowns['total_material_cost'] = (unit_cost_tube*(rho_tube*pi*(((r+t)**2)-(r**2)))) + (unit_cost_pylon*m_pylon*(1/(((2*(Su_pylon/sf)*pi*(r_pylon**2))-m_pod*g)/(m_prime*g))))
        unknowns['m_prime'] = m_prime
        unknowns['VonMises'] = VonMises
        unknowns['delta'] = (5.0*q*(dx**4))/(384.0*E_tube*I_tube)
        unknowns['m_pylon'] = m_pylon
        unknowns['R'] = .5*m_prime*dx*g+.5*m_pod*g
        unknowns['dx'] = dx

    def linearize(self, params, unknowns, resids):

        rho_tube = params['rho_tube']
        unit_cost_tube = params['unit_cost_tube']
        g = params['g']
        r = params['r']
        t = params['t']
        rho_pylon = params['rho_pylon']
        E_pylon = params['E_pylon']
        r_pylon = params['r_pylon']
        unit_cost_pylon = params['unit_cost_pylon']
        h = params['h']

        J={}
        J['total_material_cost', 't'] = unit_cost_tube*2*rho_tube*pi*(r+t) + unit_cost_pylon*((8*(h**3)*g*rho_pylon*2*rho_tube*pi*(r+t))/((pi**2)*E_pylon*(r_pylon**2)))
        J['total_material_cost', 'r_pylon'] = (-1)*unit_cost_pylon*((16*(h**3)*g*rho_pylon*rho_tube*pi*(((r+t)**2)-(r**2)))/((pi**2)*E_pylon*(r_pylon**3)))

        return J

if __name__ == '__main__':

    top = Problem()
    root = top.root = Group()

    params = (
        ('r', 1.1, {'units' : 'm'}),
        ('t', .05, {'units' : 'm'}),
        ('r_pylon', 1.1, {'units': 'm'}),
        ('Su_tube', 152.0e6, {'units': 'Pa'}),
        ('sf', 1.5),
        ('p_ambient', 100.0, {'units': 'Pa'}),
        ('p_tunnel', 101300.0, {'units': 'Pa'}),
        ('v_tube', .3),
        ('E_tube', 210.0e9, {'units': 'Pa'}),
        ('rho_tube', 7820.0, {'units': 'kg/m^3'}),
        ('rho_pylon', 2400.0, {'units': 'Pa'}),
        ('Su_pylon', 40.0e6, {'units': 'Pa'}),
        ('E_pylon', 41.0e9, {'units' : 'Pa'}),
        ('h', 10.0, {'units' : 'm'}),
        ('m_pod', 3100.0, {'units' : 'kg'})

    )
    root.add('input_vars', IndepVarComp(params))
    root.add('p', TubeandPylon())

    root.add('con1', ExecComp('c1 = (Su_tube/sf) - VonMises'))                                                      #Impose yield stress constraint for tube
    root.add('con2', ExecComp('c2 = t - r*(((4*(p_ambient-p_tunnel)*(1-(v_tube**2)))/(E_tube))**(1/3))'))           #Impose buckling constraint for tube dx = ((pi**3)*E_pylon*(r_pylon**4))/(8*(h**2)*rho_tube*pi*(((r+t)**2)-(r**2))*g)
    #root.add('con3', ExecComp('c3 = (Su_pylon/sf) - R/(pi*(r_pylon**2))'))                                          #Impose yield stress constraint for pylon
    root.add('con3', ExecComp('c3 = R - (((pi**3)*E_pylon*(r_pylon**4))/(4*(h**2)))'))

    root.connect('input_vars.r', 'p.r')
    root.connect('input_vars.t', 'p.t')
    root.connect('input_vars.r_pylon', 'p.r_pylon')

    root.connect('input_vars.Su_tube', 'con1.Su_tube')
    root.connect('input_vars.sf', 'con1.sf')
    root.connect('p.VonMises', 'con1.VonMises')

    root.connect('input_vars.t', 'con2.t')
    root.connect('input_vars.r', 'con2.r')
    root.connect('input_vars.p_ambient', 'con2.p_ambient')
    root.connect('input_vars.p_tunnel', 'con2.p_tunnel')
    root.connect('input_vars.v_tube', 'con2.v_tube')
    root.connect('input_vars.E_tube', 'con2.E_tube')

    root.connect('input_vars.E_pylon', 'con3.E_pylon')
    root.connect('input_vars.h', 'con3.h')
    root.connect('input_vars.r_pylon', 'con3.r_pylon')
    root.connect('p.R', 'con3.R')

    root.p.fd_options['force_fd'] = True
    root.p.fd_options['form'] = 'central'
    root.p.fd_options['step_size'] = 1.0e-4

    #top.driver = ScipyOptimizer()
    #top.driver.options['optimizer'] = 'SLSQP'

    #top.driver.add_desvar('input_vars.t', lower = .001)
    #top.driver.add_desvar('input_vars.r_pylon', lower = .5)
    #top.driver.add_objective('p.total_material_cost')
    #top.driver.add_constraint('con1.c1', lower = 0.0)
    #top.driver.add_constraint('con2.c2', lower = 0.0)
    #top.driver.add_constraint('con3.c3', lower = 0.0)

    top.setup()

    top.run()

    print('\n')
    print('total material cost per m is $%f /m' % top['p.total_material_cost'])
    print('pylon radius is %f m' % top['p.r_pylon'])
    print('tube thicknes is %f m' % top['p.t'])
    print('mass per unit length os %f kg/m' % top['p.m_prime'])
    print('vertical force on each pylon is %f N' % top['p.R'])
    print('Von Mises stress is %f Pa' % top['p.VonMises'])
    print('distance between pylons is %f m' % top['p.dx'])
    print('max deflection is %f m' % top['p.delta'])
    print('\n')
    print('con1 = %f' % top['con1.c1'])
    print('con2 = %f' % top['con2.c2'])


