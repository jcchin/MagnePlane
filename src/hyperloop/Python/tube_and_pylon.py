
from __future__ import print_function

from math import pi, sqrt
from openmdao.api import IndepVarComp, Component, Group, Problem, ExecComp
from openmdao.api import ScipyOptimizer, NLGaussSeidel, Newton

class TubeandPylon(Component):
    """
    Notes
    ------
    Estimates tube tunnel cost and pylon material cost
    Optimizes tunnel thickness, pylon radius, and pylon spacing

    Many parameters are currently taken from hyperloop alpha, will eventually pull from mission trajectory

    Params
    ------
    tube density : float
        density of tube material. Default is 7820 kg/m**3
    tube stiffness : float
        Young's modulus of tube material. Default value is 200e9 Pa
    Tube Poisson's ratio : float
        Poisson's ratio of tube material.  Default value is .3
    Tube strength : float
        Ultimate strength of tube material. Default value is 152e6 Pa
    safety factor : float
        Tube safety factor. Default value is 1.5
    Gravity : float
        Gravitational acceleration. Default value is 9.81 m/s**2
    Tube unit cost : float
        Cost of tube material per unit mass. Default value is .33 USD/kg
    Tube Pressure : float
        Pressure of air in tube.  Default value is 850 Pa.  Value will come from vacuum component
    Ambient Pressure : float
        Pressure of atmosphere. Default value is 101.3e3 Pa.
    Tube coefficient of thermal expansion : float
        Coefficient of thermal expansion of tube material. Default value is 0.0
    Change in tube Temperature : float
        Difference in tunnel temperature as compared ot a reference temperature. Default value is 0.0
    Pod mass : float
        total mass of pod. Default value is 3100 kg. Value will come from weight component
    Tube radius : float
        Radius of tube. Default value is 1.1 m. Value will come from aero module
    Tube thickness : float
        Thickness of the tube. Default value is 50 mm. Value is optimized in problem driver.
    Pylon density : float
        Density of pylon material. Default value is 2400 kg/m**3
    Pylon stiffness : float
        Young's modulus of pylon material. Default value is 41e9 Pa
    Pylon Poisson's ratio : float
        Poisson's ratio of pylon material. Default value is .2
    Pylon strength : float
        Ultimate strength of pylon material. Default value is 40e6 Pa
    Pylon material cost : float
        Cost of pylon material per unit mass. Default value is .05 USD/kg
    Pylon height : float
        Height of each pylon. Default value is 10 m.
    Pylon radius : float
        Radius of each pylon. Default value is 1 m. Value will be optimized in problem driver

    Returns
    -------
    pylon mass : float
        mass of individual pylon in kg/pylon
    Mass per unit length of tube: float
        Calculates mass per unit length of tube in kg/m
    Von Mises : float
        Von Mises stress in the tube in Pa
    Material Cost : float
        returns total cost of tube and pylon materials per unit distance in USD/m
    Pylon load : float
        Returns vertical component of force on each pylon in N
    Tube deflection : float
        Maximum deflection of tube between pylons in m
    Distance between pylons : float
        outputs distance in between pylons in m
    Critical thickness :
        Minimum tube thickness to satisfy vacuum tube buckling condition in m

    Notes
    -----
    [1] USA. NASA. Buckling of Thin-Walled Circular Cylinders. N.p.: n.p., n.d. Web. 13 June 2016.
    """

    def __init__(self):
        super(TubeandPylon, self).__init__()

        #Define material properties of tube
        self.add_param('rho_tube', val = 7820.0, units = 'kg/m**3', desc = 'density of steel')
        self.add_param('E_tube', val = 200.0*(10**9), units = 'Pa', desc = 'Young\'s Modulus of tube')
        self.add_param('v_tube', val = .3, desc = 'Poisson\'s ratio of tube')
        self.add_param('Su_tube', val = 152.0e6, units = 'Pa', desc = 'ultimate strength of tube')
        self.add_param('sf', val = 1.5, desc = 'safety factor')
        self.add_param('g', val = 9.81, units = 'm/s**2', desc = 'gravity')
        self.add_param('unit_cost_tube', val = .3307, units = 'USD/kg', desc = 'cost of tube materials per unit mass')
        self.add_param('p_tunnel', val = 100.0, units = 'Pa', desc = 'Tunnel Pressure')
        self.add_param('p_ambient', val = 101300.0, units = 'Pa', desc = 'Ambient Pressure')
        self.add_param('alpha_tube', val = 0.0, desc = 'Coefficient of Thermal Expansion of tube')
        self.add_param('dT_tube', val = 0.0, units = 'K', desc = 'Temperature change')
        self.add_param('m_pod', val = 3100.0, units = 'kg', desc = 'mass of pod')

        self.add_param('r', val = 1.1, units = 'm', desc = 'inner tube radius')
        self.add_param('t', val = .05, units = 'm', desc = 'tube thickness')
        #self.add_param('dx', val = 500.0, units = 'm', desc = 'distance between pylons')

        #Define pylon material properties
        self.add_param('rho_pylon', val = 2400.0, units='kg/m**3', desc='density of pylon material')
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
        self.add_output('dx', val = 500.0, units='m', desc='distance between pylons')
        self.add_output('t_crit', val = 0.0, units = 'm', desc = 'Minimum tunnel thickness for buckling')

    def solve_nonlinear(self, params, unknowns, resids):
        '''total material cost = ($/kg_tunnel)*m_prime + ($/kg_pylon)*m_pylon*(1/dx)
        m_prime = mass of tunnel per unit length = rho_tube*pi*((r+t)^2-r^2)
        m_pylon = mass of single pylon = rho_pylon*pi*(r_pylon^2)*h

        Constraint equations derived from yield on buckling conditions

        Params
        ------
        tube density : float
            density of tube material. Default is 7820 kg/m**3
        tube stiffness : float
            Young's modulus of tube material. Default value is 200e9 Pa
        Tube Poisson's ratio : float
            Poisson's ratio of tube material.  Default value is .3
        Tube strength : float
            Ultimate strength of tube material. Default value is 152e6 Pa
        safety factor : float
            Tube safety factor. Default value is 1.5
        Gravity : float
            Gravitational acceleration. Default value is 9.81 m/s**2
        Tube unit cost : float
            Cost of tube material per unit mass. Default value is .33 USD/kg
        Tube Pressure : float
            Pressure of air in tube.  Default value is 850 Pa.  Value will come from vacuum component
        Ambient Pressure : float
            Pressure of atmosphere. Default value is 101.3e3 Pa.
        Tube coefficient of thermal expansion : float
            Coefficient of thermal expansion of tube material. Default value is 0.0
        Change in tube Temperature : float
            Difference in tunnel temperature as compared ot a reference temperature. Default value is 0.0
        Pod mass : float
            total mass of pod. Default value is 3100 kg. Value will come from weight component
        Tube radius : float
            Radius of tube. Default value is 1.1 m. Value will come from aero module
        Tube thickness : float
            Thickness of the tube. Default value is 50 mm. Value is optimized in problem driver.
        Pylon density : float
            Density of pylon material. Default value is 2400 kg/m**3
        Pylon stiffness : float
            Young's modulus of pylon material. Default value is 41e9 Pa
        Pylon Poisson's ratio : float
            Poisson's ratio of pylon material. Default value is .2
        Pylon strength : float
            Ultimate strength of pylon material. Default value is 40e6 Pa
        Pylon material cost : float
            Cost of pylon material per unit mass. Default value is .05 USD/kg
        Pylon height : float
            Height of each pylon. Default value is 10 m.
        Pylon radius : float
            Radius of each pylon. Default value is 1 m. Value will be optimized in problem driver

        Returns
        -------
        pylon mass : float
            mass of individual pylon in kg/pylon
        Mass per unit length of tube: float
            Calculates mass per unit length of tube in kg/m
        Von Mises : float
            Von Mises stress in the tube in Pa
        Material Cost : float
            returns total cost of tube and pylon materials per unit distance in USD/m
        Pylon load : float
            Returns vertical component of force on each pylon in N
        Tube deflection : float
            Maximum deflection of tube between pylons in m
        Distance between pylons : float
            outputs distance in between pylons in m
        Critical thickness :
            Minimum tube thickness to satisfy vacuum tube buckling condition in m

        Notes
        -----
        [1] USA. NASA. Buckling of Thin-Walled Circular Cylinders. N.p.: n.p., n.d. Web. 13 June 2016.

        '''

        rho_tube = params['rho_tube']
        E_tube = params['E_tube']
        v_tube = params['v_tube']
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
        unknowns['t_crit'] = r*(((4.0*dp*(1.0-(v_tube**2)))/E_tube)**(1.0/3.0))

if __name__ == '__main__':

    top = Problem()
    root = top.root = Group()

    params = (
        ('r', 1.1, {'units' : 'm'}),
        ('t', 5.0, {'units' : 'm'}),
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

    root.add('con1', ExecComp('c1 = ((Su_tube/sf) - VonMises)'))            #Impose yield stress constraint for tube
    root.add('con2', ExecComp('c2 = t - t_crit'))                           #Impose buckling constraint for tube dx = ((pi**3)*E_pylon*(r_pylon**4))/(8*(h**2)*rho_tube*pi*(((r+t)**2)-(r**2))*g)

    root.connect('input_vars.r', 'p.r')
    root.connect('input_vars.t', 'p.t')
    root.connect('input_vars.r_pylon', 'p.r_pylon')

    root.connect('input_vars.Su_tube', 'con1.Su_tube')
    root.connect('input_vars.sf', 'con1.sf')
    root.connect('p.VonMises', 'con1.VonMises')

    root.connect('input_vars.t', 'con2.t')
    root.connect('p.t_crit', 'con2.t_crit')

    root.p.deriv_options['type'] = "cs"
    # root.p.deriv_options['form'] = 'forward'
    root.p.deriv_options['step_size'] = 1.0e-10

    top.driver = ScipyOptimizer()
    top.driver.options['optimizer'] = 'SLSQP'

    top.driver.add_desvar('input_vars.t', lower = .001, scaler=100.0)
    top.driver.add_desvar('input_vars.r_pylon', lower = .1)
    top.driver.add_objective('p.total_material_cost')
    top.driver.add_constraint('con1.c1', lower = 0.0, scaler = 1000.0)
    top.driver.add_constraint('con2.c2', lower = 0.0)

    top.setup()

    top.run()

    R_buckle = ((pi**3)*top['p.E_tube']*(top['p.r_pylon']**4))/(16*(top['p.h']**2))
    if top['p.R'] < R_buckle:
        print('Pylon buckling constraint is satisfied')
    else:
        r_pylon_new = ((R_buckle*16*(top['p.h']**2))/((pi**3)*top['p.E_tube']))**.25
        print('Optimizer value did not satisfy pylon buckling condition. Pylon radius set to minimum buckling value')
        print('new pylon radius is %f m' % r_pylon_new)

    print('\n')
    print('total material cost per m is $%6.2f/km' % (top['p.total_material_cost']*(1.0e3)))
    print('pylon radius is %6.3f m' % top['p.r_pylon'])
    print('tube thicknes is %6.4f mm' % (top['p.t']*(1.0e3)))
    print('mass per unit length is %6.2f kg/m' % top['p.m_prime'])
    print('vertical force on each pylon is %6.2f kN' % (top['p.R']/(1.0e3)))
    print('Von Mises stress is %6.3f MPa' % (top['p.VonMises']/(1.0e6)))
    print('distance between pylons is %6.2f m' % top['p.dx'])
    print('max deflection is %6.4f mm' % (top['p.delta']*(1.0e3)))
    print('\n')
    print('con1 = %f' % top['con1.c1'])
    print('con2 = %f' % top['con2.c2'])


    if top['con1.c1'] < 0.0:
        print('con1 not satisfied')
    elif top['con2.c2'] < 0.0:
        print('con2 not satisfied')
    else:
        print('Yield constraints are satisfied')