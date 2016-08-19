from __future__ import print_function

import numpy as np
import matplotlib.pylab as plt
from openmdao.api import IndepVarComp, Component, Group, Problem, ExecComp, ScipyOptimizer

class UnderwaterOptimization(Component):
    """
    Notes
    -----
    Estimates tube tunnel cost and pylon material cost
    Optimizes tunnel thickness, pylon radius, and pylon spacing

    Many parameters are currently taken from hyperloop alpha, will eventually pull from mission trajectory

    Params
    ------
    tube_area : float
        Inner tube radius. Default is 3.8013 m**2
    rho_tube : float
        Density of tube material. Default is 7820 kg/m**3
    E_tube : float
        Young's modulus of tube material. Default value is 200e9 Pa
    v_tube : float
        Poisson's ratio of tube material.  Default value is .3
    Su_tube : float
        Ultimate strength of tube material. Default value is 152e6 Pa
    sf : float
        Tube safety factor. Default value is 1.5
    g : float
        Gravitational acceleration. Default value is 9.81 m/s**2
    unit_cost_tube : float
        Cost of tube material per unit mass. Default value is .33 USD/kg
    p_tunnel : float
        Pressure of air in tube.  Default value is 850 Pa.  Value will come from vacuum component
    p_ambient : float
        Pressure of atmosphere. Default value is 101.3e3 Pa.
    alpha_tube : float
        Coefficient of thermal expansion of tube material. Default value is 0.0
    dT_tube : float
        Difference in tunnel temperature as compared ot a reference temperature. Default value is 0.0
    m_pod : float
        total mass of pod. Default value is 3100 kg. Value will come from weight component
    r : float
        Radius of tube. Default value is 1.1 m. Value will come from aero module
    t : float
        Thickness of the tube. Default value is 50 mm. Value is optimized in problem driver.
    rho_pylon : float
        Density of pylon material. Default value is 2400 kg/m**3
    E_pylon : float
        Young's modulus of pylon material. Default value is 41e9 Pa
    v_pylon : float
        Poisson's ratio of pylon material. Default value is .2
    Su_pylon : float
        Ultimate strength of pylon material. Default value is 40e6 Pa
    unit_cost_pylon : float
        Cost of pylon material per unit mass. Default value is .05 USD/kg
    h : float
        Height of each pylon. Default value is 10 m.
    r_pylon : float
        Radius of each pylon. Default value is 1 m. Value will be optimized in problem driver
    vac_weight : float
        Total weight of vacuums. Default value is 1500.0 kg. Value will come from vacuum component

    Returns
    -------
    m_pylon : float
        mass of individual pylon in kg/pylon
    m_prime: float
        Calculates mass per unit length of tube in kg/m
    von_mises : float
        Von Mises stress in the tube in Pa
    total_material_cost : float
        returns total cost of tube and pylon materials per unit distance in USD/m
    R : float
        Returns vertical component of force on each pylon in N
    delta : float
        Maximum deflection of tube between pylons in m
    dx : float
        outputs distance in between pylons in m
    t_crit :
        Minimum tube thickness to satisfy vacuum tube buckling condition in m

    Notes
    -----
    [1] USA. NASA. Buckling of Thin-Walled Circular Cylinders. N.p.: n.p., n.d. Web. 13 June 2016.
    """
    def __init__(self):
        super(UnderwaterOptimization, self).__init__()
        #Define material properties of tube
        self.add_param('rho_tube',
                       val=7820.0,
                       units='kg/m**3',
                       desc='density of steel')
        self.add_param('E_tube',
                       val=200.0 * (10**9),
                       units='Pa',
                       desc='Young\'s Modulus of tube')
        self.add_param('v_tube', val=.3, desc='Poisson\'s ratio of tube')
        self.add_param('Su_tube',
                       val=152.0e6,
                       units='Pa',
                       desc='ultimate strength of tube')
        self.add_param('sf', val=1.5, desc='safety factor')
        self.add_param('g', val=9.81, units='m/s**2', desc='gravity')
        self.add_param('unit_cost_tube',
                       val=.3307,
                       units='USD/kg',
                       desc='cost of tube materials per unit mass')
        self.add_param('p_tunnel',
                       val=100.0,
                       units='Pa',
                       desc='Tunnel Pressure')
        self.add_param('p_atm',
                       val=101300.0,
                       units='Pa',
                       desc='Ambient Pressure')
        self.add_param('alpha_tube',
                       val=0.0,
                       desc='Coefficient of Thermal Expansion of tube')
        self.add_param(
            'dT_tube', val=0.0,
            units='K', desc='Temperature change')
        self.add_param('m_pod', val=3100.0, units='kg', desc='mass of pod')

        self.add_param('tube_area', val=3.8013, units='m**2', desc='inner tube area')
        #self.add_param('r', val=1.1, units='m', desc='inner tube radius')
        self.add_param('t', val=.05, units='m', desc='tube thickness')
        #self.add_param('dx', val = 500.0, units = 'm', desc = 'distance between pylons')

        #Define pylon material properties
        self.add_param('rho_pylon',
                       val=7820.0,
                       units='kg/m**3',
                       desc='density of pylon material')
        self.add_param('E_pylon',
                       val=200 * (10**9),
                       units='Pa',
                       desc='Young\'s Modulus of pylon')
        self.add_param('v_pylon', val=.2, desc='Poisson\'s ratio of pylon')
        self.add_param('Su_pylon',
                       val=152.0 * (10**6),
                       units='Pa',
                       desc='ultimate strength_pylon')
        self.add_param('unit_cost_pylon',
                       val=.3307,
                       units='USD/kg',
                       desc='cost of pylon materials per unit mass')
        self.add_param('h', val=10.0, units='m', desc='height of pylon')

        self.add_param('r_pylon', val=.1, units='m', desc='inner tube radius')

        self.add_param('vac_weight', val=1500.0, units='kg', desc='vacuum weight')
        self.add_param('rho_water', val = 1025.0, units = 'kg/m**3', desc = 'Density of seawater')
        self.add_param('depth', val = 10.0, units = 'm', desc = 'Depth of submerged tube')

        #Define outputs
        self.add_output('m_pylon',
                        val=0.0,
                        units='kg',
                        desc='total mass of the pylon')
        self.add_output('m_prime',
                        val=100.0,
                        units='kg/m',
                        desc='total mass of the tube per unit length')
        self.add_output('von_mises',
                        val=0.0,
                        units='Pa',
                        desc='max Von Mises Stress')
        self.add_output('total_material_cost',
                        val=0.0,
                        units='USD/m',
                        desc='cost of materials')
        self.add_output('R', val=0.0, units='N', desc='Force on pylon')
        self.add_output('delta',
                        val=0.0,
                        units='m',
                        desc='max deflection inbetween pylons')
        self.add_output('dx',
                        val=500.0,
                        units='m',
                        desc='distance between pylons')
        self.add_output('t_crit',
                        val=0.0,
                        units='m',
                        desc='Minimum tunnel thickness for buckling')

    def solve_nonlinear(self, params, unknowns, resids):
        '''total material cost = ($/kg_tunnel)*m_prime + ($/kg_pylon)*m_pylon*(1/dx)
        m_prime = mass of tunnel per unit length = rho_tube*pi*((r+t)^2-r^2)
        m_pylon = mass of single pylon = rho_pylon*pi*(r_pylon^2)*h

        Constraint equations derived from yield on buckling conditions

        '''

        rho_tube = params['rho_tube']
        E_tube = params['E_tube']
        v_tube = params['v_tube']
        alpha_tube = params['alpha_tube']
        dT_tube = params['dT_tube']
        unit_cost_tube = params['unit_cost_tube']
        g = params['g']
        tube_area = params['tube_area']
        #r = params['r']
        t = params['t']
        m_pod = params['m_pod']
        p_tunnel = params['p_tunnel']
        p_atm = params['p_atm']
        Su_pylon = params['Su_pylon']
        sf = params['sf']
        rho_pylon = params['rho_pylon']
        E_pylon = params['E_pylon']
        r_pylon = params['r_pylon']
        unit_cost_pylon = params['unit_cost_pylon']
        h = params['h']
        vac_weight = params['vac_weight']
        rho_water = params['rho_water']
        depth = params['depth']

        #Compute intermediate variable
        r = np.sqrt(tube_area/np.pi)
        p_ambient = p_atm + rho_water*g*depth
        #print(r)
        q = (rho_water*np.pi*((r+t)**2.0)*g) - (rho_tube * np.pi * (((r + t)**2) - (r**2)) * g)  #Calculate distributed load
        dp = p_ambient - p_tunnel  #Calculate delta pressure
        I_tube = (np.pi / 4.0) * ((
            (r + t)**4) - (r**4))  #Calculate moment of inertia of tube

        m_prime = rho_tube * np.pi * (((r + t)**2) - (r**2))  #Calculate mass per unit length
        dx = ((2 * (Su_pylon / sf) * np.pi *(r_pylon**2))) / q  #Calculate dx
        M = (q * ((dx**2) / 8.0))  #Calculate max moment
        sig_theta = (dp * r) / t  #Calculate hoop stress
        sig_axial = ((dp * r) / (2 * t)) + (
            (M * r) / I_tube
        ) + alpha_tube * E_tube * dT_tube  #Calculate axial stress
        von_mises = np.sqrt((((sig_theta**2) + (sig_axial**2) + (
            (sig_axial - sig_theta)**2)) /
                    2.0))  #Calculate Von Mises stress
        m_pylon = rho_pylon * np.pi * (r_pylon**
                                    2) * h  #Calculate mass of single pylon

        # unknowns['total_material_cost'] = (unit_cost_tube * (rho_tube * np.pi * ((
        #     (r + t)**2) - (r**2)))) + (unit_cost_pylon * m_pylon * (1 / (
        #         ((2 * (Su_pylon / sf) * np.pi * (r_pylon**2)) - m_pod * g) /
        #         (m_prime * g))))
        unknowns['total_material_cost'] = (unit_cost_tube * (rho_tube * np.pi * ((
            (r + t)**2) - (r**2)))) + (unit_cost_pylon * m_pylon)/dx 
        unknowns['m_prime'] = m_prime
        unknowns['von_mises'] = von_mises
        unknowns['delta'] = (5.0 * q * (dx**4)) / (384.0 * E_tube * I_tube)
        unknowns['m_pylon'] = m_pylon
        unknowns['R'] = .5 * m_prime * dx * g + .5 * m_pod * g
        unknowns['dx'] = dx
        unknowns['t_crit'] = r * ((
            (4.0 * dp * (1.0 - (v_tube**2))) / E_tube)**(1.0 / 3.0))


if __name__ == '__main__':

    top = Problem()
    root = top.root = Group()

    params = (#('r', 1.1, {'units': 'm'}),
              ('tube_area', 53.134589, {'units': 'm**2'}),
              ('t', 5.0, {'units': 'm'}),
              ('r_pylon', 1.1, {'units': 'm'}),
              ('Su_tube', 152.0e6, {'units': 'Pa'}),
              ('sf', 1.5),
              ('p_ambient', 850.0, {'units': 'Pa'}),
              ('p_tunnel', 101300.0, {'units': 'Pa'}),
              ('v_tube', .3),
              ('rho_tube', 7820.0, {'units': 'kg/m**3'}),
              ('rho_pylon', 2400.0, {'units': 'Pa'}),
              ('Su_pylon', 40.0e6, {'units': 'Pa'}),
              ('E_pylon', 41.0e9, {'units': 'Pa'}),
              ('h', 10.0, {'units': 'm'}),
              ('m_pod', 3100.0, {'units': 'kg'})
              )
    root.add('input_vars', IndepVarComp(params))
    root.add('p', UnderwaterOptimization())

    root.add('con1', ExecComp(
        'c1 = ((Su_tube/sf) - von_mises)'))  #Impose yield stress constraint for tube
    root.add('con2', ExecComp(
        'c2 = t - t_crit'))  #Impose buckling constraint for tube dx = ((pi**3)*E_pylon*(r_pylon**4))/(8*(h**2)*rho_tube*pi*(((r+t)**2)-(r**2))*g)

    #root.connect('input_vars.r', 'p.r')
    root.connect('input_vars.tube_area', 'p.tube_area')
    root.connect('input_vars.t', 'p.t')
    root.connect('input_vars.r_pylon', 'p.r_pylon')

    root.connect('input_vars.Su_tube', 'con1.Su_tube')
    root.connect('input_vars.sf', 'con1.sf')
    root.connect('p.von_mises', 'con1.von_mises')

    root.connect('input_vars.t', 'con2.t')
    root.connect('p.t_crit', 'con2.t_crit')

    root.p.deriv_options['type'] = "cs"
    # root.p.deriv_options['form'] = 'forward'
    root.p.deriv_options['step_size'] = 1.0e-10

    top.driver = ScipyOptimizer()
    top.driver.options['optimizer'] = 'SLSQP'

    top.driver.add_desvar('input_vars.t', lower=.001, scaler=100.0)
    top.driver.add_desvar('input_vars.r_pylon', lower=.001, scaler = 1.0)
    top.driver.add_objective('p.total_material_cost', scaler = 1.0e-4)
    top.driver.add_constraint('con1.c1', lower=0.0, scaler=1000.0)
    top.driver.add_constraint('con2.c2', lower=0.0)



    top.setup()
    top['p.p_tunnel'] = 850.0
    # top['p.m_pod']= 10000.0
    top['p.h'] = 10.0
    top['p.depth'] = 10.0

    import csv

    f = open('/Users/kennethdecker/Desktop/Paper figures/water_structural_trades.csv', 'wt')
    writer = csv.writer(f)
    writer.writerow(('A_tube', 'dx', 'cost'))


    A_tube = np.linspace(20.0, 50.0, num = 30)

    dx = np.zeros((1, len(A_tube)))
    t_tube = np.zeros((1, len(A_tube)))
    r_pylon = np.zeros((1, len(A_tube)))
    cost = np.zeros((1,len(A_tube)))

    for i in range(len(A_tube)):
        top['input_vars.tube_area'] = A_tube[i]

        top.run()

        dx[0,i] = top['p.dx']
        t_tube[0,i] = top['p.t']
        r_pylon[0,i] = top['p.r_pylon']
        cost[0,i] = top['p.total_material_cost']

        print(top['p.r_pylon'])
        print(r_pylon[0,i])

        # writer.writerow((A_tube[i], dx[0,i], cost[0,i]))

    # f.close()
    plt.hold(True)
    # plt.subplot(211)
    line1, = plt.plot(A_tube, dx[0,:], 'b-', linewidth = 2.0, label = 'm_pod = 10000 kg')
    plt.xlabel('Tube Area (m^2)', fontsize = 12, fontweight = 'bold')
    plt.ylabel('Pylon Spacing (m)', fontsize = 12, fontweight = 'bold')
    plt.grid('on')
    plt.show()
    plt.subplot(211)
    line1, = plt.plot(A_tube, t_tube[0,:], 'b-', linewidth = 2.0, label = 'm_pod = 10000 kg')
    # plt.xlabel('Tube Area (m^2)', fontsize = 12, fontweight = 'bold')
    plt.ylabel('tube thickness (m)', fontsize = 12, fontweight = 'bold')
    plt.grid('on')
    plt.subplot(212)
    line1, = plt.plot(A_tube, r_pylon[0,:], 'b-', linewidth = 2.0, label = 'm_pod = 10000 kg')
    plt.xlabel('Tube Area (m^2)', fontsize = 12, fontweight = 'bold')
    plt.ylabel('Pylon Radius (m)', fontsize = 12, fontweight = 'bold')
    plt.grid('on')
    plt.show()

    # plt.plot(A_tube, dx[0,:])
    # plt.xlabel('Tube Area')
    # plt.ylabel('pylon spacing')
    # plt.show()

    # plt.plot(A_tube, total_material_cost[0,:])
    # plt.xlabel('Tube Area')
    # plt.ylabel('Cost per unit length')
    # plt.show()


    # R_buckle = ((np.pi**3) * top['p.E_tube'] *
    #             (top['p.r_pylon']**4)) / (16 * (top['p.h']**2))
    # print('Optimizer pylon radius %f' % top['p.r_pylon'])
    # if top['p.R'] < R_buckle:
    #     print('Pylon buckling constraint is satisfied')
    # else:
    #     r_pylon_new = ((R_buckle * 16 * (top['p.h']**2)) / (
    #         (np.pi**3) * top['p.E_tube']))**.25
    #     print(
    #         'Optimizer value did not satisfy pylon buckling condition. Pylon radius set to minimum buckling value')
    #     print('new pylon radius is %f m' % r_pylon_new)


    print('\n')
    print('total material cost per m is $%6.2f/km' %
          (top['p.total_material_cost'] * (1.0e3)))
    print('pylon radius is %6.3f m' % top['p.r_pylon'])
    print('tube thickness is %6.4f mm' % (top['p.t'] * (1.0e3)))
    print('mass per unit length is %6.2f kg/m' % top['p.m_prime'])
    print('vertical force on each pylon is %6.2f kN' % (top['p.R'] / (1.0e3)))
    print('Von Mises stress is %6.3f MPa' % (top['p.von_mises'] / (1.0e6)))
    print('distance between pylons is %6.2f m' % top['p.dx'])
    print('max deflection is %6.4f mm' % (top['p.delta'] * (1.0e3)))
    print('critical thickness %6.4f' % top['p.t_crit'])
    print('\n')
    print('con1 = %f' % top['con1.c1'])
    print('con2 = %f' % top['con2.c2'])

    if top['con1.c1'] < 0.0:
        print('con1 not satisfied')
    elif top['con2.c2'] < 0.0:
        print('con2 not satisfied')
    else:
        print('Yield constraints are satisfied')