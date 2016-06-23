from __future__ import print_function

from math import pi, sqrt
from openmdao.api import IndepVarComp, Component, Group, Problem, ExecComp
from openmdao.api import ScipyOptimizer, NLGaussSeidel, Newton

class PodMach(Component):
    """

    Notes
    -----

        Estimates tube diameter, inlet diameter, and compressor power
        Will optimize some sort of cost function based on pod mach number
        Many parameters are currently taken from hyperloop alpha, pod sizing analysis

    Parameters
    ----------

        pod mach : float
            pod Mach number
        tunnel radius : float
            Assumes tunel radius from alpha paper. Will take tunnel radius from aero analysis.
        pod area : float
            Assumes pod mass from alpha paper.  Will take pod area from geometry analysis


    Returns
    -------

        tunnel area : float
            will return optimal tunnel area based on pod Mach number

    References
    ----------

        USA. NASA. Buckling of Thin-Walled Circular Cylinders. N.p.: n.p., n.d. Web. 13 June 2016.

    """

    def __init__(self):
        super(PodMach, self).__init__()

        self.add_param('gam', val = 1.4, desc = 'ratio of specific heats')
        self.add_param('R', val = 287.0, units = 'J/(kg*K)', desc = 'Ideal gas constant')
        self.add_param('BF', val = .9, desc = 'A_diff/A_pod')
        self.add_param('A_pod', val = 1.4, units = 'm^2', desc = 'pod area')
        self.add_param('L', val = 22.0, units='m', desc = 'pod length')
        self.add_param('prc', val = 12.5, units='m^2', desc = 'pressure ratio of a compressor')
        self.add_param('p_ambient', val = 850.0, units='Pa', desc = 'ambient pressure')
        self.add_param('T_ambient', val = 298.0, units='K', desc = 'ambient temperature')
        self.add_param('mu', val = 1.846e-5, units = 'kg/(m*s)', desc = 'dynamic viscosity')
        self.add_param('M_duct', val = .95, desc = 'maximum pod mach number')
        self.add_param('M_diff', val = .6, desc = 'maximum pod mach number befor entering the compressor')
        self.add_param('cp', val = 1009.0, units='J/(kg*K)', desc = 'specific heat')
        self.add_param('r_pod', val = .668, units = 'm', desc = 'pod radius')
        self.add_param('r_tube', val = 1.11, units = 'm^2', desc = 'pod area')
        self.add_param('delta_star', val = .07, units = 'm', desc ='Boundary layer displacement thickness')

        self.add_param('M_pod', val = .8, desc = 'pod mach number')

        self.add_output('pwr_comp', val = 0.0, units = 'W', desc = 'Compressor Power')
        self.add_output('A_inlet', val = 0.0, units = 'm^2', desc = 'Pod inlet area')
        self.add_output('A_tube', val = 0.0, units = 'm^2', desc = 'tube area')
        self.add_output('A_bypass', val = 0.0, units = 'm^2', desc = 'bypass area')
        self.add_output('A_duct_eff', val = 0.0, units = 'm^2', desc = 'effective duct area')
        self.add_output('A_diff', val = 0.0, units = 'm^2', desc = 'Area after diffuser')
        self.add_output('Re', val = 0.0, desc = 'Reynolds Number')

    def solve_nonlinear(self, params, unknowns, resids):
        gam = params['gam']
        BF = params['BF']
        A_pod = params['A_pod']
        L = params['L']
        prc = params['prc']
        p_ambient = params['p_ambient']
        R = params['R']
        T_ambient = params['T_ambient']
        mu = params['mu']
        M_duct = params['M_duct']
        M_diff = params['M_diff']
        cp = params['cp']
        r_pod = params['r_pod']
        delta_star = params['delta_star']
        M_pod = params['M_pod']

        def mach_to_area(M1, M2, gam):
            '''(A2/A1) = f(M2)/f(M1)'''
            A_ratio = (M1/M2)*(((1.0+((gam-1.0)/2.0)*(M2**2.0))/(1.0+((gam-1.0)/2.0)*(M1**2.0)))**((gam+1.0)/(2.0*(gam-1.0))))
            return A_ratio

        #Define intermediate variables
        rho_inf = p_ambient/(R*T_ambient)
        U_inf = M_pod * ((gam*R*T_ambient)**.5)

        Re = (rho_inf*U_inf*L)/mu

        A_diff = BF*A_pod

        if M_pod > M_diff:
            A_inlet = A_diff*mach_to_area(M_diff, M_pod, gam)
        else:
            A_inlet = A_diff

        eps = mach_to_area(M_pod, M_duct, gam)
        A_tube = (A_pod+pi*(((r_pod+delta_star)**2.0)-(r_pod**2.0))-(eps*A_inlet))/((1.0+(eps**.5))*(1.0-(eps**.5)))
        pwr_comp = (rho_inf*U_inf*A_inlet)*cp*T_ambient*(1+((gam-1)/2)*(M_pod**2))*((prc**((gam-1)/gam))-1)
        A_bypass = A_tube - A_inlet
        A_duct_eff = A_tube - A_pod - pi*(((r_pod+delta_star)**2)-(r_pod**2))

        unknowns['pwr_comp'] = pwr_comp
        unknowns['A_inlet'] = A_inlet
        unknowns['A_tube'] = A_tube
        unknowns['A_bypass'] = A_bypass
        unknowns['A_duct_eff'] = A_duct_eff
        unknowns['A_diff'] = A_diff
        unknowns['Re'] = Re

if __name__ == '__main__':
    top = Problem()
    root = top.root = Group()

    params = (
        ('M_pod', .8),
        ('gam', 1.4)
    )

    root.add('input_vars', IndepVarComp(params))
    root.add('p', PodMach())

    root.connect('input_vars.M_pod', 'p.M_pod')
    root.connect('input_vars.gam', 'p.gam')

    root.deriv_options['type'] = 'fd'
    root.deriv_options['form'] = 'central'
    root.deriv_options['step_size'] = 1.0e-8

    top.setup()

    top.run()

    print('\n')
    print('Pod Mach number is %f' % top['p.M_pod'])
    print('Area of the tube is %f m^2' % top['p.A_tube'])
    print('Compressor power is %f W' % top['p.pwr_comp'])
    print('Area of the inlet is %f m^2'% top['p.A_inlet'])
    print('Area after diffuser is %f m^2' % top['p.A_diff'])
    print('Bypass area is %f m^2' % top['p.A_bypass'])
    print('Effective duct area is %f m^2' % top['p.A_duct_eff'])
    print('Reynolds number is %f' % top['p.Re'])