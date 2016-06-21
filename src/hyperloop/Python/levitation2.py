import numpy as np
from os import remove
from math import pi, atan, sin, cos, e, log

from openmdao.api import Group, Component, IndepVarComp, Problem, ExecComp
from openmdao.api import ScipyOptimizer, NLGaussSeidel, Newton


class Drag(Component):
    """Calculate minimum Drag based on given track parameters"""

    def __init__(self):
        super(Drag, self).__init__()

        # Pod Inputs
        self.add_param('mpod', val=0.375, units='kg', desc='pod mass')
        self.add_param('Br', val=0.64, units='Tesla', desc='residual magnetic flux')
        self.add_param('M', val=4.0, desc='number magnets per Halbach')
        self.add_param('d', val=0.1524, units='m', desc='thickness of magnet')
        self.add_param('rhomag', val=7.5, units='g/cm^3', desc='resistivity of aluminum at 20C')
        self.add_param('lpod', val=3, units='m', desc='Length of Pod')
        self.add_param('gamma', val=1, units='', desc='Percent Factor')

        # Track Inputs (laminated track)
        self.add_param('Pc', val=0.21, units='m', desc='width of track')
        self.add_param('Nt', val=0.005, units='m', desc='width of conductive strip')
        self.add_param('Ns', val=1, desc='number of laminated sheets')
        self.add_param('delta_c', val=0.0005334, units='m', desc='single layer thickness')
        self.add_param('strip_c', val=0.0105, units='m', desc='center strip spacing')
        self.add_param('rc', val=171.3, units='Ohm-m', desc='electric resistivity')
        self.add_param('mu0', val=4*pi*10**-7, units='hy/m', desc='Permeability of Free Space')

        # Pod/Track Relation Inputs
        self.add_param('vpod', val=350.0, units='m/s', desc='pod velocity')
        self.add_param('y', val=17., units='m', desc='levitation height')

        # outputs
        # pod outputs
        self.add_output('lam', val=0.00635*4., units='m', desc='Halbach wavelength')
        self.add_output('B0', val=0.9, units='T', desc='Halbach peak strength')
        self.add_output('A', val=0.4, units='m', desc='Total Area of Magnets')
        self.add_output('mmag', val=5., units='kg', desc='Mass of Magnets')

        #
        self.add_output('omega', val=2650., units ='rad/s', desc ='frequency')
        self.add_output('Fyu', val=17., units ='N', desc ='levitation force')
        self.add_output('Fxu', val=17., units ='N', desc ='drag force')
        self.add_output('LDratio', val=17., units='', desc='Lift to Drag Ratio')


    def solve_nonlinear(self, params, unknowns, resids):

        #Pod Parameters
        vpod = params['vpod']
        Br = params['Br']
        M = params['M']
        y = params['y']
        d = params['d']
        gamma = params['gamma']
        lpod = params['lpod']
        rhomag = params['rhomag']

        #Track Parameters
        Pc = params['Pc']
        Nt = params['Nt']
        Ns = params['Ns']
        delta_c = params['delta_c']
        strip_c = params['strip_c']
        rc = params['rc']
        mu0=params['mu0']

        # Compute Intermediate Variables
        R = rc*Pc/(delta_c*Nt*Ns)*10**-10 # Track Resistance

        lam = M*d # Compute Wavelength
        B0 = Br*(1.-e**(-2.*pi*d/lam))*((sin(pi/M))/(pi/M)) # Compute Peak Field Strength
        L = mu0*Pc/(4*pi*strip_c/lam) # Compute Track Inductance
        A = Pc*lpod*gamma # Compute Magnet Area
        mmag = rhomag*A*d # Compute Magnet Mass


        if vpod == 0:
            omega = 0
            Fyu = 0
            Fxu = 0
        else:
            omega = 2*pi*vpod/lam  # Compute Induced Frequency
            Fyu = (B0**2.*Pc/(4.*pi*L*strip_c/lam))*(1./(1.+(R/(omega*L))**2))*e**(-4.*pi*(y)/lam)*A
            Fxu = (B0**2.*Pc/(4.*pi*L*strip_c/lam))*((R/(omega*L))/(1.+(R/(omega*L))**2.))*e**(-4*pi*(y)/lam)*A
            LDratio = Fyu/Fxu

        unknowns['lam'] = lam
        unknowns['B0'] = B0
        unknowns['L'] = L
        unknowns['A'] = A
        unknowns['mmag'] = mmag
        unknowns['omega'] = omega
        unknowns['Fyu'] = Fyu
        unknowns['Fxu'] = Fxu


    # def linearize(self, params, unknowns, resids):
    #
    #     # Define Parameters
    #
    #     J = {}
    #     J['Fxu','d'] =
    #     J['Fxu', 'gamma'] =
    #
    #     return J

class Mass(Component):
    """Calculate minimum magnet mass based on given track parameters"""

    def __init__(self):
        super(Mass, self).__init__()

        # Pod Inputs
        self.add_param('d', val=0.1524, units='m', desc='thickness of magnet')
        self.add_param('rhomag', val=7.5, units='g/cm^3', desc='resistivity of aluminum at 20C')
        self.add_param('lpod', val=3, units='m', desc='Length of Pod')
        self.add_param('gamma', val=1, units='', desc='Percent Factor')

        # Track Inputs (laminated track)
        self.add_param('Pc', val=0.21, units='m', desc='width of track')

        # outputs
        # pod outputs
        self.add_output('A', val=0.4, units='m', desc='Total Area of Magnets')
        self.add_output('mmag', val=5., units='kg', desc='Mass of Magnets')

    def solve_nonlinear(self, params, unknowns, resids):  # params, unknowns, residuals

        # Pod Parameters
        d = params['d']
        gamma = params['gamma']
        lpod = params['lpod']
        rhomag = params['rhomag']

        # Track Parameters
        Pc = params['Pc']

        # Compute Intermediate Variables
        A = Pc * lpod * gamma  # Compute Magnet Area
        mmag = rhomag * A * d  # Compute Magnet Mass

        unknowns['A'] = A
        unknowns['mmag'] = mmag

    # def linearize(self, params, unknowns, resids):
    #
    #     # Define Parameters
    #     rhomag = params['rhomag']
    #     Pc = params['Pc']
    #     lpod = params['lpod']
    #     d = params['d']
    #
    #
    #     J = {}
    #     J['mmag', 'd'] = rhomag*d
    #     J['mmag', 'gamma'] = rhomag*Pc*lpod*d
    #
    #     return J

if __name__ == "__main__":

    top = Problem()
    root = top.root = Group()

    #Define Parameters
    params = (
        ('d', 1.0, {'units' : 'm'}),
        ('gamma', 1.0, {'units' : ''}),
        ('mpod', 1300.0 ,{'units' : 'kg'})
    )

    root.add('input_vars', IndepVarComp(params))

    root.add('p', Drag())
    root.add('q',Mass())

    #Constraint
    root.add('con1', ExecComp('c1 = Fyu - mpod * 9.81'))

    root.connect('p.Fyu', 'con1.Fyu')

    root.connect('input_vars.d', 'p.d')
    root.connect('input_vars.d', 'q.d')

    root.connect('input_vars.gamma','p.gamma')
    root.connect('input_vars.gamma', 'q.gamma')

    root.fd_options['force_fd'] = True
    root.fd_options['form'] = 'central'
    root.fd_options['step_size'] = 1.0e-10

    top.driver = ScipyOptimizer()
    top.driver.options['optimizer'] = 'SLSQP'

    # Design Variables
    top.driver.add_desvar('input_vars.d', lower=1.0, upper=15.0)
    top.driver.add_desvar('input_vars.gamma', lower=0.0, upper=1.0)

    top.driver.add.objective('Fxu')
    top.driver.add.objective('mmag')

    top.setup()
    top.run()

    print('\n')
    print('Lift to Drag Ratio is %f' % )
    print('Total Magnet Area is %f m' % )
    print('Total Magnet Weight is %f kg' % )



