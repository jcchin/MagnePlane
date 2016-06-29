"""
Current Levitation Code
Outputs minimum mass and area of magnets needed for levitation at desired breakpoint velocity.
Outputs Halbach array wavelength, track resistance, and inductance can be used to find
drag force at any velocity using given pod weight.
"""
from math import pi, sin, e
import numpy as np
from openmdao.api import Group, Component, IndepVarComp, Problem, ExecComp
from openmdao.api import ScipyOptimizer


class Drag(Component):
    """
    Current Drag Calculation very rough. Needs refinement.
    Default parameters taken from Inductrack I.
    Calculates minimum drag given at set breakpoint velocity desired with given track parameters.

    Params
    ------
    mpod : float
        Mass of the hyperloop pod. Default value is 3000.
    Br : float
        Strength of the Neodynium Magnets. Default value is 1.48.
    M : float
        Number of Magnets per Halbach Array. Default value is 4
    d : float
        Thickness of Magnet. Default value is 0.15.
    g : float
        Gravitational Acceleration. Default value is 9.81.
    lpod : float
        Length of the Hyperloop pod. Default value is 22.
    gamma : float
        Percent factor used in Area. Default value is 1.
    Pc : float
        Width of track. Default value is 3.
    w : float
        Width of magnet array. Default value is 3.
    spacing : float
        Halbach Spacing Factor. Default value is 0.0.
    Nt : float
        Width of conductive strip. Default value is .005.
    Ns : float
        Number of laminated sheets. Default value is 1.0.
    delta_c : float
        Single layer thickness. Default value is .005.
    strip_c : float
        Center strip spacing. Default value is .0105.
    rc : float
        Electric resistance. Default value is 1.713*10**-8.
    mu0 : float
        Permeability of Free Space. Default value is 4*pi*10^-7.
    vb : float
        Breakpoint velocity of the pod. Default value is 23.
    y : float
        Levitation height. Default value is .01.

    Returns
    -------
    lam : float
        Wavelength of the Halbach Array. Default value is 0.0.
    R : float
        Resistance of the track. Default value is 0.0
    L : float
        Inductance of the track. Default value is 0.0.
    B0 : float
        Halbach Peak Strength. Default value is 0.0.
    A : float
        Total area of the magnetic array. Default value is 0.0.
    omegab : float
        Breakpoint frequency of the induced current. Default value is 0.0.
    Fyu : float
        Levitation force. Default value is 0.0.
    Fxu : float
        Drag force. Default value is 0.0.
    LDratio : float
        Lift to drag ratio. Default value is 0.0.

    Notes
    -----
    [1] Friend, Paul. Magnetic Levitation Train Technology 1. Thesis. Bradley University, 2004. N.p.: n.p., n.d. Print.
    """

    def __init__(self):
        super(Drag, self).__init__()

        # Pod Inputs
        self.add_param('mpod', val=3000.0, units='kg', desc='Pod Mass')
        self.add_param('Br', val=1.48, units='T', desc='Residual Magnetic Flux')
        self.add_param('M', val=4.0, desc='Number of Magnets per Halbach Array')
        self.add_param('d', val=0.15, units='m', desc='Thickness of magnet')
        self.add_param('lpod', val=22, units='m', desc='Length of Pod')
        self.add_param('gamma', val=1.0, desc='Percent Factor')
        self.add_param('w', val=3., units='m', desc='Width of magnet array')
        self.add_param('spacing', val=0.0, units='m', desc='Halbach Spacing Factor')

        # Track Inputs (laminated track)
        self.add_param('Pc', val=3., units='m', desc='Width of Track')
        self.add_param('Nt', val=0.005, units='m', desc='Width of Conductive Strip')
        self.add_param('Ns', val=1.0, desc='Number of Laminated Sheets')
        self.add_param('delta_c', val=0.0005334, units='m', desc='Single Layer Thickness')
        self.add_param('strip_c', val=0.0105, units='m', desc='Center Strip Spacing')
        self.add_param('rc', val=1.713*10**-8, units='ohm-m', desc='Electric Resistivity')
        self.add_param('mu0', val=4.*pi*10**-7, units='ohm*s/m', desc='Permeability of Free Space')

        # Pod/Track Relation Inputs
        self.add_param('vb', val=23.0, units='m/s', desc='Desired Breakpoint Velocity')
        self.add_param('y', val=0.01, units='m', desc='Levitation Height')
        self.add_param('g', val=9.81, units='m/s**2', desc='Gravity')

        # outputs
        self.add_output('lam', val=0.0, units='m', desc='Halbach wavelength')
        self.add_output('L', val=0.0, units='ohm*s', desc='Inductance')
        self.add_output('B0', val=0.0, units='T', desc='Halbach peak strength')
        self.add_output('A', val=0.0, units='m**2', desc='Total Area of Magnets')
        self.add_output('omegab', val=0.0, units ='rad/s', desc='Breakpoint Frequency')
        self.add_output('Fyu', val=0.0, units='N', desc='Levitation Force')
        self.add_output('Fxu', val=0.0, units='N', desc='Drag Force')
        self.add_output('LDratio', val=0.0, desc='Lift to Drag Ratio')
        self.add_output('R', val=0.0, units='ohm', desc='Resistance')



    def solve_nonlinear(self, params, unknowns, resids):

        #Pod Parameters
        vb = params['vb']  # Breakpoint Velocity
        Br = params['Br']  # Magnet Strength
        M = params['M']  # Number of Magnets per Wavelength
        y = params['y']  # Desired Levitation Height
        d = params['d']  # Magnet Thickness
        gamma = params['gamma']  # Area Scalar
        lpod = params['lpod']  # Length of the Pod
        w = params['w']  # Width of Magnetic Array
        spacing = params['spacing']

        #Track Parameters
        Pc = params['Pc']  # Width of Track
        Nt = params['Nt']  # Width of Conductive Strip
        Ns = params['Ns']  # Number of Laminated Layers
        delta_c = params['delta_c']  # Single Layer Thickness
        strip_c = params['strip_c']  # Center Strip Spacing
        rc = params['rc']  # Electrical Resistivity of Material
        mu0 = params['mu0']  # Permeability of Free Space

        # Compute Intermediate Variables
        R = rc*Pc/(delta_c*Nt*Ns)  # Track Resistance

        lam = M*d + spacing  # Compute Wavelength
        B0 = Br*(1.-np.exp(-2.*pi*d/lam))*((sin(pi/M))/(pi/M))  # Compute Peak Field Strength
        L = mu0*Pc/(4*pi*strip_c/lam)  # Compute Track Inductance
        A = w*lpod*gamma  # Compute Magnet Area

        if vb == 0:
            omegab = 0
            Fyu = 0
            Fxu = 0
            LDratio = 0
        else:
            omegab = 2*pi*vb/lam  # Compute Induced Frequency
            Fyu = (B0**2.*w/(4.*pi*L*strip_c/lam))*(1./(1.+(R/(omegab*L))**2))*e**(-4.*pi*(y)/lam)*A  # Compute Lift Force
            Fxu = (B0**2.*w/(4.*pi*L*strip_c/lam))*((R/(omegab*L))/(1.+(R/(omegab*L))**2.))*e**(-4*pi*(y)/lam)*A  #Compute Drag Force
            LDratio = Fyu/Fxu  # Compute Lift to Drag Ratio

        unknowns['lam'] = lam
        unknowns['B0'] = B0
        unknowns['L'] = L
        unknowns['A'] = A
        unknowns['omegab'] = omegab
        unknowns['Fyu'] = Fyu
        unknowns['Fxu'] = Fxu
        unknowns['LDratio'] = LDratio
        unknowns['R'] = R

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
    """
    Current Magnet Mass Calculation very rough. Needs refinement.
    Default parameters taken from Inductrack I.
    Calculates minimum magnet mass needed at set breakpoint velocity desired with given track parameters.

    Params
    ------
    d : float
        Thickness of Magnet. Default value is 0.15.
    rhomag : float
        Density of Magnet. Default value is 7500.
    lpod : float
        Length of the Hyperloop pod. Default value is 22.
    gamma : float
        Percent factor used in Area. Default value is 1.
    w : float
        Width of magnet array. Default value is 3.
    costperkg : flost
        Cost of the magnets per kilogram. Default value is 44.

    Returns
    -------
    A : float
        Total area of the magnetic array. Default value is 0.0
    mmag : float
        Mass of the permanent magnets. Default value is 0.0.
    cost : float
        Total cost of the magnets. Default value is 0.0.

    Notes
    -----
    [1] Friend, Paul. Magnetic Levitation Train Technology 1. Thesis. Bradley University, 2004. N.p.: n.p., n.d. Print.
    """

    def __init__(self):
        super(Mass, self).__init__()

        # Pod Inputs
        self.add_param('d', val=0.15, units='m', desc='Thickness of Magnet')
        self.add_param('rhomag', val=7500, units='kg/m**3', desc='Density of Magnet')
        self.add_param('lpod', val=22, units='m', desc='Length of Pod')
        self.add_param('gamma', val=1.0, desc='Percent Factor')
        self.add_param('costperkg', val=44., units= 'USD/kg',desc='Cost of Magnet per Kilogram')
        self.add_param('w', val=3., units='m', desc='Width of Magnet Array')

        # outputs
        self.add_output('A', val=0.0, units='m', desc='Total Area of Magnets')
        self.add_output('mmag', val=0.0, units='kg', desc='Mass of Magnets')
        self.add_output('cost', val = 0.0, units='USD', desc='Cost of Magnets')

    def solve_nonlinear(self, params, unknowns, resids):  # params, unknowns, residuals

        #Parameters
        d = params['d'] # Thickness of Magnet
        w = params['w'] # Width of Magnet Array
        gamma = params['gamma'] # Area Scalar
        lpod = params['lpod'] # Length of Pod
        rhomag = params['rhomag'] # Density of Magnets
        costperkg = params['costperkg'] # Cost per kg of Magnet

        # Compute Intermediate Variables
        A = w * lpod * gamma  # Compute Magnet Area
        mmag = rhomag * A * d  # Compute Magnet Mass
        cost = mmag * costperkg # Compute Total Magnet Cost

        unknowns['A'] = A
        unknowns['mmag'] = mmag
        unknowns['cost'] = cost

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

    # Define Parameters
    params = (
        ('d', .05, {'units': 'm'}),
        ('gamma', .05),
        ('mpod', 3000.0, {'units': 'kg'}),
        ('g', 9.81, {'units': 'm/s**2'})
    )

    # Add Components
    root.add('input_vars', IndepVarComp(params))
    root.add('p', Drag())
    root.add('q', Mass())

    # Constraint Equation
    root.add('con1', ExecComp('c1 = (Fyu - mpod * g)/1e5'))

    # Connect
    root.connect('input_vars.mpod', 'p.mpod')
    root.connect('p.mpod', 'con1.mpod')
    root.connect('p.Fyu', 'con1.Fyu')
    root.connect('input_vars.g', 'p.g')
    root.connect('p.g', 'con1.g')

    root.connect('input_vars.d', 'p.d')
    root.connect('input_vars.d', 'q.d')

    root.connect('input_vars.gamma', 'p.gamma')
    root.connect('input_vars.gamma', 'q.gamma')

    # Finite Difference
    root.deriv_options['type'] = 'fd'
    root.fd_options['form'] = 'forward'
    root.fd_options['step_size'] = 1.0e-6

    # Optimizer Driver
    top.driver = ScipyOptimizer()
    top.driver.options['optimizer'] = 'COBYLA'

    # Design Variables
    top.driver.add_desvar('input_vars.d', lower=.01, upper=.15, scaler=100)
    top.driver.add_desvar('input_vars.gamma', lower=0.1, upper=1.0)

    # Add Constraint
    top.driver.add_constraint('con1.c1', lower = 0.0)

    #Problem Objective
    root.add('obj_cmp', ExecComp('obj = Fxu + mmag'))
    root.connect('p.Fxu', 'obj_cmp.Fxu')
    root.connect('q.mmag', 'obj_cmp.mmag')

    top.driver.add_objective('obj_cmp.obj')

    top.setup(check=True)


    top.run()

    # from openmdao.devtools.partition_tree_n2 import view_tree
    # view_tree(top)

    # Print Outputs
    print('\n')
    print('Lift to Drag Ratio is %f' % top['p.LDratio'])
    print('Fyu is %f' % top['p.Fyu'])
    print('Fxu is %f' % top['p.Fxu'])
    print('c1 is %f' % top['con1.c1'])
    print('Total Magnet Area is %f m^2' % top['p.A'])
    print('Total Magnet Weight is %f kg' % top['q.mmag'])
    print('Total Magnet Cost is $%f' % top['q.cost'])
    print('d is %f m' % top['p.d'])
    print('Gamma is %f' % top['p.gamma'])
    print('\n')
    print('R is %f m' % top['p.R'])
    print('L is %12.12f m' % top['p.L'])
    print('B0 is %f m' % top['p.B0'])