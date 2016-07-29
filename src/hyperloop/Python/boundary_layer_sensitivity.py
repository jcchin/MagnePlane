from __future__ import print_function

import numpy as np
from openmdao.api import IndepVarComp, Component, Group, Problem, ExecComp
import matplotlib.pylab as plt

class BoundaryLayerSensitivity(Component):
	
	"""
    Notes
    ------
    Component is not a part of the system model, but is instead intended to analyze the sensitivity of tube area to the bounday layer
    thickness over the pod. Can be made to calculatee based on Reynolds number accourding to flat plate assumption or to vary boundary
    layer to account for boundary layer suction or some other version of flow control.

    Params
    ------
    gam : float
        Ratio of specific heats. Default value is 1.4
    R : float
        Ideal gas constant. Default valut is 287 J/(m*K).
    A_pod : float
        cross sectional area of the pod. Default value is 1.4 m**2. Value will be taken from pod geometry module
    comp_inlet_area : float
        Inlet area of compressor. (m**2)
    L : float
        Pod length. Default value is 22 m. Value will be taken from pod geometry module
    prc : float
        Pressure ratio across compressor inlet and outlet.  Default value is 12.5.  Value will be taken from NPSS
    p_tube : float
        Pressure of air in tube.  Default value is 850 Pa.  Value will come from vacuum component
    T_ambient : float
        Tunnel ambient temperature. Default value is 298 K.
    mu : float
        Fluid dynamic viscosity. Default value is 1.846e-5 kg/(m*s)
    M_duct : float
        Maximum Mach number allowed in the duct. Default value is .95
    M_diff : float
        Maximum Mach number allowed at compressor inlet. Default valu is .6
    cp : float
        Specific heat of fluid. Default value is 1009 J/(kg*K)
    M_pod : float
        pod Mach number. Default value is .8
    length_calc : bool
    	True calculates boundary layer thickness. False takes boundary layer thickness as an input. Default value is false.

    Returns
    -------
    A_tube : float
        will return optimal tunnel area based on pod Mach number
    pwr_comp : float
        will return the power that needs to be delivered to the flow by the compressor.  Does not account for compressor efficiency
    A_bypass : float
        will return area of that the flow must go through to bypass pod
    A_inlet : float
        returns area of the inlet necessary to slow the flow down to M_diffuser
    A_duct_eff : float
        returns effective duct area which accounts for displacement boundary layer thickness approximation
    A_diff : float
        returns area of diffuser outlet
    Re : float
        returns free stream Reynolds number
    """

	def __init__(self):
        
		super(BoundaryLayerSensitivity, self).__init__()

		self.add_param('gam', val=1.4, desc='ratio of specific heats')
		self.add_param('R',
		               val=287.0,
		               units='J/(kg*K)',
		               desc='Ideal gas constant')
		self.add_param('BF', val=.9, desc='A_diff/A_pod')
		self.add_param('A_pod', val=3.0536, units='m**2', desc='pod area')
		self.add_param('L', val=20.5, units='m', desc='pod length')
		self.add_param('prc',
		               val=12.5,
		               units='m**2',
		               desc='pressure ratio of a compressor')
		self.add_param('p_tube',
		               val=850.0,
		               units='Pa',
		               desc='ambient pressure')
		self.add_param('T_ambient',
		               val=320.0,
		               units='K',
		               desc='ambient temperature')
		self.add_param('mu',
		               val=1.846e-5,
		               units='kg/(m*s)',
		               desc='dynamic viscosity')
		self.add_param('M_duct', val=.95, desc='maximum pod mach number')
		self.add_param(
		    'M_diff',
		    val=.6,
		    desc='maximum pod mach number befor entering the compressor')
		self.add_param('cp',
		               val=1009.0,
		               units='J/(kg*K)',
		               desc='specific heat')
		self.add_param('delta_star',
		               val=.14,
		               units='m',
		               desc='Boundary layer displacement thickness')

		self.add_param('M_pod', val=.8, desc='pod mach number')
		self.add_param('length_calc', val = False, desc = 'Determines if boundary layer is calculated or left as default')

		self.add_output('pwr_comp',
		                val=0.0,
		                units='W',
		                desc='Compressor Power')
		self.add_output('A_inlet',
		                val=0.0,
		                units='m**2',
		                desc='Pod inlet area')
		self.add_output('A_tube', val=0.0, units='m**2', desc='tube area')
		self.add_output('A_bypass', val=0.0, units='m**2', desc='bypass area')
		self.add_output('A_duct_eff',
		                val=0.0,
		                units='m**2',
		                desc='effective duct area')
		self.add_output('A_diff',
		                val=0.0,
		                units='m**2',
		                desc='Area after diffuser')
		self.add_output('Re', val=0.0, desc='Reynolds Number')

	def solve_nonlinear(self, params, unknowns, resids):
		gam = params['gam']
		BF = params['BF']
		A_pod = params['A_pod']
		L = params['L']
		prc = params['prc']
		p_tube = params['p_tube']
		R = params['R']
		T_ambient = params['T_ambient']
		mu = params['mu']
		M_duct = params['M_duct']
		M_diff = params['M_diff']
		cp = params['cp']
		delta_star = params['delta_star']
		M_pod = params['M_pod']

		def mach_to_area(M1, M2, gam):
			'''(A2/A1) = f(M2)/f(M1) where f(M) = (1/M)*((2/(gam+1))*(1+((gam-1)/2)*M**2))**((gam+1)/(2*(gam-1)))'''
			A_ratio = (M1 / M2) * (((1.0 + ((gam - 1.0) / 2.0) * (M2**2.0)) /
			                        (1.0 + ((gam - 1.0) / 2.0) * (M1**2.0)))**(
			                            (gam + 1.0) / (2.0 * (gam - 1.0))))
			return A_ratio

		#Define intermediate variables
		rho_inf = p_tube / (R *
		                    T_ambient)  #Calculate density of free stream flow
		U_inf = M_pod * (np.sqrt((gam * R * T_ambient)))        #Calculate velocity of free stream flow
		r_pod = np.sqrt((A_pod / np.pi))  #Calculate pod radius

		Re = (rho_inf * U_inf *
		      L) / mu  #Calculate length based Reynolds Number


		if params['length_calc']:
			delta_star = (.04775*L)/(Re**.2)

		A_diff = BF * A_pod  #Calculate diffuser output area based on blockage factor input

		#Calculate inlet area. Inlet is necessary if free stream Mach number is greater than max compressore mach number M_diff
		if M_pod > M_diff:
		    A_inlet = A_diff * mach_to_area(M_diff, M_pod, gam)
		else:
		    A_inlet = A_diff

		eps = mach_to_area(M_pod, M_duct, gam)
		A_tube = (A_pod + np.pi * (((r_pod + delta_star)**2.0) - (r_pod**2.0)) -
		          (eps * A_inlet)) / ((1.0 + (np.sqrt(eps))) * (1.0 - (np.sqrt(eps))))
		pwr_comp = (rho_inf * U_inf * A_inlet) * cp * T_ambient * (1.0 + (
		    (gam - 1) / 2.0) * (M_pod**2)) * ((prc**((gam - 1) / gam)) - 1)
		A_bypass = A_tube - A_inlet
		A_duct_eff = A_tube - A_pod - np.pi * ((
		    (r_pod + delta_star)**2) - (r_pod**2))

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
		('delta_star', .02, {'units' : 'm'}),
		('A_pod', 2.0, {'units' : 'm**2'}),
		('L_pod', 22.0, {'units' : 'm'}),
		('length_calc', False))

	root.add('input_vars', IndepVarComp(params), promotes = ['delta_star', 'A_pod', 'L_pod', 'length_calc'])
	root.add('p', BoundaryLayerSensitivity())

	root.connect('delta_star', 'p.delta_star')
	root.connect('A_pod', 'p.A_pod')
	root.connect('L_pod', 'p.L')
	root.connect('length_calc', 'p.length_calc')

	top.setup()

	delta_star = np.linspace(.02, .12, num = 50)
	A_pod = np.linspace(2, 3, num = 3)
	L = np.linspace(20.0, 40.0, num = 50)

	if top['length_calc']:
		A_tube = np.zeros((1, len(L)))
		for i in range(len(L)):
			top['L_pod'] = L[i]

			top.run()

			A_tube[0, i] = top['p.A_tube']

		plt.plot(L, A_tube[0,:])
		plt.show()

	else:
		A_tube = np.zeros((len(A_pod), len(delta_star)))
		for i in range(len(delta_star)):
			for j in range(len(A_pod)):
				top['delta_star'] = delta_star[i]
				top['A_pod'] = A_pod[j]

				top.run()

				A_tube[j,i] = top['p.A_tube']

		plt.plot(delta_star, A_tube[0,:])
		plt.show()
