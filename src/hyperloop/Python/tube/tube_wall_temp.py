"""The `TubeTemp` group represents a cycle of explicit and implicit
    calculations to determine the steady state temperature of the
    hyperloop tube.

    The `TubeWallTemp` calculates Q released/absorbed by hyperloop tube due to:
    Internal Convection, Tube Conduction, Ambient Natural Convection,
    Solar Flux In, Radiation Out

    The `TempBalance` implicit component varies the boundary temp between the
    inside and outside of the tube, until Q released matches Q absorbed.
    """
from math import log, pi, sqrt, e

from openmdao.core.group import Group, Component, IndepVarComp
from openmdao.solvers.newton import Newton
from openmdao.units.units import convert_units as cu
from openmdao.solvers.scipy_gmres import ScipyGMRES
from openmdao.api import NLGaussSeidel
from openmdao.solvers.ln_gauss_seidel import LinearGaussSeidel
from openmdao.solvers.ln_direct import DirectSolver

from pycycle import species_data
from pycycle.species_data import janaf
from pycycle.components import FlowStart
from pycycle.constants import AIR_FUEL_MIX, AIR_MIX
from pycycle.flowstation import FlowIn, PassThrough

class TempBalance(Component):
    """
    Params
    ------
    tube_area : float
        tube inner area (m^2)
    tube_thickness : float
        tube thickness (m)
    length_tube : float
        Length of the entire Hyperloop tube (m)
    num_pods : int
        Number of Pods in the Tube at a given time
    temp_boundary : float
        Average Temperature of the tube wall (K). This state variable is varied
    temp_outside_ambient : float
        Average Temperature of the outside air (K)
    nozzle_air_W : float
        mass flow rate of the air exiting the pod nozzle (kg/s)
    nozzle_air_T : float
        temp of the air exiting the pod nozzle (K)
    solar_insolation : float
        solar irradiation at sea level on a clear day
    nn_incidence_factor : float
        Non-normal incidence factor
    surface_reflectance : float
        Solar Reflectance Index
    emissivity_tube : float
        Emmissivity of the Tube
    sb_constant : float
        Stefan-Boltzmann Constant (W/((m**2)*(K**4)))
    Nu_multiplier : float
        optional fudge factor on Nusslet number to account for
        a small breeze on tube, 1 assumes no breeze

    Returns
    -------
    diameter_outer_tube : float
        outer diameter of the tube
    nozzle_q : float
        heat released from the nozzle (W)
    q_per_area_solar : float
        Solar Heat Rate Absorbed per Area (W/m**2)
    q_total_solar : float
        Solar Heat Absorbed by Tube (W)
    area_rad : float
        Tube Radiating Area (m**2)
    GrDelTL3 : float
        see [_1], Natural Convection Term (1/((ft**3)*F)))
    Pr : float
        Prandtl #
    Gr : float
        Grashof #
    Ra : float
        Rayleigh #
    Nu : float
        Nusselt #
    k : float
        Thermal conductivity (W/(m*K))
    h : float
        Heat Rate Radiated to the outside (W/((m**2)*K))
    area_convection : float
        Convection Area (m**2)
    q_per_area_nat_conv : float
        Heat Radiated per Area to the outside (W/(m**2))
    total_q_nat_conv : float
        Total Heat Radiated to the outside via Natural Convection (W)
    heat_rate_pod : float
        Heating Due to a Single Pods (W)
    total_heat_rate_pods : float
        Heating Due to All Pods (W)
    q_rad_per_area : float
        Heat Radiated to the outside per area (W/(m**2))
    q_rad_tot : float
        Heat Radiated to the outside (W)
    viewing_angle : float
        Effective Area hit by Sun (m**2)
    q_total_out : float
        Total Heat Released via Radiation and Natural Convection (W)
    q_total_in : float
        Total Heat Absorbed/Added via Pods and Solar Absorption (W)
    ss_temp_residual : float
        Energy balance to be driven to zero (K)

    Notes
    -----
    Some of the original calculations from Jeff Berton, ported and extended by
    Jeff Chin. Compatible with OpenMDAO v1.5, python 2 and 3

    References
    ----------
    .. [1] https://mdao.grc.nasa.gov/publications/Berton-Thesis.pdf pg51

    .. [2] 3rd Ed. of Introduction to Heat Transfer by Incropera and DeWitt,
    equations (9.33) and (9.34) on page 465
    """
    
    def __init__(self):
        super(TempBalance, self).__init__()
        self.add_param('ss_temp_residual', val=0.)
        self.add_state('temp_boundary', val=322.0)

    def solve_nonlinear(self, params, unknowns, resids):
        pass

    def apply_nonlinear(self, params, unknowns, resids):
        resids['temp_boundary'] = params[
            'ss_temp_residual']  #drive ss_temp_residual to 0

    def apply_linear(self, params, unknowns, dparams, dunknowns, dresids,
                     mode):

        if mode == "fwd":
            if 'ss_temp_residual' in dparams and 'temp_boundary' in dresids:

                dresids['temp_boundary'] += dparams['ss_temp_residual']

        if mode == "rev":
            if 'temp_boundary' in dresids and 'ss_temp_residual' in dparams:
                dparams['ss_temp_residual'] += dresids['temp_boundary']

class TubeWallTemp(Component):
    """ Calculates Q released/absorbed by the hyperloop tube """

    def __init__(self, thermo_data=species_data.janaf, elements=AIR_MIX):
        super(TubeWallTemp, self).__init__()
        self.deriv_options['type'] = 'fd'

        #--Inputs--
        #Hyperloop Parameters/Design Variables
        self.add_param('tube_area',
                       3.9057,
                       units='m**2',
                       desc='tube inner area')
        self.add_param('tube_thickness',
                       .05,
                       units='m',
                       desc='tube thickness')  #7.3ft
        self.add_param(
            'length_tube',
            482803.,
            units='m',
            desc='Length of entire Hyperloop')  #300 miles, 1584000ft
        self.add_param('num_pods',
                       34,
                       desc='Number of Pods in the Tube at a given time')  #
        self.add_param('temp_boundary',
                       322.0,
                       units='K',
                       desc='Average Temperature of the tube wall')  #
        self.add_param('temp_outside_ambient',
                       305.6,
                       units='K',
                       desc='Average Temperature of the outside air')  #
        #nozzle_air = FlowIn(iotype="in", desc="air exiting the pod nozzle")
        #bearing_air = FlowIn(iotype="in", desc="air exiting the air bearings")
        self.add_param('nozzle_air_W',
                        34.,
                        #units = 'kg/s',
                        desc='mass flow rate of the air exiting the pod nozzle')
        self.add_param('nozzle_air_Cp',
                        1.009,
                        units='kJ/kg/K',
                        desc='specific heat of air exiting the pod nozzle')
        self.add_param('nozzle_air_Tt',
                        34.,
                        #units = 'K',
                        desc='temp of the air exiting the pod nozzle')
        # self.add_param('bearing_air_W', 34., desc='air exiting the air bearings')
        # self.add_param('bearing_air_Cp', 34., desc='air exiting the air bearings')
        # self.add_param('bearing_air_Tt', 34., desc='air exiting the air bearings')

        #constants
        self.add_param('solar_insolation',
                       1000.,
                       units='W/m**2',
                       desc='solar irradiation at sea level on a clear day')  #
        self.add_param('nn_incidence_factor',
                       0.7,
                       desc='Non-normal incidence factor')  #
        self.add_param('surface_reflectance',
                       0.5,
                       desc='Solar Reflectance Index')  #
        self.add_param('emissivity_tube',
                       0.5,
                       units='W',
                       desc='Emmissivity of the Tube')  #
        self.add_param('sb_constant',
                       0.00000005670373,
                       units='W/((m**2)*(K**4))',
                       desc='Stefan-Boltzmann Constant')  #
        self.add_param('Nu_multiplier',
                        1.,
                        desc="fudge factor on nusslet number to account for small breeze on tube")

        #--Outputs--
        self.add_output('diameter_outer_tube', shape=1)
        self.add_output('bearing_q', shape=1)
        self.add_output('nozzle_q', shape=1)
        self.add_output('area_viewing', shape=1)
        self.add_output('q_per_area_solar',
                        350.,
                        units='W/m**2',
                        desc='Solar Heat Rate Absorbed per Area')  #
        self.add_output('q_total_solar',
                        375989751.,
                        units='W',
                        desc='Solar Heat Absorbed by Tube')  #
        self.add_output('area_rad',
                        337486.1,
                        units='m**2',
                        desc='Tube Radiating Area')  #
        #Required for Natural Convection Calcs
        self.add_output('GrDelTL3',
                        1946216.7,
                        units='1/((ft**3)*F)',
                        desc='Heat Radiated to the outside')  #
        self.add_output('Pr', 0.707, desc='Prandtl')  #
        self.add_output('Gr', 12730351223., desc='Grashof #')  #
        self.add_output('Ra', 8996312085., desc='Rayleigh #')  #
        self.add_output('Nu', 232.4543713, desc='Nusselt #')  #
        self.add_output('k',
                        0.02655,
                        units='W/(m*K)',
                        desc='Thermal conductivity')  #
        self.add_output('h',
                        0.845464094,
                        units='W/((m**2)*K)',
                        desc='Heat Radiated to the outside')  #
        self.add_output('area_convection',
                        3374876.115,
                        units='m**2',
                        desc='Convection Area')  #
        #Natural Convection
        self.add_output('q_per_area_nat_conv',
                        7.9,
                        units='W/(m**2)',
                        desc='Heat Radiated per Area to the outside')  #
        self.add_output(
                        'total_q_nat_conv',
                        286900419.,
                        units='W',
                        desc='Total Heat Radiated to the outside via Natural Convection')  #
        #Exhausted from Pods
        self.add_output('heat_rate_pod',
                        519763,
                        units='W',
                        desc='Heating Due to a Single Pods')  #
        self.add_output('total_heat_rate_pods',
                        17671942.,
                        units='W',
                        desc='Heating Due to a All Pods')  #
        #Radiated Out
        self.add_output('q_rad_per_area',
                        31.6,
                        units='W/(m**2)',
                        desc='Heat Radiated to the outside')  #
        self.add_output('q_rad_tot',
                        106761066.5,
                        units='W',
                        desc='Heat Radiated to the outside')  #
        #Radiated In
        self.add_output('viewing_angle',
                        1074256,
                        units='m**2',
                        desc='Effective Area hit by Sun')  #
        #Total Heating
        self.add_output('q_total_out',
                        286900419.,
                        units='W',
                        desc='Total Heat Released via Radiation and Natural Convection')  #
        self.add_output('q_total_in',
                        286900419.,
                        units='W',
                        desc='Total Heat Absorbed/Added via Pods and Solar Absorption')  #
        #Residual (for solver)
        self.add_output('ss_temp_residual',
                        shape=1,
                        units='K',
                        desc='Residual of T_released - T_absorbed')

    def solve_nonlinear(self, p, u, r):
        """Calculate Various Paramters"""

        u['diameter_outer_tube'] = 2*sqrt(p['tube_area']/pi) + p['tube_thickness']

        # u['bearing_q'] = cu(p['bearing_air_W'], 'lbm/s', 'kg/s') * cu(
        #     p['bearing_air_Cp'], 'Btu/(lbm*degR)', 'J/(kg*K)') * (
        #         cu(p['bearing_air_Tt'], 'degR', 'degK') - p['temp_boundary'])
        u['nozzle_q'] = cu(p['nozzle_air_W'], 'lbm/s', 'kg/s') * cu(
            p['nozzle_air_Cp'], 'Btu/(lbm*degR)', 'J/(kg*K)') * (
                cu(p['nozzle_air_Tt'], 'degR', 'degK') - p['temp_boundary'])
        #Q = mdot * cp * deltaT
        u['heat_rate_pod'] = u['nozzle_q'] #+ u['bearing_q']
        #Total Q = Q * (number of pods)
        u['total_heat_rate_pods'] = u['heat_rate_pod'] * p['num_pods']

        #Determine thermal resistance of outside via Natural Convection or forced convection
        if (p['temp_outside_ambient'] < 400):
            u['GrDelTL3'] = 41780000000000000000 * (
                (p['temp_outside_ambient'])**(-4.639)
            )  #SI units (https://mdao.grc.nasa.gov/publications/Berton-Thesis.pdf pg51)
        else:
            u['GrDelTL3'] = 4985000000000000000 * (
                (p['temp_outside_ambient'])**(-4.284)
            )  #SI units (https://mdao.grc.nasa.gov/publications/Berton-Thesis.pdf pg51)

        #Prandtl Number
        #Pr = viscous diffusion rate/ thermal diffusion rate = Cp * dyanamic viscosity / thermal conductivity
        #Pr << 1 means thermal diffusivity dominates
        #Pr >> 1 means momentum diffusivity dominates
        if (p['temp_outside_ambient'] < 400):
            u['Pr'] = 1.23 * (
                p['temp_outside_ambient']**(-0.09685)
            )  #SI units (https://mdao.grc.nasa.gov/publications/Berton-Thesis.pdf pg51)
        else:
            u['Pr'] = 0.59 * (p['temp_outside_ambient']**(0.0239))
        #Grashof Number
        #Relationship between buoyancy and viscosity
        #Laminar = Gr < 10^8
        #Turbulent = Gr > 10^9
        u['Gr'] = u['GrDelTL3'] * abs(p['temp_boundary'] - p[
            'temp_outside_ambient']) * (
                u['diameter_outer_tube']**3
            )  #JSG: Added abs incase subtraction goes negative
        #Rayleigh Number
        #Buoyancy driven flow (natural convection)
        u['Ra'] = u['Pr'] * u['Gr']
        #Nusselt Number
        #Nu = convecive heat transfer / conductive heat transfer
        if (u['Ra'] <= 10**12):  #valid in specific flow regime
            u['Nu'] = p['Nu_multiplier'] * (
                (0.6 + 0.387 * u['Ra']**(1. / 6.) /
                 (1 + (0.559 / u['Pr'])**(9. / 16.))**(8. / 27.))**2
            )  #3rd Ed. of Introduction to Heat Transfer by Incropera and DeWitt, equations (9.33) and (9.34) on page 465
        if (p['temp_outside_ambient'] < 400):
            u['k'] = 0.0001423 * (
                p['temp_outside_ambient']**(0.9138)
            )  #SI units (https://mdao.grc.nasa.gov/publications/Berton-Thesis.pdf pg51)
        else:
            u['k'] = 0.0002494 * (p['temp_outside_ambient']**(0.8152))
        #h = k*Nu/Characteristic Length
        u['h'] = (u['k'] * u['Nu']) / u['diameter_outer_tube']
        #Convection Area = Surface Area
        u['area_convection'] = pi * p['length_tube'] * u['diameter_outer_tube']
        #Determine heat radiated per square meter (Q)
        u['q_per_area_nat_conv'] = u['h'] * (
            p['temp_boundary'] - p['temp_outside_ambient'])
        #Determine total heat radiated over entire tube (Qtotal)
        u['total_q_nat_conv'] = u['q_per_area_nat_conv'] * u['area_convection']
        #Determine heat incoming via Sun radiation (Incidence Flux)
        #Sun hits an effective rectangular cross section
        u['area_viewing'] = p['length_tube'] * u['diameter_outer_tube']
        u['q_per_area_solar'] = (
            1 - p['surface_reflectance']
        ) * p['nn_incidence_factor'] * p['solar_insolation']
        u['q_total_solar'] = u['q_per_area_solar'] * u['area_viewing']
        #Determine heat released via radiation
        #Radiative area = surface area
        u['area_rad'] = u['area_convection']
        #P/A = SB*emmisitivity*(T^4 - To^4)
        u['q_rad_per_area'] = p['sb_constant'] * p['emissivity_tube'] * (
            (p['temp_boundary']**4) - (p['temp_outside_ambient']**4))
        #P = A * (P/A)
        u['q_rad_tot'] = u['area_rad'] * u['q_rad_per_area']
        #------------
        #Sum Up
        u['q_total_out'] = u['q_rad_tot'] + u['total_q_nat_conv']
        u['q_total_in'] = u['q_total_solar'] + u['total_heat_rate_pods']

        u['ss_temp_residual'] = (u['q_total_out'] - u['q_total_in']) / 1e6
        # print("u['ss_temp_residual'] ", u['ss_temp_residual'])
        # print("temp boundary", p['temp_boundary'])

class TubeTemp(Group):
    """An Assembly that computes Steady State temp"""

    def __init__(self):
        super(TubeTemp, self).__init__()

        self.add('tm', TubeWallTemp(), promotes=[
            'length_tube','tube_area','tube_thickness','num_pods',
            'nozzle_air_W','nozzle_air_Tt'])

        self.add('tmp_balance', TempBalance(), promotes=['temp_boundary'])

        #self.add('nozzle_air', FlowStart(thermo_data=janaf, elements=AIR_MIX))
        #self.add('bearing_air', FlowStart(thermo_data=janaf, elements=AIR_MIX))

        #self.connect("nozzle_air.Fl_O:tot:T", "tm.nozzle_air_Tt")
        #self.connect("nozzle_air.Fl_O:tot:Cp", "tm.nozzle_air_Cp")
        #self.connect("nozzle_air.Fl_O:stat:W", "tm.nozzle_air_W")

        self.connect('tm.ss_temp_residual', 'tmp_balance.ss_temp_residual')
        self.connect('temp_boundary', 'tm.temp_boundary')

        self.nl_solver = Newton()
        self.nl_solver.options['atol'] = 1e-5
        self.nl_solver.options['iprint'] = 1
        self.nl_solver.options['rtol'] = 1e-5
        self.nl_solver.options['maxiter'] = 50

        self.ln_solver = ScipyGMRES()
        self.ln_solver.options['atol'] = 1e-6
        self.ln_solver.options['maxiter'] = 100
        self.ln_solver.options['restart'] = 100

#run stand-alone component
if __name__ == "__main__":
    from openmdao.api import Problem

    prob = Problem()
    prob.root = Group()

    prob.root.add('tt', TubeTemp())

    params = (('P', 0.3, {'units': 'psi'}), ('T', 1500.0, {'units': 'degR'}),
              ('W', 1.0, {'units': 'lbm/s'}), ('Cp', 0.24, {'units': 'Btu/(lbm*degF)'}))
    dvars = (
        ('tube_area',3.9057),  #desc='Tube out diameter' #7.3ft
        ('tube_thickness',.05),
        ('length_tube',
         482803.),  #desc='Length of entire Hyperloop') #300 miles, 1584000ft
        ('num_pods',
         34),  #desc='Number of Pods in the Tube at a given time') #
        ('temp_boundary', 340),  #desc='Average Temperature of the tube') #
        ('temp_outside_ambient', 305.6
         )  #desc='Average Temperature of the outside air
    )
    #nozzle
    prob.root.add('des_vars', IndepVarComp(params))
    #bearings
    prob.root.add('des_vars2', IndepVarComp(params))
    #tube inputs
    prob.root.add('vars', IndepVarComp(dvars))

    #prob.root.connect('des_vars.P', 'tt.nozzle_air_P')
    prob.root.connect('des_vars.T', 'tt.nozzle_air_Tt')
    prob.root.connect('des_vars.W', 'tt.nozzle_air_W')
    prob.root.connect('des_vars.Cp', 'tt.nozzle_air_Cp')

    prob.root.connect('vars.tube_area', 'tt.tube_area')
    prob.root.connect('vars.tube_thickness', 'tt.tube_thickness')
    prob.root.connect('vars.length_tube', 'tt.length_tube')
    prob.root.connect('vars.num_pods', 'tt.num_pods')
    #prob.root.connect('vars.temp_boundary','tmp_balance.temp_boundary')
    prob.root.connect('vars.temp_outside_ambient',
                      'tt.tm.temp_outside_ambient')

    prob.setup()
    prob.root.list_connections()

    prob['des_vars.T'] = 1710.0
    prob['des_vars.P'] = 0.304434211
    prob['des_vars.W'] = 1.08
    prob['des_vars.Cp'] = 0.24

    prob.run()

    print("temp_boundary: ", prob['tt.tm.temp_boundary'])
    print("temp_resid: ", prob['tt.tm.ss_temp_residual'])

    # print "-----Completed Tube Heat Flux Model Calculations---"
    # print ""
    # print "CompressQ-{} SolarQ-{} RadQ-{} ConvecQ-{}".format(test.tm.total_heat_rate_pods, test.tm.q_total_solar, test.tm.q_rad_tot, test.tm.total_q_nat_conv )
    # print "Equilibrium Wall Temperature: {} K or {} F".format(tesparams['temp_boundary'], cu(tesparams['temp_boundary'],'degK','degF'))
    # print "Ambient Temperature:          {} K or {} F".format(test.tm.temp_outside_ambient, cu(test.tm.temp_outside_ambient,'degK','degF'))
    # print "Q Out = {} W  ==>  Q In = {} W ==> Error: {}%".format(test.tm.q_total_out,test.tm.q_total_in,((test.tm.q_total_out-test.tm.q_total_in)/test.tm.q_total_out)*100)
