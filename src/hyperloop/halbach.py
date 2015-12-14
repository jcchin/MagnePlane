import numpy as np
from math import pi, atan, sin, e, log

from openmdao.core.group import Group, Component, IndepVarComp
from openmdao.solvers.newton import Newton
from openmdao.solvers.scipy_gmres import ScipyGMRES

from pycycle.components import Compressor, Shaft, FlowStart, Inlet, Nozzle, Duct
from pycycle.species_data import janaf
from pycycle.connect_flow import connect_flow
from pycycle.constants import AIR_FUEL_MIX, AIR_MIX

from openmdao.solvers.ln_gauss_seidel import LinearGaussSeidel
from openmdao.solvers.ln_direct import DirectSolver



class Lift(Component):
    """Calculate Lift based on Magnet Parameters
        inspired by: Jed Storey - http://mitrocketscience.blogspot.com/search/label/
        and: Paul R. Friend - http://cegt201.bradley.edu/projects/proj2004/maglevt1/reason.html"""

    def __init__(self):
        super(Lift, self).__init__()
        self.ln_solver = LinearGaussSeidel()

        # Pod Inputs
        self.add_param('Br', val=1.42, units='Tesla', desc='residual magnetic flux')
        self.add_param('d', val=0.012, units='m', desc='magnet thickness')
        self.add_param('N', val=16.0, desc='number magnets')
        self.add_param('M', val=16.0, desc='number magnets per halbach')
        self.add_param('Is', val=3.0, desc='number vertically oriented magnets')
        self.add_param('lambda', val=0.012, units='m', desc='halbach wavelength')
        self.add_param('edge', val=0.25*0.0254, units='m', desc='edge length of cube magnets being used')
        self.add_param('mass', val=0.4, units='kg', desc='pod mass')

        # Track Inputs (laminated track)
        self.add_param('delta_c', val=0.05*0.0254, units='m', desc='single layer thickness')
        self.add_param('strip_c', val=0.012, units='m', desc='center strip spacing')
        self.add_param('Pc', val=0.12, units='m', desc='width of track')
        self.add_param('rc', val=0.012, units='Ohm-m', desc='electric resistivity')
        self.add_param('Nt', val=0.005, units='m', desc='width of conductive strip')
        self.add_param('Ns', val=4, desc='number of laminated sheets')

        # Inductive Loading
        self.add_param('al', val=0.0005, units='m', desc='conductor bundle height loaded')
        self.add_param('wf', val=0.12, units='m', desc='total ferrite tile width')

        # Pod/Track Relation
        self.add_param('y1', val=0.12, units='m', desc='rolling clearance')
        self.add_param('veloc', val=0.12, units='m/s', desc='pod velocity')

        self.add_param('dia_out', val=0.0406, units='m', desc='diameter of largest magnet ring. Cenetered on magnets')
        self.add_param('dia_in', val=0.0406, units='m', desc='diameter of smallest magnet ring')
        self.add_param('fill_frac', val=0.8, units='m', desc='fraction of area filled by magnet')
        self.add_param('nr', val=1.0, desc='number of magnet rings')
        self.add_param('rpm', val=10000.0, desc='motor rotations / minute')
        self.add_param('t_plate', val=0.25*0.0254, units='m', desc='conductive plate thickness')
        self.add_param('k', val=1., desc='constant for thin sheet')
        self.add_param('rho0', val=2.82E-08, units='Ohm-m', desc= 'resistivity of aluminum at 20C')
        self.add_param('alpha', val=0.0039, desc='constant for linear dependence of rho on temperature of aluminum.')
        self.add_param('T', val=26., units ='degC', desc ='temperature of aluminum')
        self.add_param('Den', val=2700, units ='kg/m^3', desc ='density of aluminum')
        self.add_param('mu', val=1.26E-06, units ='H/m', desc ='magnetic permeability of aluminum')
        self.add_param('halbach', val=1.8,desc= 'field multiplier for using halbach')
        self.add_param('height', val=0.0051, units='m', desc='rotor lift height')

        # outputs
        # pod outputs
        self.add_output('B0', val=0.9, units='T', desc='halbach peak strength')
        self.add_output('area_mag', val=0.4, units='m', desc='actual magnetized area')
        self.add_output('pforce', val=5., units ='N', desc ='required levitation force')
        # system outputs
        self.add_output('l1d', val=5.7E-08, units ='Henrys', desc ='distributed "1 turn" inductance')
        self.add_output('li', val=0., units ='Henrys', desc ='added inductance from loading')
        self.add_output('l1', val=5.7E-08, units ='Henrys', desc ='one turn inductance')
        self.add_output('r1', val=0.0007, units ='Ohm', desc ='one turn resistance')
        self.add_output('rl_pole', val=12250, units ='rad/s', desc ='R/L Pole')
        self.add_output('omegaOsc', val=47., units ='rad/s', desc ='Oscillation frequency')
        self.add_output('vOsc', val=0.41, units ='m/s', desc ='Oscillation velocity')
        # breakpoint analysis
        self.add_output('vb', val=23., units ='m/s', desc ='levitation breakpoint')
        self.add_output('sb', val=52., units ='mi/h', desc ='levitation breakpoint speed mph')
        self.add_output('omegab', val=2650., units ='rad/s', desc ='frequency breakpoint')
        self.add_output('Fxb', val=17., units ='N', desc ='drag force at breakpoint')
        self.add_output('l2db', val=0.2, desc ='lift to drag at breakpoint')
        # transition analysis
        self.add_output('vt', val=47, units ='m/s', desc ='transition velocity')
        self.add_output('st', val=52., units ='mi/h', desc ='levitation transition speed mph')
        self.add_output('omegat', val=2650., units ='rad/s', desc ='frequency transition')
        self.add_output('Fxyt', val=17., units ='N', desc ='drag force at transition')
        self.add_output('Lht', val=17., units ='m', desc ='levitation height')
        self.add_output('l2dt', val=0.2, desc ='lift to drag at transition')
        # user specified output point
        self.add_output('omegau', val=2650., units ='rad/s', desc ='frequency')
        self.add_output('su', val=52., units ='mi/h', desc ='levitation speed mph')
        self.add_output('Fyu', val=17., units ='N', desc ='levitation force')
        self.add_output('Fxu', val=17., units ='N', desc ='drag force')
        self.add_output('Lhu', val=17., units ='m', desc ='levitation height')
        self.add_output('Fyuf', val=17., units ='N', desc ='fixed levitation force')
        self.add_output('Fxuf', val=17., units ='N', desc ='fixed drag force')
        self.add_output('l2du', val=0.2, desc ='lift to drag')


        self.add_output('rho', val=2.88599E-08, units ='Ohm-m', desc ='resistivity of aluminum at elevated temps')
        self.add_output('n', val=4.0, desc='number of halbach cycles (4 magnets per cycle)')

        self.add_output('vol_mag', val=0.4, units='m', desc='volume of plate with eddy current')
        self.add_output('omega', val=16.0, units='rad/s', desc='rotation speed')
        self.add_output('area_ring', val=0.0, units='lbm/s', desc='area of ring that contains all magnets')
        self.add_output('B', val=0.172, units='T', desc='magnetic field at plate (Tesla')
        self.add_output('f', val=4.0, units='Hz', desc='magnet rotation frequency')
        self.add_output('P_norm', val = 11214.5582, units ='W/kg', desc ='watts per mass dissipated in aluminum')
        self.add_output('P', val = 155.7289266, units ='W', desc ='watts dissipated in aluminum')
        self.add_output('delta', val = 0.003311459, units ='m', desc ='skin depth equation for good conductors. \
                                                point at which current density has fallen to 0.37 of surface. \
                                                make sure thickness of plate is at least 2*delta')

    def solve_nonlinear(self, p, u, r): # params, unknowns, residuals

        edge = p['edge']
        height = p['height']
        lambdaa = p['lambda']

        u['B0'] = p['Br'] * (1. - e**(-2.*pi*edge/lambdaa))*((sin(pi/p['M']))/(pi/p['M']))
        u['pforce'] = p['mass'] * 9.81

        u['l1d'] = 4*pi*10**-7*p['Pc']/(4*pi*p['strip_c']/lambdaa)

        trackArea = p['delta_c'] * p['Nt']
        R = p['rc']*p['Pc']/(u['area_mag']*p['Ns'])*10**-10
        levc = p['delta_c'] * p['Ns']/2.
        p['y1'] = 0.01 + levc


        Q = (e**(pi*p['al']/lambdaa)+e**(-pi*p['al']/lambdaa))/(e**(pi*p['al']/lambdaa)-e**(-pi*p['al']/lambdaa))
        u['li'] = (p['wf']/p['Pc'])*(Q-1.)*u['l1d']
        #     levs = (p['edge']/p['Pc'])*(p['l1d']/(Ll+p['l1d']));
        #     levsl = (p['l1d']/(Ll+p['l1d']));
        levsl = 1.

        u['l1'] = u['l1d'] + u['li']

        # Fringe Effects
        if p['Is'] == 0:
            levsf = 1
        else:
            levsf = 1-1/(2*p['Is'])
        # Scale Factor
        levs = levsf*levsl

        u['rl_pole'] = u['r1']/u['l1']

        # Oscillation Frequency and Velocity
        u['omegaOsc'] = (4*pi*9.81/lambdaa)**0.5
        u['vOsc'] = u['omegaOsc']*lambdaa/(2*pi)

        # Break Point Calculations:
        u['omegab'] = u['r1']/(u['l1']*(((((levs*u['B0']**2.*p['edge']/(4*pi*u['l1']*p['delta_c']/lambdaa))*u['area_mag']*e**(-4*pi*p['y1']/lambdaa))/u['pforce'])**0.5)-1))
        u['vb'] = (u['omegab']*lambdaa)/(2*pi)
        u['sb'] = u['vb']*(100/2.54)*1/(5280*12)*60*60;
        u['Fxb'] = levs*(u['B0']**2.*p['edge']/(4*pi*u['l1']*p['delta_c']/lambdaa))*((u['r1']/(u['omegab']*u['l1']))/(1+(u['r1']/(u['omegab']*u['l1']))**2.))*e**(-4*pi*p['y1']/lambdaa)*u['area_mag']
        u['l2db'] = u['omegab']*u['l1']/u['r1']

        # Transition Calculations (Lift = Drag):
        u['omegat'] = u['r1']/u['l1']
        u['vt'] = u['omegat']*lambdaa/(2.*pi)
        u['st'] = u['vt']*(100./2.54)*1./(5280.*12.)*60.*60.
        u['Lht'] = log((u['pforce']/(levs*(u['B0']**2.*p['edge']/(4.*pi*u['l1']*p['strip_c']/lambdaa))*(1/(1+(u['r1']/(u['omegat']*u['l1']))**2.))*u['area_mag'])))*(lambdaa/(-4.*pi))-levc
        #    Fxyt = levs*(Bo^2*w/(4*pi*L*dc/lambdaa))*((R/(u['omegat']*L))/(1+(R/(u['omegat']*L))^2))*exp(-4*pi*(Lht+levc)/lambdaa)*A;
        u['Fxyt'] = levs*(u['B0']**2.*p['edge']/(4.*pi*u['l1']*p['strip_c']/lambdaa))*((u['r1']/(u['omegat']*u['l1']))/(1.+(u['r1']/(u['omegat']*u['l1']))**2.))*e**(-4.*pi*(p['y1'])/lambdaa)*u['area_mag']
        u['l2dt'] = u['omegat']*u['l1']/u['r1']

        # User Input:
        if p['veloc'] == 0:
            u['omegau'] = 0
            u['su'] = 0
            Fyu = 0
            Fxu = 0
            Lhu = 0
            L2Du = 0
        else:
            u['omegau'] = 2*pi*p['veloc']/lambdaa
            u['su'] = p['veloc']*(100./2.54)*1./(5280.*12.)*60.*60.
            u['Lhu'] = log((u['pforce']/(levs*(u['B0']**2.*p['edge']/(4.*pi*u['l1']*p['strip_c']/lambdaa))*(1/(1+(u['r1']/(u['omegau']*u['l1']))**2.))*u['area_mag'])))*(lambdaa/(-4.*pi))-levc
            u['Fyu'] = levs*(u['B0']**2.*p['edge']/(4.*pi*u['l1']*p['delta_c']/lambdaa))*(1./(1.+(u['r1']/(u['omegau']*u['l1']))**2))*e**(-4.*pi*(u['Lhu']+levc)/lambdaa)*u['area_mag']
            u['Fxu'] = levs*(u['B0']**2.*p['edge']/(4.*pi*u['l1']*p['delta_c']/lambdaa))*((u['r1']/(u['omegau']*u['l1']))/(1+(u['r1']/(u['omegau']*u['l1']))**2.))*e**(-4*pi*p['y1']/lambdaa)*u['area_mag']
            u['Fyuf'] = levs*(u['B0']**2.*p['edge']/(4.*pi*u['l1']*p['strip_c']/lambdaa))*(1./(1.+(u['r1']/(u['omegau']*u['l1']))**2))*e**(-4.*pi*(p['y1'])/lambdaa)*u['area_mag']
            u['Fxuf'] = levs*(u['B0']**2.*p['edge']/(4.*pi*u['l1']*p['strip_c']/lambdaa))*((u['r1']/(u['omegau']*u['l1']))/(1.+(u['r1']/(u['omegau']*u['l1']))**2.))*e**(-4.*pi*(p['y1'])/lambdaa)*u['area_mag']

            u['l2du']= u['omegau']*u['l1']/u['r1']

        u['n'] = p['N'] / p['M']
        u['omega'] = p['rpm'] * 2.0 * pi / 60.0
        u['area_ring'] = pi * ((((p['dia_out']+edge)/2.)**2) - (((p['dia_out']-edge)/2.)**2))
        u['area_mag'] = u['area_ring'] * p['fill_frac']
        u['vol_mag'] = u['area_ring'] * p['t_plate']
        u['f'] = u['n'] * u['omega'] / (2 * pi)
        #                                 *(ATAN((B6*B6)  /(2*   B20* SQRT(4*B20^2+B6^2+B6^2)))             -ATAN((B6*B6) /(2*(B6+B20)*     SQRT(4*(B6+B20)^2+B6^2+B6^2))))
        u['B'] = (p['Br']/pi)*(atan((edge**2)/(2*height*(4*height**2+edge**2+edge**2)**0.5))-atan((edge*edge)/(2*(edge+height)*(4*(edge+height)**2+edge**2+edge**2)**0.5)))*p['halbach']
        u['rho'] = p['rho0'] * (1+p['alpha']*(p['T']-20))
        u['delta'] = (u['rho']/(pi * p['mu'] * u['f']))**0.5
        u['P_norm'] = ((pi * u['B'] * p['t_plate'] * u['f'])**2) / (6. * p['k'] * u['rho'] * p['Den'])
        u['P'] = u['P_norm'] * u['vol_mag'] * p['Den']


if __name__ == "__main__":
    from openmdao.core.problem import Problem


    root = Group()
    root.add('lift', Lift())

    p = Problem(root)

    p.setup()
    p.run()

    print('n: ', p['lift.n'])
    print('omega: ', p['lift.omega'])
    print('area_ring: ', p['lift.area_ring'])
    print('area_mag: ', p['lift.area_mag'])
    print('vol_mag: ', p['lift.vol_mag'])
    print('f: ', p['lift.f'])
    print('B-: ', p['lift.B'])
    print('rho: ', p['lift.rho'])
    print('delta: ', p['lift.delta'])
    print('P_norm: ', p['lift.P_norm'])
    print('P: ', p['lift.P'])

    if 2*p['lift.delta'] > p['lift.t_plate']:
        print('plate is thick enough')
    else:
        print('plate should be thicker (see delta)')
