import numpy as np
from math import pi, atan

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
        inspired by: http://mitrocketscience.blogspot.com/search/label/maglev"""

    def __init__(self):
        super(Lift, self).__init__()
        self.ln_solver = LinearGaussSeidel()

        # Pod Inputs
        self.add_param('Br', val=1.42, units='Tesla', desc='residual magnetic flux')
        self.add_param('d', val=0.012, units='m', desc='magnet thickness')
        self.add_param('N', val=16.0, desc='number magnets')
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
        self.add_output('omegaOsc', val=47, units ='rad/s', desc ='Oscillation frequency')
        self.add_output('vOsc', val=47, units ='m/s', desc ='Oscillation velocity')
        # breakpoint analysis

        # transition analysis

        # summary outputs


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

    def solve_nonlinear(self, params, unknowns, resids):

        edge = params['edge']
        height = params['height']

        unknowns['n'] = params['N'] / 4.0
        unknowns['omega'] = params['rpm'] * 2.0 * pi / 60.0
        unknowns['area_ring'] = pi * ((((params['dia_out']+edge)/2.)**2) - (((params['dia_out']-edge)/2.)**2))
        unknowns['area_mag'] = unknowns['area_ring'] * params['fill_frac']
        unknowns['vol_mag'] = unknowns['area_ring'] * params['t_plate']
        unknowns['f'] = unknowns['n'] * unknowns['omega'] / (2 * pi)
        #                                 *(ATAN((B6*B6)  /(2*   B20* SQRT(4*B20^2+B6^2+B6^2)))             -ATAN((B6*B6) /(2*(B6+B20)*     SQRT(4*(B6+B20)^2+B6^2+B6^2))))
        unknowns['B'] = (params['Br']/pi)*(atan((edge**2)/(2*height*(4*height**2+edge**2+edge**2)**0.5))-atan((edge*edge)/(2*(edge+height)*(4*(edge+height)**2+edge**2+edge**2)**0.5)))*params['halbach']
        unknowns['rho'] = params['rho0'] * (1+params['alpha']*(params['T']-20))
        unknowns['delta'] = (unknowns['rho']/(pi * params['mu'] * unknowns['f']))**0.5
        unknowns['P_norm'] = ((pi * unknowns['B'] * params['t_plate'] * unknowns['f'])**2) / (6. * params['k'] * unknowns['rho'] * params['Den'])
        unknowns['P'] = unknowns['P_norm'] * unknowns['vol_mag'] * params['Den']


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
