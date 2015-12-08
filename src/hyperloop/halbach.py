import numpy as np
from math import pi

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
    """Calculate Lift based on Magnet Parameters"""

    def __init__(self):
        super(Lift, self).__init__()
        self.ln_solver = LinearGaussSeidel()
        # inputs
        self.add_param('edge', val=0.25, units='inch', desc='edge length of cube magnets being used')
        self.add_param('dia_out', val=0.4, units='m', desc='diameter of largest magnet ring. Cenetered on magnets')
        self.add_param('dia_in', val=0.4, units='m', desc='diameter of smallest magnet ring')
        self.add_param('fill_frac', val=0.8, units='m', desc='fraction of area filled by magnet')
        self.add_param('nr', val=1.0, desc='number of magnet rings')
        self.add_param('N', val=16.0, desc='number magnets')
        self.add_param('rpm', val=10000.0, desc='motor rotations / minute')
        self.add_param('t_plate', val=0.25, units='inch', desc='conductive plate thickness')
        self.add_param('k', val= 1. ,desc='constant for thin sheet')
        self.add_param('rho0', val= 2.82E-08, units='Ohm-m', desc= 'resistivity of aluminum at 20C')
        self.add_param('alpha', val= 0.0039, desc='constant for linear dependence of rho on temperature of aluminum.')
        self.add_param('T', val = 26., units ='degC', desc ='temperature of aluminum')
        self.add_param('rho', val = 2.88599E-08, units ='Ohm-m' desc ='resistivity of aluminum at elevated temps')
        self.add_param('Den', val = 2700, units ='kg/m^3' desc ='density of aluminum')
        self.add_param('mu', val =  1.26E-06, units ='H/m' desc ='magnetic permeability of aluminum')

        # outputs
        self.add_output('n', val=4.0, desc='number of halbach cycles (4 magnets per cycle)')
        self.add_output('area_mag', val=0.4, units='m', desc='actual magnetized area')
        self.add_output('vol_mag', val=0.4, units='m', desc='volume of plate with eddy current')
        self.add_output('omega', val=16.0, units='rad/s', desc='rotation speed')
        self.add_output('area_ring', val=0.0, units='lbm/s', desc='area of ring that contains all magnets')

        self.add_output('height', val=0.005, units='m', desc='rotor lift height')
        self.add_output('br', val=4.0, desc='residual magnetic flux')
        self.add_output('B', val=4.0, units='T', desc='magnetic field at plate (Tesla')
        self.add_output('f', val=4.0, ,units='Hz', desc='magnet rotation frequency')






        self.add_output('P_norm', val = 11214.5582, units ='W/kg' desc ='watts per mass dissipated in aluminum')
        self.add_output('P', val = 155.7289266, units ='W' desc ='watts dissipated in aluminum')
        self.add_output('delta', val = 0.003311459 units ='m' desc ='skin depth equation for good conductors. \
                                                point at which current density has fallen to 0.37 of surface. \
                                                make sure thickness of plate is at least 2*delta')

    def solve_nonlinear(self, params, unknowns, resids):
        unknowns['n'] = params['N'] / 4.0
        unknowns['omega'] = params['rpm'] * 2.0 * pi / 60.0
        unknowns['f'] = params['n'] * params['omega'] / (2 * pi)
        unknowns['rho'] = params['rho0'] * (1+params['alpha']*(params['T']-20))
        unknowns['delta'] = (params['rho']/pi * params['mu'] unknowns['f'])**0.5

if __name__ == "__main__":
    from openmdao.core.problem import Problem


    root = Group()
    root.add('lift', Lift())

    p = Problem(root)

    p.setup()
    p.run()

    print(p['lift.n'])