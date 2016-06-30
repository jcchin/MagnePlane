from __future__ import print_function, division

import numpy as np
try:
    from openmdao.api import pyOptSparseDriver
except:
    pyOptSparseDriver = None

from openmdao.api import ScipyOptimizer
from pointer.components import Problem, Trajectory, RHS, EOMComp, CollocationPhase
from scipy import interpolate

class TerrainElevationComp(EOMComp):
    '''
    The terrain component uses the given latitude and longitude (x and y) to
    look up the elevation of the local terrain.
    '''

    def __init__(self, grid_data):
        super(TerrainElevationComp, self).__init__(grid_data, time_units='s')

        self.deriv_options['type'] = 'fd'

        self.add_param('x', desc='horizontal component of position, positive north', units='m', eom_state=True)
        self.add_param('y', desc='horizontal component of position, positive east', units='m', eom_state=True)
        self.add_param('z', desc='vertical component of position, positive down', units='m', eom_state=True)
        self.add_param('v', desc='velocity', units='m/s', eom_state=True)

        self.add_output('elev', desc='terrain elevation at the given point', units='m/s/s')
        self.add_output('alt', desc='ground-relative altitude of the track', units='m')

        self.interpolant = interpolate.interp2d(np.loadtxt('xx.txt'), np.loadtxt('yy.txt'), np.loadtxt('zz.txt'), kind = 'cubic')

    def solve_nonlinear(self, params, unknowns, resids):

        #convert x/y to lat/lon, then feed into interpolant
        unknowns['elev'] = self.interpolant(params['lat'],params['long'])
        unknowns['alt'] = -z - unknowns['elev']
    
if __name__ == "__main__":
    from openmdao.core.problem import Problem

    root = Group()
    p = Problem(root)

    p.root.add('EOMcomp', TerrainElevationComp())

    p.setup()
    p.root.list_connections()
    p.run()

    print('Elev : %f' % p['elev'])
    print('Alt : %f' % p['alt'])
