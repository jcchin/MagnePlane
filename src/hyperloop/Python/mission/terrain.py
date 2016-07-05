from __future__ import print_function, division

import numpy as np

from openmdao.api import Group, Problem, Group
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

        nn = grid_data['num_nodes']

        self.add_param('x', desc='horizontal component of position, positive north', units='m', eom_state=True)
        self.add_param('y', desc='horizontal component of position, positive east', units='m', eom_state=True)
        self.add_param('z', desc='vertical component of position, positive down', units='m', eom_state=True)

        #usgs_file.files['xx', 'yy', 'zz']
        usgs_file = np.load('usgs_data.npz')

        self.add_output('elev', shape=(nn,), desc='terrain elevation at the given point', units='m/s/s')
        self.add_output('alt', shape=(nn,), desc='ground-relative altitude of the track', units='m')

        self.interpolant = interpolate.RectBivariateSpline(usgs_file['xx'], usgs_file['yy'], usgs_file['zz'])
    
    def solve_nonlinear(self, params, unknowns, resids):
        #convert x/y to lat/lon, then feed into interpolant
        unknowns['elev'] = self.interpolant(params['lat'],params['long'])
        unknowns['alt'] = -z - unknowns['elev']

if __name__ == "__main__":
    root = Group()
    p = Problem(root)

    griddata = { 'num_nodes': 1 }

    p.root.add('EOMcomp', TerrainElevationComp(griddata))

    p.setup()
    p.root.list_connections()
    p.run()

    print('Elev : %f' % p['elev'])
    print('Alt : %f' % p['alt'])
