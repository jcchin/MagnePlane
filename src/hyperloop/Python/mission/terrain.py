from __future__ import print_function, division

import os

import numpy as np

from openmdao.api import Group, Problem, Group, IndepVarComp, Component
from pointer.components import Trajectory, RHS, EOMComp, CollocationPhase
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

        self.add_param('lat', shape=(nn,), desc='latitude', units='deg', eom_state=False)
        self.add_param('long', shape=(nn,), desc='longitude', units='deg', eom_state=False)
        self.add_param('z', shape=(nn,), desc='vertical component of position, positive down', units='m', eom_state=False)

        mydir = os.path.dirname(os.path.realpath(__file__))
        data_file_path = os.path.join(mydir,'usgs_data.npz')

        usgs_file = np.load(data_file_path)

        self.add_output('elev', shape=(nn,), desc='terrain elevation at the given point', units='m/s/s')
        self.add_output('alt', shape=(nn,), desc='ground-relative altitude of the track', units='m')

        self.interpolant = interpolate.RectBivariateSpline(usgs_file['Longitude'], usgs_file['Latitude'], usgs_file['Elevation'])

        import matplotlib.pyplot as plt

        xx = usgs_file['XX']
        yy = usgs_file['YY']
        zz = usgs_file['Elevation']

        plt.contourf(xx,yy,zz)
        #
        # min_lon, max_lon = usgs_file['YY'][0,0], usgs_file['YY'][479,479]
        # min_lat, max_lat = usgs_file['XX'][0,0], usgs_file['XX'][479,479]
        #
        # print(min_lon,max_lon)
        # print(min_lat,max_lat)
        #
        # lons = np.linspace(min_lon,max_lon,40)
        # lats = np.linspace(min_lat,max_lat,40)
        #
        # elevs = []
        #
        # for lat in lats:
        #     for lon in lons:
        #         elev = self.interpolant.ev(lon, lat)
        #         print(lon, lat, elev)
        #         elevs.append(elev)
        # #
        # # print(lons)
        # # print(elevs)
        # #
        # # print(len(lons))
        # # print(len(elevs))
        #
        # zz = np.reshape(elevs,(40,40))
        #
        # plt.figure()
        # plt.contourf(lons,lats,zz)
        # plt.show()

    def solve_nonlinear(self, params, unknowns, resids):
        #convert x/y to lat/lon (see Component lat_long.py), then feed into interpolant
        elev = unknowns['elev']

        for i in range(self.num_nodes):
            elev[i] = self.interpolant(params['long'][i],params['lat'][i])

        #unknowns['alt'] = -params['z'] - unknowns['elev']

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