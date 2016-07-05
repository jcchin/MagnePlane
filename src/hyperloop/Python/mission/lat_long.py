from __future__ import print_function

import numpy as np
from openmdao.api import IndepVarComp, Component, Group, Problem
from pointer.components import EOMComp

class LatLong(EOMComp):

	def __init__(self, grid_data, Re=6378.137, lon_origin=-121.0, lat_origin=35.0):
		super(LatLong, self).__init__(grid_data, time_units='s')

		nn = grid_data['num_nodes']

		self.add_param('x', val=np.zeros(nn), desc='X-component in N-E-D frame', units='km')
		self.add_param('y', val=np.zeros(nn), desc='Y-component in N-E-D frame', units='km')

		self._Re = Re
		self._lon_origin = lon_origin
		self._lat_origin = lat_origin

		self.add_output('lat', shape=(nn,), desc='Latitude at input coordinate', units='deg')
		self.add_output('long', shape=(nn,), desc='Longitude at an input coordinate', units='deg')

	def solve_nonlinear(self, p, u, r):
		x = p['x']
		y = p['y']

		lat_origin_rad = self._lat_origin * (np.pi/180.0)
		long_origin_rad = self._lon_origin * (np.pi/180.0)

		lat_rad = lat_origin_rad + (x/self._Re)
		long_rad = long_origin_rad + (y/(self._Re*np.cos(lat_rad)))

		u['lat'] = lat_rad*(180.0/np.pi)
		u['long'] = long_rad*(180.0/np.pi)

if __name__ == '__main__':
	top = Problem()
	root = top.root = Group()
	grid_data = { 'num_nodes': 1 }

	params = (('x', 100.0, {'units' : 'km'}),
			('y',100.0, {'units' : 'km'}),
		)

	root.add('input_vars', IndepVarComp(params), promotes = ['x', 'y'])
	root.add('p', LatLong(grid_data=grid_data), promotes = ['x', 'y'])

	top.setup()
	top.run()

	print('\n')
	print('Point is at %f degrees N and %f degrees E' % (top['p.lat'], top['p.long']))
	