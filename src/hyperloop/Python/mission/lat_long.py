from __future__ import print_function

import numpy as np
from openmdao.api import IndepVarComp, Component, Group, Problem

class LatLong(Component):

	def __init__(self):
		super(LatLong, self).__init__()

		self.add_param('x', val=0.0, desc='X-component in N-E-D frame', units='km')
		self.add_param('y', val=0.0, desc='Y-component in N-E-D frame', units='km')
		self.add_param('long_origin', val=-121.0, desc='Longitude of window origin', units='deg')
		self.add_param('lat_origin', val=35.0, desc='Latitude of window origin', units='deg')
		self.add_param('R_E', val=6378, desc='Radius of the Earth', units='km')

		self.add_output('lat', val=0.0, desc='Latitude at input coordinate', units='deg')
		self.add_output('long', val=0.0, desc='Longitude at an input coordinate', units='deg')

	def solve_nonlinear(self, p, u, r):
		x = p['x']
		y = p['y']
		R_E = p['R_E']

		lat_origin_rad = p['lat_origin'] * (np.pi/180.0)
		long_origin_rad = p['long_origin'] * (np.pi/180.0)

		lat_rad = lat_origin_rad + (x/R_E)
		long_rad = long_origin_rad + (y/(R_E*np.cos(lat_rad)))

		u['lat'] = lat_rad*(180.0/np.pi)
		u['long'] = long_rad*(180.0/np.pi)

if __name__ == '__main__':
	top = Problem()
	root = top.root = Group()

	params = (('x', 100.0, {'units' : 'km'}),
			('y',100.0, {'units' : 'km'}),
		)

	root.add('input_vars', IndepVarComp(params), promotes = ['x', 'y'])
	root.add('p', LatLong(), promotes = ['x', 'y'])

	top.setup()
	top.run()

	print('\n')
	print('Point is at %f degrees N and %f degrees E' % (top['p.lat'], top['p.long']))




