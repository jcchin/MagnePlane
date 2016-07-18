from __future__ import print_function

import numpy as np
from math import sin, cos
from openmdao.api import IndepVarComp, Component, Group, Problem

class BodyFrameAcceleration(Component):
	''' 
	Super sweet docs, bro
	'''
	def __init__(self):

		super(BodyFrameAcceleration, self).__init__()

		self.add_param('psi', val = 0.0, desc = 'Pod yaw angle', units = 'rad')
		self.add_param('theta', val = 0.0, desc = 'Pod pitch angle', units = 'rad')
		self.add_param('phi', val = 0.0, desc = 'Pod roll angle', units = 'rad')
		self.add_param('omega', val = np.array([0.0, 0.0, 0.0]), desc = 'Pod angular velocity', units = 'm/s')
		self.add_param('v', val = 0.0, desc = 'Pod linear velocity', units = 'm/s')
		self.add_param('a_linear', val = 9.81, desc = 'Pod linear acceleration', units = 'm/s**2')
		self.add_param('L_pod', val = 20.5, desc = 'Pod yaw angle', units = 'rad')

		self.add_output('a_body', val = np.matrix([[0.0], [0.0], [0.0]]), desc = 'Body fram acceleration vector', units = 'm/s**2')

	def solve_nonlinear(self, p, u, r):

		omega = p['omega']
		v = p['v']
		a_linear = p['a_linear']
		L_pod = p['L_pod']

		#Calculate rotational matrix from inertial frame into the body frame
		# BN = np.matrix([[cos(theta)*cos(psi), cos(theta)*sin*(psi), -sin(theta)], \
		# 	[sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(phi)*sin(psi), sin(phi)*cos(theta)], \
		# 	[cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi), cos(phi)*cos(theta)]])

		b1 = np.array([1.0, 0.0, 0.0])
		r = -L_pod*b1

		u['a_body'] = a_linear*b1 + v*np.cross(omega, b1) - np.cross(omega, np.cross(omega,r))

if __name__ == '__main__':

	top = Problem()
	root = top.root = Group()
	root.add('p', BodyFrameAcceleration())

	top.setup()
	top.run()

	print('\n')
	print('Body frame acceleration vector is \n')
	print(top['p.a_body'])



