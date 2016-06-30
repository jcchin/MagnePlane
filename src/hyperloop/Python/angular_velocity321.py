from __future__ import print_function

import numpy as np
from openmdao.api import IndepVarComp, Component, Group, Problem, 

class AngularVelocity321(Component):
    """
	Notes
	------

	Evaluates the body frame angular velocity from 321 Euler angles and their derivatives
	Units are in radians and radians/s

    Params
    ------
    Yaw : float
    	Yaw angle (3-axis rotation) of body frame with respect to the inertial NED frame. Default value is 0.0 rad
    Pitch : float
    	Pitch angle (2-axis rotation) of body fram with respect to the inertial NED frame. Default value is 0.0 rad
    Roll : float
    	Roll angle (1-axis rotation) of body fram with respect to the inertial NED frame. Default value is 0.0 rad
    Yaw rate : float
    	Yaw rate of pod body frame. Default value is .01 rad/s
    Pitch rate : float
    	Pitch rate of pod body frame. Default value is .01 rad/s
    Roll rate : float
    	Roll rate of pod body frame. Default value is 0.0 rad/s


    Returns
    -------
    
    Angular velocity : float
    	Returns the body fame angular velocity of the pod in rad/s
    """

    def __init__(self):
    	super(AngularVelocity321, self).__init__()

    	self.add_param('psi', val = 0.0, units = 'rad', desc = 'Pod yaw angle')
    	self.add_param('theta', val = 0.0, units = 'rad', desc = 'Pod pitch angle')
    	self.add_param('phi', val = 0.0, units = 'rad', desc = 'Pod roll angle')
    	self.add_param('psi_dot', val = 0.0, units = 'rad', desc = 'Pod yaw rate')
    	self.add_param('theta_dot', val = 0.0, units = 'rad', desc = 'Pod pitch rate')
    	self.add_param('phi_dot', val = 0.0, units = 'rad', desc = 'Pod roll rate')

    	self.add_output('omega_b', val = np.matrix('0.0; 0.0; 0.0'), units = 'rad/s', desc = 'Angular velocity vector')

    def solve_nonlinear(self, p, u, r):
    	"""
		Notes
		------

		omega = [[s(psi)*s(theta), c(psi), 0], [c(psi)*s(theta), -s(psi), 0], [c(theta), 0,1]] * [[phi], [theta], [psi]]

	    Params
	    ------
	    Yaw : float
	    	Yaw angle (3-axis rotation) of body frame with respect to the inertial NED frame. Default value is 0.0 rad
	    Pitch : float
	    	Pitch angle (2-axis rotation) of body fram with respect to the inertial NED frame. Default value is 0.0 rad
	    Roll : float
	    	Roll angle (1-axis rotation) of body fram with respect to the inertial NED frame. Default value is 0.0 rad
	    Yaw rate : float
	    	Yaw rate of pod body frame. Default value is .01 rad/s
	    Pitch rate : float
	    	Pitch rate of pod body frame. Default value is .01 rad/s
	    Roll rate : float
	    	Roll rate of pod body frame. Default value is 0.0 rad/s


	    Returns
	    -------
	    
	    Angular velocity : float
	    	Returns the body fame angular velocity of the pod in rad/s
	    """

    	psi = p['psi']
    	theta = p['theta']
    	phi = p['phi']
    	psi_dot = p['psi_dot']
    	theta_dot = p['theta_dot']
    	phi_dot = p['phi_dot']

    	B = np.matrix([[-np.sin(theta), 0.0, 1.0], [np.sin(phi)*np.cos(theta), np.cos(phi), 0.0], [np.cos(phi)*np.cos(theta), -np.sin(phi), 0]])

    	u['omega_b'] = B * np.matrix([[phi_dot], [theta_dot], [psi_dot]])


if __name__ == '__main__':
	top = Problem()
	root = top.root = Group()

	params = (
		('psi', 0.0, {'units' : 'rad'}),
		('theta', 0.0, {'units' : 'rad'}),
		('phi', 0.0, {'units' : 'rad'}),
		('psi_dot', 0.1, {'units' : 'rad'}),
		('theta_dot', 0.1, {'units' : 'rad'}),
		('phi_dot', 0.0, {'units' : 'rad'})
		)

	root.add('input_vars', IndepVarComp(params), promotes = ['psi', 'theta', 'phi', 'psi_dot', 'theta_dot', 'psi_dot'])
	root.add('p', AngularVelocity321(), promotes = ['psi', 'theta', 'phi', 'psi_dot', 'theta_dot', 'psi_dot', 'omega_b'])

	top.setup()
	top.run()

	print('Bod frame angular velocity vector = ')
	print(top['omega_b'])

