from __future__ import print_function

import numpy as np
from openmdao.api import IndepVarComp, Component, Group, Problem,

class SampleMission(Component):
	'''
	Notes
	-------
	This component outputs relevant mission parameters assuming a flat trajectore from LA to SF
	'''

	def __init__(self):
		super(SampleMission(), self).__init__()

		self.add_param('p_tunnel', 850.0, desc = 'Tunnel pressure', units = 'Pa')
		self.add_param('T_tunnel', 320.0, desc = 'T_tunnel', units = 'K')
		self.add_param('Cd', .2, desc = 'Pod drag coefficient', units = 'unitless')
		self.add_param('S', 40.0, desc = 'Pod planform area', units = 'm**2')