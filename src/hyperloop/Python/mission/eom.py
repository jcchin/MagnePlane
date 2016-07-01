from __future__ import print_function, division

import numpy as np

from pointer.components import EOMComp

"""
A test demonstration of the capabilities of **pointer**.

This is the Brachistochrone optimal control problem for
a bead traversing a frictionless guide-wire from point (0 m,10 m)
to point (10 m, 5 m).

The initial guess is seeded with the approximate optimal solution.
"""


class MagneplaneEOM(EOMComp):
    """ The equations of motion for the MagnePlane.

    These equations of motion are derived from a brachistochrone in 3D
    with the addition of thrust and drag along the body x-axis.
    """

    def __init__(self, grid_data):
        super(MagneplaneEOM, self).__init__(grid_data, time_units='s')

        self.deriv_options['type'] = 'fd'

        nn = grid_data['num_nodes']

        self.add_param('x',
                       desc='horizontal component of position, positive north',
                       units='m',
                       eom_state=True)

        self.add_param('y',
                       desc='horizontal component of position, positive east',
                       units='m',
                       eom_state=True)

        self.add_param('z',
                       desc='vertical component of position, positive down',
                       units='m',
                       eom_state=True)

        self.add_param('v',
                       desc='velocity',
                       units='m/s',
                       eom_state=True)

        self.add_param('g',
                       desc='gravitational acceleration',
                       units='m/s/s')

        self.add_param('theta',
                       desc='elevation angle, up from horizontal',
                       units='rad')

        self.add_param('psi',
                       desc='azimuth angle, clockwise from north',
                       units='rad')

        self.add_param('F_thrust',
                       val=np.ones(nn),
                       desc='thrust force',
                       units='N')

        self.add_param('F_drag',
                       val=np.ones(nn),
                       desc='drag force',
                       units='N')

        self.add_param('mass',
                       val=np.ones(nn),
                       desc='pod mass',
                       units='kg')

    def solve_nonlinear(self, params, unknowns, resids):

        theta = params['theta']
        psi = params['psi']

        ctheta = np.cos(theta)
        stheta = np.sin(theta)

        g = params['g']
        v = params['v']

        T = params['F_thrust']
        D = params['F_drag']
        mass = params['mass']

        unknowns['dXdt:v'][:] = -g*stheta + (T-D)/mass
        unknowns['dXdt:x'][:] = v*ctheta*np.cos(psi)
        unknowns['dXdt:y'][:] = v*ctheta*np.sin(psi)
        unknowns['dXdt:z'][:] = -v*stheta
