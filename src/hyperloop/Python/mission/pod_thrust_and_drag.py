"""
Computes the total drag force acting on the pod.
Will be fed into Mission EOM component
"""
from __future__ import print_function

import numpy as np
from openmdao.api import IndepVarComp, Group, Problem

from pointer.components import EOMComp


class PodThrustAndDrag(EOMComp):
    """
    Params
    ------
    Drag coefficient : float
        Drag coefficient of pod.  Default value is .2.
        More accurate results will come from CFD
    Reference Area: float
        Reference area of the pod. Default value is 1.4 m**2.
        Value will be pulled from geometry module
    Tube Pressure : float
        Pressure of air in tube.  Default value is 850 Pa.
        Value will come from vacuum component
    Ambient Temperature : float
        Tunnel ambient temperature. Default value is 298 K.
    Ideal Gas Constant : float
        Ideal gas constant. Default valut is 287 J/(m*K).
    Magnetic Drag : float
        Drag force from magnetic levitation in N. Default value is 150 N.
        Value will come from levitation analysis
    Pod Speed : float
        Speed of the pod.  Default value is 335 m/s.

    Returns
    -------
    Drag : float
        Total drag force acting on pod. Default value is 0.0.
    """

    def __init__(self, grid_data):
        super(PodThrustAndDrag, self).__init__(grid_data, time_units='s')

        self.deriv_options['type'] = 'fd'
        nn = grid_data['num_nodes']

        self.add_param('Cd',
                       val=.2*np.ones(nn),
                       desc='Drag Coefficient')

        self.add_param('S',
                       val=1.4*np.ones(nn),
                       units='m**2',
                       desc='Frontal Area')

        self.add_param('p_tube',
                       val=850.0*np.ones(nn),
                       units='Pa',
                       desc='Ambient Pressure')

        self.add_param('T_ambient',
                       val=298.0*np.ones(nn),
                       units='K',
                       desc='Ambient Temperture')

        self.add_param('R',
                       val=287.0*np.ones(nn),
                       units='J/(kg*K)',
                       desc='Ideal Gas Constant')

        self.add_param('D_magnetic',
                       val=150.0*np.ones(nn),
                       units='N',
                       desc='Drag from magnetic levitation')

        self.add_param('v',
                       val=335.0*np.ones(nn),
                       units='m/s',
                       desc='Velocity')

        self.add_output('F_drag',
                        val=0.0*np.ones(nn),
                        units='N',
                        desc='Drag Force')

        self.add_output('F_thrust',
                        val=0.0*np.ones(nn),
                        units='N',
                        desc='Thrust Force')

    def solve_nonlinear(self, params, unknowns, resids):
        #  dCalculate air density and drag force
        rho = params['p_tube']/(params['R']*params['T_ambient'])
        unknowns['F_drag'][:] = (.5*rho*(params['v']**2)*params['S']) + params['D_magnetic']
        unknowns['F_thrust'][:] = 30000.0
        # TODO: thrust value as determined by cycle analysis

if __name__ == '__main__':

    top = Problem()
    root = top.root = Group()

    params = (
        ('Cd', .2),
        ('V', 335.0, {'units': 'm/s'}),
        ('D_magnetic', 150.0, {'units': 'N'})
    )

    root.add('input_vars', IndepVarComp(params),
             promotes=['Cd', 'v', 'D_magnetic'])
    root.add('p', PodThrustAndDrag(),
             promotes=['F_drag', 'Cd', 'v', 'D_magnetic'])

    top.setup()
    top.run()

    print('\n')
    print('Drag Force = %f N' % top['D'])
