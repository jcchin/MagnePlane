from pointer.components import RHS

from .eom import MagneplaneEOM
from pod_thrust_and_drag import PodThrustAndDrag


class MagnePlaneRHS(RHS):

    def __init__(self, grid_data, dynamic_controls=None, static_controls=None):
        super(MagnePlaneRHS, self).__init__(grid_data, dynamic_controls,
                                            static_controls)

        self.add(name='eom',
                 system=MagneplaneEOM(grid_data),
                 promotes=['*'])

        self.add(name='pod_thrust_drag',
                 system=PodThrustAndDrag(grid_data),
                 promotes=['*'])

        self.complete_init()
