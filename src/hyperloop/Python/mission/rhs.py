from pointer.components import RHS

from .eom import MagneplaneEOM
from pod_thrust_and_drag import PodThrustAndDrag
from lat_long import LatLong
from terrain import TerrainElevationComp


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

        self.add(name='latlon',
                 system=LatLong(grid_data),
                 promotes=['*'])

        self.add(name='terrain',
                 system=TerrainElevationComp(grid_data),
                 promotes=['*'])

        self.complete_init()