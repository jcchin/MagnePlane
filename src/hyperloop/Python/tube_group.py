"""
Group for Tube components containing the following components:
Vacuum, Tube Temperature, Tube and Pylon (structural), Propulsion Mechanics, Tube Power
"""

from openmdao.api import Component, Problem, Group

from hyperloop.Python.tube_vacuum import Vacuum
from hyperloop.Python.OldMagnePlaneCode.tube_wall_temp import TubeTemp
from hyperloop.Python.tube_and_pylon import TubeAndPylon
from hyperloop.Python.propulsion_mechanics import PropulsionMechanics
from hyperloop.Python.tube_power import TubePower

"""imports from Pod group
---------

Geometry: (to Therm), cross sectional area, planform area, length(to Propulsion)
Mag Group: Dmag (to Propulsion)
Weight: total weight(to Structure/to Propulsion)

"""

class TubeGroup(Group):
    def __init__(self):
        super(TubeGroup, self).__init__()

        self.add('User_in',IndepVarComp())

        self.add('Tube', TubeGroup())
        self.add('Vacuum', Vacuum())
        self.add('Thermal', TubeTemp())
        self.add('Struct',TubeAndPylon())
        self.add('PropMech',PropulsionMechanics())
        self.add('TubePower', TubePower())

        self.connect('Vacuum.weighttot', 'Struct.vac_weight')  # need to add Vac_weight to tube_and_pylon
        self.connect('Vacuum.totpwr', 'TubePow.vac_power')

        self.connect('Thermal.ss_temp_residual', 'Struct.dT_tube')
        self.connect('Thermal.temp_boundary', 'PropMech.T_ambient')
        self.connect('Thermal.temp_boundary', 'TubePow.tube_temp')

        self.connect('PropMech.pwr_req', 'TubePow.prop_power')

if __name__ == "__main__":

    prob = Problem()
    root = prob.root = Group()

    """
    root.add('Tube', TubeGroup())
    root.add('Cycle', CompressionCycle())
    root.add('Aero',PodMach())
    root.add('Geom',Geometry())
    root.add('LevGroup',LevGroup())
    root.add('Weight',Weight())

    root.add('PodGroup',PodGroup())
    """

    prob.setup()
    prob.root.list_connections()
    prob.run()

    print('Tube Temperature: %f' % prob['Tube.Thermal.tubetemp'])