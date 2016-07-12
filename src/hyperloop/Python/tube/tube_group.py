"""
Group for Tube components containing the following components:
Vacuum, Tube Temperature, Tube and Pylon (structural), Propulsion Mechanics, Tube Power
"""

from openmdao.api import Group, Problem, IndepVarComp, ScipyGMRES

from hyperloop.Python.tube.propulsion_mechanics import PropulsionMechanics
from hyperloop.Python.tube.tube_and_pylon import TubeAndPylon
from hyperloop.Python.tube.tube_power import TubePower
from hyperloop.Python.tube.tube_vacuum import Vacuum
from hyperloop.Python.tube.tube_wall_temp import TempBalance, TubeWallTemp

class TubeGroup(Group):
    def __init__(self):
        super(TubeGroup, self).__init__()

        self.add('Vacuum', Vacuum(), promotes=['tube_radius', 'tube_length'])
        self.add('TempBalance', TempBalance(), promotes=['tube_length'])
        self.add('TubeWallTemp', TubeWallTemp(),promotes=['temp_boundary'])
        self.add('Struct', TubeAndPylon())
        self.add('PropMech', PropulsionMechanics())
        self.add('TubePower', TubePower())

        self.connect('Vacuum.weighttot', 'Struct.vac_weight')  # need to add Vac_weight to tube_and_pylon
        self.connect('Vacuum.totpwr', 'TubePower.vac_power')

        self.connect('TubeWallTemp.ss_temp_residual', 'Struct.dT_tube')
        self.connect('TempBalance.temp_boundary', 'PropMech.T_ambient')
        #self.connect('TempBalance.temp_boundary', 'TubePower.tube_temp')

        self.connect('PropMech.pwr_req', 'TubePower.prop_power')

        self.ln_solver = ScipyGMRES()

if __name__ == "__main__":

    top = Problem()
    root = top.root = TubeGroup()

    """
    root.add('tube',TubeGroup())
    params = (
        ()
    )
    top.root.add('PodVar',IndepVarComp(params))

    top.root.connect('Cycle',)
    top.root.connect('PodMach',)
    top.root.connect('DriveTrain',)
    top.root.connect('Geom',)
    top.root.connect('MagLev',)
    top.root.connect('Weight',)
    """

    #root.ln_solver = ScipyGMRES()
    top.setup()
    top.root.list_connections()
    top.run()


    print('Vacuum.weighttot:%f' % top['Vacuum.weighttot'])
    print('Struct.vac_weight: %f' % top['Struct.vac_weight'])
    print('Vacuum.totpwr %f' % top['Vacuum.totpwr'])
    print('TubePower.vac_power: %f' %top['TubePower.vac_power'])
    print('Thermal.ss_temp_residual: %f' %top['TubeWallTemp.ss_temp_residual'])
    print('Struct.dT_tube: %f' %top['Struct.dT_tube'])
    print('PropMech.pwr_req: %f' %top['PropMech.pwr_req'])
    print('TubePower.prop_power: %f' %top['TubePower.prop_power'])

    print('Total Power: %f' % top['TubePower.tot_power'])

