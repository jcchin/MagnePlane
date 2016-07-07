"""
Group for Tube components containing the following components:
Vacuum, Tube Temperature, Tube and Pylon (structural), Propulsion Mechanics, Tube Power
"""

from openmdao.api import Group, Problem, IndepVarComp, ScipyGMRES

from hyperloop.Python.tube.tube_vacuum import Vacuum
from hyperloop.Python.tube.tube_wall_temp import TempBalance, TubeWallTemp
from hyperloop.Python.tube.tube_and_pylon import TubeAndPylon
from hyperloop.Python.pod.propulsion_mechanics import PropulsionMechanics
from hyperloop.Python.tube.tube_power import TubePower

class TubeGroup(Group):
    def __init__(self):
        super(TubeGroup, self).__init__()

        #params = (('',,{'units': ''}))
        #self.add('Indep',IndepVarComp(params))

        #Adding in components to Tube Group
        self.add('Vacuum', Vacuum(), promotes=['rad']) #need to add A_tube and calculate radius and length in vacuum
        self.add('TempBalance', TempBalance(), promotes=['temp_boundary'])
        self.add('TubeWallTemp',TubeWallTemp(), promotes=['radius_outer_tube',
                                                         'length_tube'])
        self.add('Struct', TubeAndPylon(), promotes=['m_pod',
                                                    'r',
                                                    'h']) #need to add A_tube and calculate radius in tubeAndPylon
        self.add('PropMech', PropulsionMechanics(), promotes=['A',
                                                             't',
                                                             'm_pod',
                                                             'Cd',
                                                             'S',
                                                             'D_magnetic',
                                                             'Thrust_pod'])
        self.add('TubePower', TubePower())

        #Connects vacuum outputs to downstream components
        self.connect('Vacuum.weighttot', 'Struct.vac_weight')  # need to add Vac_weight to tube_and_pylon
        self.connect('Vacuum.totpwr', 'TubePower.vac_power')

        #Connects tube_wall_temp outputs to downstream components
        self.connect('temp_boundary', 'PropMech.T_ambient')
        #self.connect('TempBalance.temp_boundary', 'TubePower.tube_temp')

        #Connects propulsion_mechanics outputs to downstream components
        self.connect('PropMech.pwr_req', 'TubePower.prop_power')

        self.ln_solver = ScipyGMRES()

if __name__ == "__main__":

    top = Problem()
    top.root = TubeGroup()

    top.setup()
    top.root.list_connections()
    top.run()


    print('\n')
    print('Vacuum.weighttot:%f' % top['Vacuum.weighttot'])
    print('Struct.vac_weight: %f' % top['Struct.vac_weight'])
    print('Vacuum.totpwr %f' % top['Vacuum.totpwr'])
    print('TubePower.vac_power: %f' %top['TubePower.vac_power'])
    print('PropMech.pwr_req: %f' %top['PropMech.pwr_req'])
    print('TubePower.prop_power: %f' %top['TubePower.prop_power'])

    print('Total Power: %f' % top['TubePower.tot_power'])

