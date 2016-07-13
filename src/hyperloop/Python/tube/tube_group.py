from openmdao.api import Group, Problem, IndepVarComp, ScipyGMRES

from hyperloop.Python.tube.tube_vacuum import Vacuum
from hyperloop.Python.tube.tube_wall_temp import TempBalance, TubeWallTemp
from hyperloop.Python.tube.tube_and_pylon import TubeAndPylon
from hyperloop.Python.tube.propulsion_mechanics import PropulsionMechanics
from hyperloop.Python.tube.tube_power import TubePower

class TubeGroup(Group):
    """
    Group containing tube components

    Components
    ----------
    Vacuum : Vacuum
    Tube Temperature/Thermal Conditions : TempBalance, TubeWallTemp
    Structural : TubeAndPylon
    Propulsion Mechanics : PropulsionMechanics
    Tube Power Requirements : TubePower

    Params
    ------
    tube_length : float
        Total length of tube from Mission (m)
    tube_area : float
        Cross sectional area of tube from Aero (m)
    nozzle.Fl_O:stat:W : float
        Pod exit flow rate from Cycle (kg/s)
    nozzle.Fl_O:tot:T : float
        Pod exit temperature from Cycle (K)
    m_pod : float
        Total weight of pod from pod_mass (kg)
    pylon_height : float
        Height of pylons from Mission (m)
    A : float
        Area of magnets from Breakpoint Levitation (m^2)
    d : float
        Thickness of magnets (m)
    pod_frontal_area : float
        Front cross sectional area of pod from Geometry (m^2)
    magdrag : float
        Magnetic drag on pod from Breakpoint Levitation (N)
    nozzle.Fg : float
        Nozzle thrust from Cycle (N)
    inlet.F_ram : float
        Inlet ram drag from Cycle (N)


    Outputs
    -------
    temp_boundary : float
        Ambient temperature inside tube (K)
    tot_power : float
        Total power requirement for tube components (W)
    """

    def __init__(self):
        super(TubeGroup, self).__init__()

        #Adding in components to Tube Group
        self.add('Vacuum', Vacuum(), promotes=['tube_radius',
                                               'tube_length',
                                               'pinit',
                                               'pfinal',
                                               'pwr',
                                               'speed',
                                               'eprice',
                                               'tdown',
                                               'gamma',
                                               'pumpweight']) #need to add A_tube and calculate radius and length in vacuum
        self.add('TempBalance', TempBalance(), promotes=['temp_boundary'])
        self.add('TubeWallTemp',TubeWallTemp(), promotes=['radius_outer_tube',
                                                          'length_tube',
                                                          'nozzle_air_W',
                                                          'nozzle_air_Tt',
                                                          'nozzle_air_Cp'])
        self.add('Struct', TubeAndPylon(), promotes=['p_tunnel',
                                                     'm_pod',
                                                     'r',
                                                     'h']) #need to add A_tube and calculate radius in tubeAndPylon
        self.add('PropMech', PropulsionMechanics(), promotes=['p_tube',
                                                              'vf',
                                                              'v0',
                                                              'm_pod',
                                                              'Cd',
                                                              'S',
                                                              'D_magnetic',
                                                              'Thrust_pod', #need to calculate nozzle.Fg - inlet.F_ram to get thrust
                                                              'A',
                                                              't'])
        self.add('TubePower', TubePower())

        #Connects vacuum outputs to downstream components
        self.connect('Vacuum.weighttot', 'Struct.vac_weight')
        self.connect('Vacuum.totpwr', 'TubePower.vac_power')

        #Connects tube_wall_temp outputs to downstream components
        self.connect('temp_boundary', 'PropMech.T_ambient')
        self.connect('temp_boundary', 'TubePower.tube_temp')

        #Connects propulsion_mechanics outputs to downstream components
        self.connect('PropMech.pwr_req', 'TubePower.prop_power')

        self.ln_solver = ScipyGMRES()

if __name__ == "__main__":

    top = Problem()
    top.root = Group()
    top.root.add('TubeGroup', TubeGroup())


    params = (
              ('radius_outer_tube', 1.115),
              ('length_tube', 482803.0),
              ('m_pod', 3100.0),
              ('r', 1.1),
              ('h', 10.0),
              ('Cd', 0.2),
              ('S', 1.4),
              ('D_magnetic', 150.0),
              ('Thrust_pod', 3500.0),
              ('A', .0225),
              ('t', .05),
              )

    top.root.add('des_vars',IndepVarComp(params))
    top.root.connect('des_vars.radius_outer_tube','TubeGroup.radius_outer_tube')
    top.root.connect('des_vars.length_tube','TubeGroup.length_tube')
    top.root.connect('des_vars.m_pod','TubeGroup.m_pod')
    top.root.connect('des_vars.m_pod', 'TubeGroup.m_pod')
    top.root.connect('des_vars.r','TubeGroup.r')
    top.root.connect('des_vars.h','TubeGroup.h')
    top.root.connect('des_vars.Cd','TubeGroup.Cd')
    top.root.connect('des_vars.S','TubeGroup.S')
    top.root.connect('des_vars.D_magnetic','TubeGroup.D_magnetic')
    top.root.connect('des_vars.Thrust_pod','TubeGroup.Thrust_pod')
    top.root.connect('des_vars.A','TubeGroup.A')
    top.root.connect('des_vars.t','TubeGroup.t')

    top.setup()
    top.root.list_connections()
    top.run()

    print('\n')

    print('Vacuum.weighttot:%f' % top['TubeGroup.Vacuum.weighttot'])
    print('Struct.vac_weight: %f' % top['TubeGroup.Struct.vac_weight'])

    print('temp_boundary: %f' %top['TubeGroup.temp_boundary'])
    print('PropMech.T_ambient: %f' % top['TubeGroup.PropMech.T_ambient'])
    print('TubePower.tube_temp: %f' % top['TubeGroup.TubePower.tube_temp'])

    print('Vacuum.totpwr %f' % top['TubeGroup.Vacuum.totpwr'])
    print('TubePower.vac_power: %f' % top['TubeGroup.TubePower.vac_power'])
    print('PropMech.pwr_req: %f' % top['TubeGroup.PropMech.pwr_req'])
    print('TubePower.prop_power: %f' % top['TubeGroup.TubePower.prop_power'])

    print('Total Power: %f' % top['TubeGroup.TubePower.tot_power'])
    print('mpod: %f' %top['TubeGroup.m_pod'])

