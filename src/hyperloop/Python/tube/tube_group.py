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
        Cross sectional area of tube from Pod Mach (m)
    pressure_initial : float
        initial Pressure before the pump down . Default value is 760.2.
    pressure_final : float
        Desired pressure within tube. Default value is 7.0.
    speed : float
        Pumping speed. Default value is 163333.3.
    pwr : float
        Motor rating. Default value is 18.5.
    electricity_price : float
        Cost of electricity per kilowatt hour. Default value is 0.13.
    time_down : float
        Desired pump down time. Default value is 300.0.
    gamma : float
        Operational percentage of the pump per day. Default value is 0.8.
    pump_weight : float
        Weight of one pump. Default value is 715.0.
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
    mag_drag : float
        Magnetic drag on pod from Breakpoint Levitation (N)
    nozzle.Fg : float
        Nozzle thrust from Cycle (N)
    inlet.F_ram : float
        Inlet ram drag from Cycle (N)

    Returns
    -------
    temp_boundary : float
        Ambient temperature inside tube (K)
    tot_power : float
        Total power requirement for tube components (W)
    """

    def __init__(self):
        super(TubeGroup, self).__init__()

        #Adding in components to Tube Group
        self.add('Vacuum', Vacuum(), promotes=['tube_length',
                                               'tube_area',
                                               'pressure_initial',
                                               'pressure_final',
                                               'pwr',
                                               'speed',
                                               'electricity_price',
                                               'time_down',
                                               'gamma',
                                               'pump_weight'])
        self.add('TempBalance', TempBalance(), promotes=['temp_boundary'])
        self.add('TubeWallTemp',TubeWallTemp(), promotes=['length_tube',
                                                           'diameter_outer_tube',
                                                           'nozzle_air_W',
                                                           'nozzle_air_Tt',
                                                           'nozzle_air_Cp'])
        self.add('Struct', TubeAndPylon(), promotes=['p_tunnel',
                                                     'm_pod',
                                                     'tube_area',
                                                     'h'])
        self.add('PropMech', PropulsionMechanics(), promotes=['p_tube',
                                                              'vf',
                                                              'v0',
                                                              'm_pod',
                                                              'S',
                                                              'mag_drag',
                                                              'nozzle.Fg',
                                                              'inlet.F_ram']
        self.add('TubePower', TubePower(), promotes=['num_thrust', 'elec_price', 'time_thrust'])

        #Connects vacuum outputs to downstream components
        self.connect('Vacuum.weight_tot', 'Struct.vac_weight')
        self.connect('Vacuum.pwr_tot', 'TubePower.vac_power')
        self.connect('Vacuum.energy_tot', 'TubePower.vac_energy')

        #Connects tube_wall_temp outputs to downstream components
        self.connect('temp_boundary', 'T_ambient')
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
              ('tube_area', 40.0),
              ('h', 10.0),
              ('Cd', 0.2),
              ('S', 1.4),
              ('mag_drag', 150.0),
              ('pod_thrust', 3500.0),
              ('A', .0225),
              ('t', .05))

    top.root.add('des_vars',IndepVarComp(params))
    top.root.connect('des_vars.radius_outer_tube','TubeGroup.radius_outer_tube')
    top.root.connect('des_vars.length_tube','TubeGroup.length_tube')
    top.root.connect('des_vars.m_pod','TubeGroup.m_pod')
    top.root.connect('des_vars.m_pod', 'TubeGroup.m_pod')
    top.root.connect('des_vars.tube_area','TubeGroup.tube_area')
    top.root.connect('des_vars.h','TubeGroup.h')
    top.root.connect('des_vars.Cd','TubeGroup.Cd')
    top.root.connect('des_vars.S','TubeGroup.S')
    top.root.connect('des_vars.mag_drag','TubeGroup.mag_drag')
    top.root.connect('des_vars.pod_thrust','TubeGroup.pod_thrust')
    top.root.connect('des_vars.A','TubeGroup.A')
    top.root.connect('des_vars.t','TubeGroup.t')

    top.setup()
    top.root.list_connections()
    top.run()

    # print('\n')
    # print('Vacuum.weight_tot:%f' % top['TubeGroup.Vacuum.weight_tot'])
    # print('Struct.vac_weight: %f' % top['TubeGroup.Struct.vac_weight'])
    #
    # print('temp_boundary: %f' %top['TubeGroup.temp_boundary'])
    # print('PropMech.T_ambient: %f' % top['TubeGroup.T_ambient'])
    # print('TubePower.tube_temp: %f' % top['TubeGroup.TubePower.tube_temp'])
    #
    # print('Vacuum.tot_pwr %f' % top['TubeGroup.Vacuum.tot_pwr'])
    # print('TubePower.vac_power: %f' % top['TubeGroup.TubePower.vac_power'])
    # print('PropMech.pwr_req: %f' % top['TubeGroup.PropMech.pwr_req'])
    # print('TubePower.prop_power: %f' % top['TubeGroup.TubePower.prop_power'])

    print('\n')
    print('Total Power: %f' % top['TubeGroup.TubePower.tot_power'])
    print('Tube Temp: %f' % top['TubeGroup.temp_boundary'])
