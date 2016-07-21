from openmdao.api import Component, Group, Problem, IndepVarComp, ScipyGMRES

from hyperloop.Python.tube.tube_vacuum import Vacuum
from hyperloop.Python.tube.tube_wall_temp import TubeTemp, TempBalance
from hyperloop.Python.tube.tube_and_pylon import TubeAndPylon
from hyperloop.Python.tube.propulsion_mechanics import PropulsionMechanics
from hyperloop.Python.tube.tube_power import TubePower

class TubeGroup(Group):
    """
    Group containing tube and pod groups

    Params
    ------
    tube_length : float
        Total length of tube from Mission (m)
    tube_area : float
        Cross sectional inner area of tube from Pod Mach. Default is 41. m**2
    pressure_initial : float
        initial Pressure before the pump down . Default value is 760.2.
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
    tube_thickness : float
        Thickness of tube in m. Default value is .05
    num_pods : int
        Number of Pods in the Tube at a given time
    nozzle_air_W : float
        mass flow rate of the air exiting the pod nozzle (kg/s)
    nozzle_air_T : float
        temp of the air exiting the pod nozzle (K)
    tunnel_pressure : float
        Pressure of air in tube.  Default value is 850 Pa.  Value will come from vacuum component
    pod_mass : float
        Total weight of pod from pod_mass (kg)
    h : float
        Height of each pylon. Default value is 10 m.
    R : float
        Ideal gas constant. Default valut is 287 J/(m*K).
    T_ambient : float
        Tunnel ambient temperature. Default value is 298 K.
    g : float
        Gravitational acceleration. Default value is 9.81 m/s**2
    vf : float
        Top pod speed after boosting section. Default value is 335 m/s. Value will be taken from aero module
    vo : float
        Speed of pod when it enters boosting section. Default value is 324 m/s.
    eta : float
        Efficiency of propulsion system. Default value is .8. value will come from propulsion module.
    Cd : float
        Drag coefficient of pod.  Default value is .2. More accurate results will come from CFD
    S : float
        Reference area of the pod. Default value is 1.4 m**2. Value will be pulled from geometry module
    D_mag : float
        Drag force from magnetic levitation in N. Default value is 150 N.  Value will come from levitation analysis
    nozzle_thrust : float
        Thrust produced by pod compressed air. Default value 21473.92 N. Will pull value from flow_path.py
    ram_drag : float
        Drag produced by inlet ram pressure. Default value is 7237.6
    num_thrust : float
        Number of propulsion thrusts required for trip (unitless)
    time_thrust : float
        Time required to accelerate pod to 1G (s)

    Returns
    -------
    temp_boundary : float
        Ambient temperature inside tube (K)
    """

    def __init__(self):
        super(TubeGroup, self).__init__()

        # Adding in components to Tube Group
        self.add('Vacuum', Vacuum(), promotes=['tube_area',
        									                     'tube_length',
                          									   'electricity_price',
                          									   'pressure_initial',
                                               'pwr',
                                               'speed',
                                               'time_down',
                                               'gamma',
                                               'pump_weight'])

        self.add('Temp',TubeTemp(), promotes=['nozzle_air_W',
                                              'nozzle_air_Tt',
                                              'num_pods',
                                              'temp_boundary',
                                              'tube_thickness'])

        self.add('Struct', TubeAndPylon(), promotes=['h', 'p_tunnel', 'm_pod'])
        
        self.add('PropMech', PropulsionMechanics(), promotes=['vf',
                                                              'v0',
                                                              'Cd',
                                                              'S',
                                                              'D_mag',
                                                              'nozzle_thrust',
                                                              'ram_drag'])
        
        self.add('TubePower', TubePower(), promotes=['num_thrust',
                                                     'time_thrust'])

        # Connects tube group level variables to downstream components
        self.connect('tube_area', ['Temp.tube_area', 'Struct.tube_area'])
        self.connect('tube_length', 'Temp.length_tube')
        self.connect('p_tunnel', ['PropMech.p_tube', 'Vacuum.pressure_final'])
        self.connect('electricity_price', 'TubePower.elec_price')
        self.connect('tube_thickness', 'Struct.t')
        self.connect('m_pod', 'PropMech.m_pod')

        # Connects vacuum outputs to downstream components
        self.connect('Vacuum.weight_tot', 'Struct.vac_weight')
        self.connect('Vacuum.pwr_tot', 'TubePower.vac_power')
        self.connect('Vacuum.energy_tot', 'TubePower.vac_energy_day')

        # Connects tube_wall_temp outputs to downstream components
        self.connect('temp_boundary', 'PropMech.T_ambient')
        self.connect('temp_boundary', 'TubePower.tube_temp')

        # Connects propulsion_mechanics outputs to downstream components
        self.connect('PropMech.pwr_req', 'TubePower.prop_power')

        self.ln_solver = ScipyGMRES()

if __name__ == "__main__":

    top = Problem()
    top.root = Group()
    top.root.add('TubeGroup', TubeGroup())

    des_vars = (('pressure_initial', 760.2, {'units' : 'torr'}),
              ('pwr', 18.5, {'units' : 'kW'}),
              ('speed', 163333.3, {'units' : 'L/min'}),
              ('time_down', 300.0, {'units' : 'min'}),
              ('gamma', .8, {'units' : 'unitless'}),
              ('pump_weight', 715.0, {'units' : 'kg'}),
              ('nozzle_air_W',1.08, {'units': 'kg/s'}),
              ('nozzle_air_Tt',1710.0, {'units': 'K'}),
              ('num_pods',34, {'units': 'unitless'}),
              ('h', 10.0, {'units': 'm'}),
              ('vf',335.0, {'units': 'm/s'}),
              ('v0',324.0, {'units': 'm/s'}),
              ('Cd', 0.2, {'units': 'm'}),
              ('S', 1.4, {'units': 'm**2'}),
              ('D_mag', 150.0, {'units': 'N'}),
              ('nozzle_thrust', 3500.0, {'units': 'N'}),
              ('ram_drag',7237.6, {'units': 'N'}),
              ('num_thrust',5.0, {'units': 'unitless'}),
              ('time_thrust',1.5, {'units': 's'}),
              ('tube_area', 41., {'units': 'm**2'}),
              ('tube_length', 480000., {'units': 'm'}),
              ('tunnel_pressure', 850., {'units': 'Pa'}),
              ('electricity_price', .13, {'units': 'USD/kW/h'}),
              ('tube_thickness', .05, {'units': 'm'}),
              ('pod_mass', 3100., {'units': 'kg'}))

    top.root.add('des_vars',IndepVarComp(des_vars))
    top.root.connect('des_vars.nozzle_air_W', 'TubeGroup.nozzle_air_W')
    top.root.connect('des_vars.nozzle_air_Tt', 'TubeGroup.nozzle_air_Tt')
    top.root.connect('des_vars.num_pods', 'TubeGroup.num_pods')
    top.root.connect('des_vars.h','TubeGroup.h')
    top.root.connect('des_vars.vf', 'TubeGroup.vf')
    top.root.connect('des_vars.v0', 'TubeGroup.v0')
    top.root.connect('des_vars.Cd','TubeGroup.Cd')
    top.root.connect('des_vars.S','TubeGroup.S')
    top.root.connect('des_vars.D_mag','TubeGroup.D_mag')
    top.root.connect('des_vars.nozzle_thrust','TubeGroup.nozzle_thrust')
    top.root.connect('des_vars.ram_drag','TubeGroup.ram_drag')
    top.root.connect('des_vars.num_thrust', 'TubeGroup.num_thrust')
    top.root.connect('des_vars.time_thrust', 'TubeGroup.time_thrust')
    top.root.connect('des_vars.tube_area', 'TubeGroup.tube_area')
    top.root.connect('des_vars.tube_length', 'TubeGroup.tube_length')
    top.root.connect('des_vars.tunnel_pressure', 'TubeGroup.p_tunnel')
    top.root.connect('des_vars.electricity_price', 'TubeGroup.electricity_price')
    top.root.connect('des_vars.tube_thickness', 'TubeGroup.tube_thickness')
    top.root.connect('des_vars.pod_mass', 'TubeGroup.m_pod')

    # from openmdao.api import view_tree
    # view_tree(top)
    # exit()
    top.setup()
    top.root.list_connections()
    top.run()

    # print('\n')
    # print('Vacuum.weight_tot:%f' % top['TubeGroup.Vacuum.weight_tot'])
    # print('Struct.vac_weight: %f' % top['TubeGroup.Struct.vac_weight'])
    #
    # print('temp_boundary: %f' %top['TubeGroup.temp_boundary'])
    # print('PropMech.T_ambient: %f' % top['TubeGroup.PropMech.T_ambient'])
    # print('TubePower.tube_temp: %f' % top['TubeGroup.TubePower.tube_temp'])
    #
    # print('Vacuum.pwr_tot %f' % top['TubeGroup.Vacuum.pwr_tot'])
    # print('TubePower.vac_power: %f' % top['TubeGroup.TubePower.vac_power'])
    # print('PropMech.pwr_req: %f' % top['TubeGroup.PropMech.pwr_req'])
    # print('TubePower.prop_power: %f' % top['TubeGroup.TubePower.prop_power'])

    print('\n')
    print('Total Tube Power [kW]: %f' % top['TubeGroup.TubePower.tot_power'])
    print('Tube Temp [K]: %f' % top['TubeGroup.temp_boundary'])
