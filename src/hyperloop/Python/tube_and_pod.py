"""
Group for Tube and Pod components containing the following two sub-groups:
Pod and Tube
"""
from openmdao.api import Component, Group, Problem, IndepVarComp, NLGaussSeidel, ScipyGMRES
from hyperloop.Python.tube.tube_group import TubeGroup
from hyperloop.Python.pod.pod_group import PodGroup

class TubeAndPod(Group):
    def __init__(self):
        """TODOs

        Params
        ------
        tube_pressure : float
            Tube total pressure (Pa)
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
        tube_length : float
            Total length of tube from Mission (m)
        vf : float
            Top pod speed after boosting section. Default value is 335 m/s. Value will be taken from aero module
        vo : float
            Speed of pod when it enters boosting section. Default value is 324 m/s.
        num_thrust : float
            Number of propulsion thrusts required for trip (unitless)
        time_thrust : float
            Time required to accelerate pod to 1G (s)
        pod_mach : float
            Vehicle mach number (unitless)
        comp.map.PRdes : float
            Pressure ratio of compressor (unitless)
        nozzle.Ps_exhaust : float
            Exit pressure of nozzle (psi)
        comp_inlet_area : float
            Inlet area of compressor. (m**2)
        des_time : float
            time until design power point (h)
        time_of_flight : float
            total mission time (h)
        motor_max_current : float
            max motor phase current (A)
        motor_LD_ratio : float
            length to diameter ratio of motor (unitless)
        motor_oversize_factor : float
            scales peak motor power by this figure
        inverter_efficiency : float
            power out / power in (W)
        battery_cross_section_area : float
            cross_sectional area of battery used to compute length (cm^2)
        n_passengers : float
            Number of passengers per pod. Default value is 28
        A_payload : float
            Cross sectional area of passenger compartment. Default value is 2.72
        
        Returns
        -------
        S : float
            Platform area of the pod
        total_pod_mass : float
            Pod Mass (kg)

        References
        ----------
        .. [1] Friend, Paul. Magnetic Levitation Train Technology 1. Thesis.
           Bradley University, 2004. N.p.: n.p., n.d. Print.
        """
        super(TubeAndPod, self).__init__()

        self.add('tube', TubeGroup(), promotes=['pressure_initial', 'pwr', 'num_pods', 'Cd',
                                              'speed', 'time_down', 'gamma', 'pump_weight',
                                              'electricity_price', 'tube_thickness', 'r_pylon',
                                              'tube_length', 'h', 'vf', 'v0', 'num_thrust', 'time_thrust',])
        self.add('pod', PodGroup(), promotes=['pod_mach', 'tube_pressure', 'comp.map.PRdes',
                                              'nozzle.Ps_exhaust', 'comp_inlet_area', 'des_time',
                                              'time_of_flight', 'motor_max_current', 'motor_LD_ratio',
                                              'motor_oversize_factor', 'inverter_efficiency', 'battery_cross_section_area',
                                              'n_passengers', 'A_payload', 'S', 'total_pod_mass', 'vel_b',
                                              'h_lev', 'vel', 'mag_drag'])

        # Connects promoted group level params
        self.connect('tube_pressure', 'tube.p_tunnel')

        # Connects tube group outputs to pod
        self.connect('tube.temp_boundary', 'pod.tube_temp')

        # Connects pod group outputs to tube
        self.connect('pod.nozzle.Fg', 'tube.nozzle_thrust')
        self.connect('pod.inlet.F_ram', 'tube.ram_drag')
        self.connect('pod.nozzle.Fl_O:tot:T', 'tube.nozzle_air_Tt')
        self.connect('pod.nozzle.Fl_O:stat:W', 'tube.nozzle_air_W')
        self.connect('pod.A_tube', 'tube.tube_area')
        self.connect('S', 'tube.S')
        self.connect('mag_drag', 'tube.D_mag')
        self.connect('total_pod_mass', 'tube.m_pod')

        self.nl_solver = NLGaussSeidel()
        self.nl_solver.options['maxiter'] = 20
        self.nl_solver.options['atol'] = 0.0001
        self.nl_solver.options['iprint'] = 2

        self.ln_solver = ScipyGMRES()
        self.ln_solver.options['maxiter'] = 100

if __name__ == '__main__':

    prob = Problem()
    root = prob.root = Group()
    root.add('TubeAndPod', TubeAndPod())

    params = (('tube_pressure', 850.0, {'units' : 'Pa'}),
              ('pressure_initial', 760.2, {'units' : 'torr'}),
              ('num_pods', 18.),
              ('pwr', 18.5, {'units' : 'kW'}),
              ('speed', 163333.3, {'units' : 'L/min'}),
              ('time_down', 1440.0, {'units' : 'min'}),
              ('gamma', .8, {'units' : 'unitless'}),
              ('pump_weight', 715.0, {'units' : 'kg'}),
              ('electricity_price', 0.13, {'units' : 'USD/(kW*h)'}),
              ('tube_thickness', .05, {'units' : 'm'}),
              ('tube_length', 480000., {'units' : 'm'}),
              ('vf', 286.85, {'units' : 'm/s'}),
              ('v0', 286.85-15.0, {'units' : 'm/s'}),
              ('Cd', 0.2, {'units': 'm'}),
              ('num_thrust', 600.0/24.0, {'units' : 'unitless'}),
              ('time_thrust', 1.5, {'units' : 's'}),
              ('pod_mach', .8, {'units': 'unitless'}),
              ('comp_inlet_area', 2.3884, {'units': 'm**2'}),
              ('comp_PR', 6.0, {'units': 'unitless'}),
              ('PsE', 0.05588, {'units': 'psi'}),
              ('des_time', 1.0),
              ('time_of_flight', 1.0),
              ('motor_max_current', 800.0),
              ('motor_LD_ratio', 0.83),
              ('motor_oversize_factor', 1.0),
              ('inverter_efficiency', 1.0),
              ('battery_cross_section_area', 15000.0, {'units': 'cm**2'}),
              ('n_passengers', 28.),
              ('A_payload', 2.72),
              ('r_pylon', .1, {'units' : 'm'}),
              ('h', 10.0, {'units' : 'm'}),
              ('vel_b', 23.0, {'units': 'm/s'}),
              ('h_lev', 0.01, {'unit': 'm'}),
              ('vel', 286.86, {'units': 'm/s'}))

    prob.root.add('des_vars', IndepVarComp(params))
    prob.root.connect('des_vars.tube_pressure', 'TubeAndPod.tube_pressure')
    prob.root.connect('des_vars.pressure_initial', 'TubeAndPod.pressure_initial')
    prob.root.connect('des_vars.num_pods', 'TubeAndPod.num_pods')
    prob.root.connect('des_vars.pwr','TubeAndPod.pwr')
    prob.root.connect('des_vars.speed', 'TubeAndPod.speed')
    prob.root.connect('des_vars.time_down', 'TubeAndPod.time_down')
    prob.root.connect('des_vars.gamma','TubeAndPod.gamma')
    prob.root.connect('des_vars.pump_weight','TubeAndPod.pump_weight')
    prob.root.connect('des_vars.electricity_price','TubeAndPod.electricity_price')
    prob.root.connect('des_vars.tube_thickness', 'TubeAndPod.tube_thickness')
    prob.root.connect('des_vars.tube_length', 'TubeAndPod.tube_length')
    prob.root.connect('des_vars.h', 'TubeAndPod.h')
    prob.root.connect('des_vars.r_pylon', 'TubeAndPod.r_pylon')
    prob.root.connect('des_vars.vf', 'TubeAndPod.vf')
    prob.root.connect('des_vars.v0', 'TubeAndPod.v0')
    prob.root.connect('des_vars.Cd', 'TubeAndPod.Cd')
    prob.root.connect('des_vars.num_thrust', 'TubeAndPod.num_thrust')
    prob.root.connect('des_vars.time_thrust', 'TubeAndPod.time_thrust')
    prob.root.connect('des_vars.pod_mach', 'TubeAndPod.pod_mach')
    prob.root.connect('des_vars.comp_inlet_area', 'TubeAndPod.comp_inlet_area')
    prob.root.connect('des_vars.comp_PR', 'TubeAndPod.comp.map.PRdes')
    prob.root.connect('des_vars.PsE', 'TubeAndPod.nozzle.Ps_exhaust')
    prob.root.connect('des_vars.des_time', 'TubeAndPod.des_time')
    prob.root.connect('des_vars.time_of_flight', 'TubeAndPod.time_of_flight')
    prob.root.connect('des_vars.motor_max_current', 'TubeAndPod.motor_max_current')
    prob.root.connect('des_vars.motor_LD_ratio', 'TubeAndPod.motor_LD_ratio')
    prob.root.connect('des_vars.motor_oversize_factor', 'TubeAndPod.motor_oversize_factor')
    prob.root.connect('des_vars.inverter_efficiency', 'TubeAndPod.inverter_efficiency')
    prob.root.connect('des_vars.battery_cross_section_area', 'TubeAndPod.battery_cross_section_area')
    prob.root.connect('des_vars.n_passengers', 'TubeAndPod.n_passengers')
    prob.root.connect('des_vars.A_payload', 'TubeAndPod.A_payload')
    prob.root.connect('des_vars.vel_b', 'TubeAndPod.vel_b')
    prob.root.connect('des_vars.h_lev', 'TubeAndPod.h_lev')
    prob.root.connect('des_vars.vel', 'TubeAndPod.vel')

    prob.setup()
    # from openmdao.api import view_tree
    # view_tree(prob)
    prob.run()

    # prob.root.list_states()

    print('\n')
    print('tube area            %f' % prob['TubeAndPod.pod.A_tube'])
    print('tube temp            %f' % prob['TubeAndPod.tube.temp_boundary'])
    print('\n')
    print('nozzle exit temp     %f' % prob['TubeAndPod.pod.nozzle.Fl_O:tot:T'])
    print('nozzle mass flow     %f' % prob['TubeAndPod.pod.nozzle.Fl_O:stat:W'])
    print('nozzle thrust        %f' % prob['TubeAndPod.pod.nozzle.Fg'])
    print('ram drag             %f' % prob['TubeAndPod.pod.inlet.F_ram'])
    print('\n')
    print('compressor mass      %f' % prob['TubeAndPod.pod.cycle.comp_mass'])
    print('compressor power     %f' % prob['TubeAndPod.pod.cycle.comp.power'])
    print('compressor trq       %f' % prob['TubeAndPod.pod.cycle.comp.trq'])
    print('\n')
    print('battery length       %f' % prob['TubeAndPod.pod.drivetrain.battery_length'])
    print('battery volume       %f' % prob['TubeAndPod.pod.drivetrain.battery_volume'])
    print('motor length         %f' % prob['TubeAndPod.pod.drivetrain.motor_length'])
    print('battery mass         %f' % prob['TubeAndPod.pod.drivetrain.battery_mass'])
    print('motor mass           %f' % prob['TubeAndPod.pod.drivetrain.motor_mass'])
    print('\n')
    print('pod length           %f' % prob['TubeAndPod.pod.pod_geometry.L_pod'])
    print('pod cross section    %f' % prob['TubeAndPod.pod.pod_geometry.A_pod'])
    print('pod diameter         %f' % prob['TubeAndPod.pod.pod_geometry.D_pod'])
    print('\n')
    print('pod mass w/o magnets %f' % prob['TubeAndPod.pod.pod_mass.pod_mass'])
    print('mag mass             %f' % prob['TubeAndPod.pod.levitation_group.Mass.m_mag'])
    print('\n')
    print('S:                   %f' % prob['TubeAndPod.S'])
    print('Mag Drag:            %f' % prob['TubeAndPod.mag_drag'])
    print('total pod mass       %f' % prob['TubeAndPod.total_pod_mass'])
