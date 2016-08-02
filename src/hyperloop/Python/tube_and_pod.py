"""
Group for Tube and Pod components containing the following two sub-groups:
Pod and Tube
"""
from openmdao.api import Component, Group, Problem, IndepVarComp, NLGaussSeidel, ScipyGMRES
from hyperloop.Python.tube.tube_group import TubeGroup
from hyperloop.Python.pod.pod_group import PodGroup
from hyperloop.Python.ticket_cost import TicketCost
from hyperloop.Python.sample_mission import SampleMission

import numpy as np 
import matplotlib.pylab as plt 

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
                                              'tube_length', 'h', 'vf', 'v0', 'num_thrust', 'time_thrust', 
                                              'fl_start.W', 'depth', 'pod_period'])
        self.add('pod', PodGroup(), promotes=['pod_mach', 'tube_pressure', 'comp.map.PRdes',
                                              'nozzle.Ps_exhaust', 'comp_inlet_area', 'des_time',
                                              'time_of_flight', 'motor_max_current', 'motor_LD_ratio',
                                              'motor_oversize_factor', 'inverter_efficiency', 'battery_cross_section_area',
                                              'n_passengers', 'A_payload', 'S', 'total_pod_mass', 'vel_b',
                                              'h_lev', 'vel', 'mag_drag', 'L_pod'])
        self.add('cost', TicketCost(), promotes = ['land_length', 'water_length', 'track_length'])
        self.add('mission', SampleMission())

        # Connects promoted group level params
        self.connect('tube_pressure', ['tube.p_tunnel', 'cost.p_tunnel', 'mission.p_tunnel'])

        # Connects tube group outputs to pod
        self.connect('tube.temp_boundary', 'pod.tube_temp')

        # Connects pod group outputs to tube
        self.connect('pod.nozzle.Fg', ['tube.nozzle_thrust', 'mission.nozzle_thrust'])
        self.connect('pod.inlet.F_ram', ['tube.ram_drag', 'mission.ram_drag'])
        self.connect('pod.nozzle.Fl_O:tot:T', 'tube.nozzle_air_Tt')
        self.connect('pod.nozzle.Fl_O:stat:W', 'tube.nozzle_air_W')
        self.connect('pod.A_tube', 'tube.tube_area')
        self.connect('S', ['tube.S', 'cost.S', 'mission.S'])
        self.connect('L_pod', 'tube.L_pod')
        self.connect('mag_drag', ['tube.D_mag', 'cost.D_mag', 'mission.D_mag'])
        self.connect('total_pod_mass', ['tube.m_pod', 'cost.m_pod', 'mission.m_pod'])
        self.connect('vf', 'cost.vf')
        self.connect('pod_period', 'cost.pod_period')
        self.connect('tube.Struct.total_material_cost', 'cost.land_cost')
        self.connect('tube.Vacuum.pwr_tot', 'cost.vac_power')
        self.connect('tube.PropMech.pwr_req', 'cost.prop_power')
        self.connect('pod.cycle.comp.power', 'cost.pod_power')
        self.connect('tube.comp.power', 'cost.steady_vac_power')
        self.connect('tube.SubmergedTube.material_cost', 'cost.water_cost')
        self.connect('pod_mach', 'mission.M_pod')
        self.connect('track_length', 'mission.track_length')

        self.nl_solver = NLGaussSeidel()
        self.nl_solver.options['maxiter'] = 20
        self.nl_solver.options['atol'] = 0.0001
        # self.nl_solver.options['iprint'] = 2

        self.ln_solver = ScipyGMRES()
        self.ln_solver.options['maxiter'] = 20

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
              ('tube_thickness', .0415014, {'units' : 'm'}),
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
              ('A_payload', 2.3248, {'units' : 'm**2'}),
              ('r_pylon', 0.232, {'units' : 'm'}),
              ('h', 10.0, {'units' : 'm'}),
              ('vel_b', 23.0, {'units': 'm/s'}),
              ('h_lev', 0.01, {'unit': 'm'}),
              ('vel', 286.86, {'units': 'm/s'}),
              ('pod_period', 120.0, {'units' : 's'}),
              ('prop_period', 25.0e3, {'units' : 'm'}),
              ('ib', .04),
              ('bm', 20.0, {'units' : 'yr'}),
              ('track_length', 600.0, {'units' : 'km'}),
              ('avg_speed', 286.86, {'units' : 'm/s'}),
              ('depth', 10.0, {'units' : 'm'}),
              ('land_length', 600.0e3, {'units' : 'm'}),
              ('water_length', 0.0e3, {'units' : 'm'}))

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
    prob.root.connect('des_vars.pod_period', 'TubeAndPod.cost.pod_period')
    prob.root.connect('des_vars.prop_period', 'TubeAndPod.cost.prop_period')
    prob.root.connect('des_vars.ib', 'TubeAndPod.cost.ib')
    prob.root.connect('des_vars.bm', 'TubeAndPod.cost.bm')
    prob.root.connect('des_vars.track_length', 'TubeAndPod.track_length')
    prob.root.connect('des_vars.avg_speed', 'TubeAndPod.cost.avg_speed')
    prob.root.connect('des_vars.land_length', 'TubeAndPod.land_length')
    prob.root.connect('des_vars.water_length', 'TubeAndPod.water_length')

    prob.setup()

    # from openmdao.api import view_tree
    # view_tree(prob)
    # prob.run()

    # prob.root.list_states()
    # prob.run()

    # p_tunnel = np.concatenate((np.linspace(100.0, 1000.0, num =50, endpoint = False), np.linspace(1000.0,4000.0, num = 50, endpoint = True)))
    p_tunnel = 5.0*np.logspace(2,3,num = 50)
    A_tube = np.zeros((1, len(p_tunnel)))
    Re = np.zeros((1, len(p_tunnel)))
    T_tunnel = np.zeros((1, len(p_tunnel)))
    L_pod = np.zeros((1, len(p_tunnel)))
    L_bat = np.zeros((1, len(p_tunnel)))
    power = np.zeros((1, len(p_tunnel)))
    steady_vac = np.zeros((1,len(p_tunnel)))
    total_energy = np.zeros((1, len(p_tunnel)))

    # with open('/Users/kennethdecker/Desktop/Paper figures/pressure_trade.txt', 'w') as f:
    # f = open('/Users/kennethdecker/Desktop/Paper figures/pressure_trade.txt', 'w')
    # f.write('%10s \t %10s \t %10s \t %10s \t %10s \t %12s \r\n' % ('pressure', 'A_tube', 'Re', 'pod power', 'Vac Power', 'Total Energy'))

    for i in range(len(p_tunnel)):
        prob['des_vars.tube_pressure'] = p_tunnel[i]

        prob.run()

        A_tube[0,i] = prob['TubeAndPod.pod.A_tube']
        Re[0,i] = prob['TubeAndPod.pod.pod_mach.Re']
        T_tunnel[0,i] = prob['TubeAndPod.tube.temp_boundary']
        L_pod[0,i] = prob['TubeAndPod.L_pod']
        L_bat[0,i] = prob['TubeAndPod.pod.drivetrain.battery_length']
        power[0,i] = -1.0*prob['TubeAndPod.pod.cycle.comp.power']
        steady_vac[0,i] = -1.0*prob['TubeAndPod.tube.comp.power']
        total_energy[0,i] = prob['TubeAndPod.cost.total_energy_cost']

        # f.write('%10.2f \t %10.4f \t %10.0f \t %10.4f \t %10.4f \t %10.4f \r\n' % (p_tunnel[i], A_tube[0,i], Re[0,i], power[0,i], steady_vac[0,i], total_energy[0,i]))
    
    # f.close()
    plt.plot(p_tunnel, A_tube[0,:], 'b-', linewidth = 2.0)
    plt.xlabel('Tube Pressure (Pa)', fontsize = 16, fontweight = 'bold')
    plt.ylabel('Tube Area (m^2)', fontsize = 16, fontweight = 'bold')
    plt.show()
    plt.plot(p_tunnel, steady_vac[0,:], 'r-', linewidth = 2.0)
    plt.xlabel('Tube Pressure (Pa)', fontsize = 16, fontweight = 'bold')
    plt.ylabel('Vacuum Power (hp)', fontsize = 16, fontweight = 'bold')
    plt.show()
    plt.plot(p_tunnel, total_energy[0,:]/(1.0e6), 'r-', linewidth = 2.0)
    plt.xlabel('Tube Pressure (Pa)', fontsize = 16, fontweight = 'bold')
    plt.ylabel('Total Energy Cost per Year (Million USD)', fontsize = 16, fontweight = 'bold')
    plt.show()

    # plt.plot(p_tunnel, Re[0,:])
    # plt.show()
    # plt.plot(p_tunnel, T_tunnel[0,:])
    # plt.show()
    # plt.plot(p_tunnel, L_pod[0,:])
    # plt.show()
    # plt.plot(p_tunnel, L_bat[0,:])
    # plt.show()
    # plt.plot(p_tunnel, power[0,:])
    # plt.show()

    # print('\n')
    # print('------ Freestream and Pod Inputs ------')
    # print('tube pressure                      %f Pa' % prob['des_vars.tube_pressure'])
    # print('pod mach number                    %f' % prob['des_vars.pod_mach'])
    # print('compressor area inlet              %f m**2' % prob['des_vars.comp_inlet_area'])
    # print('passenger cross sectional area     %f m**2' % prob['des_vars.A_payload'])
    # print('Pod drag coefficient               %f' % prob['des_vars.Cd'])
    # print('Passengers per pod                 %.0f passengers' % prob['des_vars.n_passengers'])
    # print('Time between departures            %f s' % prob['des_vars.pod_period'])

    # print('\n')
    # print('------ Cycle Outputs ------')
    # print('Mass Flow                          %f kg/s' % prob['TubeAndPod.pod.cycle.FlowPathInputs.m_dot'])
    # print('compressor mass                    %f kg' % prob['TubeAndPod.pod.cycle.comp_mass'])
    # print('compressor power                   %f hp' % prob['TubeAndPod.pod.cycle.comp.power'])
    # print('compressor trq                     %f ft-lbs' % prob['TubeAndPod.pod.cycle.comp.trq'])
    # print('duct area                          %f in**2' % prob['TubeAndPod.pod.cycle.comp.Fl_O:stat:area'])
    # print('nozzle exit temp                   %f degR' % prob['TubeAndPod.pod.nozzle.Fl_O:tot:T'])
    # print('nozzle mass flow                   %f kg/s' % prob['TubeAndPod.pod.nozzle.Fl_O:stat:W'])
    # print('nozzle thrust                      %f lbs' % prob['TubeAndPod.pod.nozzle.Fg'])
    # print('ram drag                           %f lbs' % prob['TubeAndPod.pod.inlet.F_ram'])
    # print('net thrust                         %f lbs' % (prob['TubeAndPod.pod.nozzle.Fg']-prob['TubeAndPod.pod.inlet.F_ram']))

    # print('\n')
    # print('------ Drivetrain Outputs ------')
    # print('battery length                     %f cm' % prob['TubeAndPod.pod.drivetrain.battery_length'])
    # print('battery volume                     %f cm**3' % prob['TubeAndPod.pod.drivetrain.battery_volume'])
    # print('motor length                       %f m' % prob['TubeAndPod.pod.drivetrain.motor_length'])
    # print('battery mass                       %f kg' % prob['TubeAndPod.pod.drivetrain.battery_mass'])
    # print('motor mass                         %f kg' % prob['TubeAndPod.pod.drivetrain.motor_mass'])

    # print('\n')
    # print('------ Pod Mass and Geometry Outputs ------')
    # print('pod length                         %f m' % prob['TubeAndPod.L_pod'])
    # print('pod cross section                  %f m**2' % prob['TubeAndPod.pod.pod_geometry.A_pod'])
    # print('pod diameter                       %f m' % prob['TubeAndPod.pod.pod_geometry.D_pod'])
    # print('planform area                      %f m**2' % prob['TubeAndPod.S']) 
    # print('inlet area                         %f m**2' % prob['TubeAndPod.pod.pod_mach.A_inlet'])
    # print('pod mass w/o magnets               %f kg' % prob['TubeAndPod.pod.pod_mass.pod_mass'])
    # print('mag mass                           %f kg' % prob['TubeAndPod.pod.levitation_group.Mass.m_mag'])
    # print('total pod mass                     %f kg' % prob['TubeAndPod.total_pod_mass'])

    # print('\n')
    # print('------ Tube Outputs ------')
    # print('tube cross sectional area          %f m**2' % prob['TubeAndPod.pod.A_tube'])
    # print('tube temperature                   %f K' % prob['TubeAndPod.tube.temp_boundary'])
    # print('power per booster section          %f W' % prob['TubeAndPod.tube.PropMech.pwr_req'])
    # print('number of vacuum pumps             %.0f pumps' % np.ceil(prob['TubeAndPod.tube.Vacuum.number_pumps']))
    # print('steady sate vacuum power           %f hp' % prob['TubeAndPod.tube.comp.power'])
    # print('tube mass per unit length          %f kg/m' % prob['TubeAndPod.tube.Struct.m_prime'])
    # print('distance between pylons            %f m' % prob['TubeAndPod.tube.Struct.dx'])

    # print('\n')
    # print('------ Cost Results ------')
    # print('number of pods                     %.0f pods' % prob['TubeAndPod.cost.num_pods'])
    # print('structural cost per unit length    %f USD/m' % prob['TubeAndPod.tube.Struct.total_material_cost'])
    # print('populsion enrgy cost per year      %f USD' % prob['TubeAndPod.cost.prop_energy_cost'])
    # print('estimated ticket cost              %f USD' % prob['TubeAndPod.cost.ticket_cost'])

    # print('\n')
