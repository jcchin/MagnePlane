"""
Group for Pod components containing the following components:
Cycle Group, Pod Mach (Aero), DriveTrain group, Geometry, Levitation group, and Pod Mass
"""
from openmdao.api import Component, Group, Problem, IndepVarComp
from hyperloop.Python.pod.pod_mass import PodMass
from hyperloop.Python.pod.drivetrain.drivetrain import Drivetrain
from hyperloop.Python.pod.pod_mach import PodMach
from hyperloop.Python.pod.cycle.cycle_group import Cycle
from hyperloop.Python.pod.pod_geometry import PodGeometry
from hyperloop.Python.pod.magnetic_levitation.levitation_group import LevGroup
from openmdao.api import Newton, ScipyGMRES
class PodGroup(Group):
    """TODOs

    Params
    ------
    p_tunnel : float
        Pressure of air in tube.  Default value is 850 Pa.  Value will come from vacuum component
    M_pod : float
        pod Mach number. Default value is .8
    T_ambient : float
        Tunnel ambient temperature. Default value is 298 K
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
    p_tunnel : float
        Pressure of air in tube.  Default value is 850 Pa.  Value will come from vacuum component
    M_pod : float
        pod Mach number. Default value is .8
    n_passengers : float
        Number of passengers per pod. Default value is 28
    A_payload : float
        Cross sectional area of passenger compartment. Default value is 2.72

    Returns
    -------
    A_tube : float
        will return optimal tunnel area based on pod Mach number
    S : float
        Platform area of the pod
    mag_drag : float
        magnetic drag from levitation system (N)
    pod_mass : float
            Pod Mass (kg)

    References
    ----------
    .. [1] Friend, Paul. Magnetic Levitation Train Technology 1. Thesis.
       Bradley University, 2004. N.p.: n.p., n.d. Print.
    """
    def __init__(self):
        super(PodGroup, self).__init__()

        self.nl_solver = Newton()
        # self.nl_solver.options['maxiter'] = 1000
        # self.nl_solver.options['atol'] = 0.0001

        self.ln_solver = ScipyGMRES()
        # self.ln_solver.options['maxiter'] = 100

        self.add('cycle', Cycle(), promotes=['nozzle.Fg', 'inlet.F_ram','M_pod', 'p_tunnel', 'nozzle.Fl_O:stat:W',
                                             'nozzle.Fl_O:tot:T', 'T_tunnel', 'p_tunnel', 'M_pod', 'fl_start.MN_target'])
        self.add('pod_mach', PodMach(), promotes=['p_tunnel', 'M_pod', 'T_ambient', 'A_tube'])
        self.add('drivetrain', Drivetrain(), promotes=['des_time', 'time_of_flight', 'motor_max_current', 'motor_LD_ratio',
                                                       'inverter_efficiency', 'motor_oversize_factor', 'battery_cross_section_area'])
        self.add('pod_geometry', PodGeometry(), promotes=['A_payload', 'n_passengers', 'S'])
        self.add('levitation_group', LevGroup(), promotes=['mag_drag'])
        self.add('pod_mass', PodMass(), promotes=['pod_mass'])

        # TODO Connects Cycle outputs to downstream components
        self.connect('cycle.comp_mass', 'pod_mass.comp_mass')
        self.connect('cycle.comp.power', 'drivetrain.design_power')
        self.connect('cycle.comp.trq', 'drivetrain.design_torque')
        self.connect('cycle.comp_len', 'pod_geometry.L_comp')
        self.connect('cycle.FlowPath.comp.Fl_O:stat:area', 'pod_geometry.A_duct')
        
        # Connects Drivetrain outputs to downstream components
        self.connect('drivetrain.battery_mass', 'pod_mass.battery_mass')
        self.connect('drivetrain.battery_length', 'pod_geometry.L_bat')
        self.connect('drivetrain.motor_mass', 'pod_mass.motor_mass')
        self.connect('drivetrain.motor_length', 'pod_geometry.L_motor')

        # Connects Pod Geometry outputs to downstream components
        self.connect('pod_geometry.A_pod', 'pod_mach.A_pod')
        self.connect('pod_geometry.L_pod', ['pod_mach.L', 'pod_mass.pod_len', 'levitation_group.l_pod'])
        self.connect('pod_geometry.BF', 'pod_mach.BF')
        self.connect('pod_geometry.D_pod', 'pod_mass.podgeo_d')

        # Connects Levitation outputs to downstream components
        self.connect('levitation_group.m_mag', 'pod_mass.mag_mass')

        # Connects Pod Mass outputs to downstream components
        self.connect('pod_mass', 'levitation_group.m_pod')

if __name__ == "__main__":

    prob = Problem()
    root = prob.root = Group()
    root.add('Pod', PodGroup())

    params = (('p_tube', 850.0, {'units' : 'Pa'}),
             ('M_pod', .8, {'units' : 'unitless'}),
             ('A_payload', 1.4),
             ('w_track', 2.0, {'units': 'm'}),
             ('prc', 12.5, {'units' : 'unitless'}),
             ('vehicleMach', 0.8),
             ('inlet_MN', 0.65),
             ('P', 0.1885057735, {'units': 'psi'}),
             ('T', 591.0961831, {'units': 'degR'}),
             ('W', 4.53592, {'units': 'kg/s'}),
             ('PsE', 0.59344451, {'units': 'psi'}),
             ('cmpMach', 0.65), )

    prob.root.add('des_vars', IndepVarComp(params))
    prob.root.connect('des_vars.PsE', 'Pod.cycle.FlowPath.nozzle.Ps_exhaust')
    prob.root.connect('des_vars.P', 'Pod.cycle.FlowPath.fl_start.P')
    prob.root.connect('des_vars.T', 'Pod.cycle.FlowPath.fl_start.T')
    prob.root.connect('des_vars.W', 'Pod.cycle.FlowPath.fl_start.W')
    prob.root.connect('des_vars.vehicleMach', 'Pod.cycle.FlowPath.fl_start.MN_target')
    prob.root.connect('des_vars.inlet_MN', 'Pod.cycle.FlowPath.inlet.MN_target')
    prob.root.connect('des_vars.p_tube', 'Pod.p_tube')
    prob.root.connect('des_vars.M_pod', 'Pod.M_pod')
    prob.root.connect('des_vars.A_payload', 'Pod.A_payload')
    prob.root.connect('des_vars.w_track', 'Pod.w_track')

    prob.setup()
    prob.root.list_connections()
    prob.run()

    print('A Tube: %f' % prob['Pod.A_tube'])
    print('Mag Drag :%f' % prob['Pod.mag_drag'])
    print('S :%f' % prob['Pod.S'])
    print('nozzle.Fl_O:tot:T %f' % prob['Pod.nozzle.Fl_O:tot:T'])
    print('nozzle.Fl_O:stat:W %f' % prob['Pod.nozzle.Fl_O:stat:W'])
    print('nozzle.Fg %f' % prob['Pod.nozzle.Fg'])