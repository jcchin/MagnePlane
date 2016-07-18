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
    p_tube : float
        pressure of the tube (kg)
    M_pod : float
        Mach number of the pod
    A_payload : float
    	cross sectional area of passenger compartment
    M_duct : float
    	Mach number of the duct
    w_track : float
        width of the track (m)
    prc : float
    	pressure ratio of the compressor
    n_passengers : float
    	number of passengers
    T_tunnel : float
    	tunnel temperature (K)
	p_tunnel : float
    	tunnel pressure (Pa)
    T_ambient : float
    	ambient temperature (K)
    des_time : float
        time until design power point (h)
    time_of_flight : float
        total mission time (h)
    motor_max_current : float
        max motor phase current (A)
    motor_oversize_factor : float
        scales peak motor power by this figure
    inverter_efficiency : float
        power out / power in (W)
    battery_cross_section_area : float
        cross_sectional area of battery used to compute length (cm**2)

    Returns
    -------
    mag_drag : float
        magnetic drag from levitation system (N)
    nozzle.Fl_O:stat:W : float
        Pod exit flow rate from Cycle (kg/s)
    nozzle.Fl_O:tot:T : float
        Pod exit temperature from Cycle (K)
    nozzle.Fg : float
        Nozzle thrust from Cycle (N)
    inlet.F_ram : float
        Inlet ram drag from Cycle (N)
    A_tube : float
    	Area of the tube (m**2)
    S : float
    	platform area of pod (m**2)

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

        self.add('pod_mass', PodMass(), promotes=['pod_mass'])
        self.add('drivetrain', Drivetrain(), promotes=['des_time', 'time_of_flight', 'motor_max_current', 'inverter_efficiency',
        											   'motor_oversize_factor', 'battery_cross_section_area'])
        self.add('levitation_group', LevGroup(), promotes=['w_track', 'mag_drag'])
        self.add('pod_mach', PodMach(), promotes=['p_tube', 'M_pod', 'A_tube', 'prc', 'T_ambient'])
        self.add('cycle', Cycle(), promotes=['nozzle.Fg', 'inlet.F_ram','M_pod', 'p_tunnel', 'nozzle.Fl_O:stat:W',
        									 'nozzle.Fl_O:tot:T', 'T_tunnel', 'p_tunnel', 'M_pod'])
        self.add('pod_geometry', PodGeometry(), promotes=['A_payload', 'S', 'n_passengers'])

        self.connect('pod_geometry.A_pod', 'pod_mach.A_pod')
        self.connect('pod_geometry.L_pod', ['pod_mach.L', 'pod_mass.pod_len', 'levitation_group.l_pod'])
        self.connect('drivetrain.motor_mass', 'pod_mass.motor_mass')
        self.connect('drivetrain.battery_mass', 'pod_mass.battery_mass')
        self.connect('drivetrain.motor_length', 'pod_geometry.L_motor')
        self.connect('pod_mass', 'levitation_group.m_pod')
        self.connect('levitation_group.m_mag', 'pod_mass.mag_mass')
        self.connect('pod_geometry.D_pod', 'pod_mass.podgeo_d')
        self.connect('cycle.comp_mass', 'pod_mass.comp_mass')
        self.connect('cycle.comp.power', 'drivetrain.design_power')
        self.connect('cycle.comp.trq', 'drivetrain.design_torque')
        self.connect('cycle.comp_len', 'pod_geometry.L_comp')
        self.connect('cycle.FlowPath.comp.Fl_O:stat:area', 'pod_geometry.A_duct')

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