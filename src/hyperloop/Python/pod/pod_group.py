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
    pod_mach : float
        Vehicle mach number (unitless)
    tube_pressure : float
        Tube total pressure (Pa)
    tube_temp : float
        Tube total temperature (K)
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
    nozzle.Fg : float
        Nozzle thrust (lbf)
    inlet.F_ram : float
        Ram drag (lbf)
    nozzle.Fl_O:tot:T : float
        Total temperature at nozzle exit (degR)
    nozzle.Fl_O:stat:W : float
        Total mass flow rate at nozzle exit (lbm/s)
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

        self.add('cycle', Cycle(), promotes=['comp.map.PRdes', 'nozzle.Ps_exhaust', 'comp_inlet_area',
                                             'nozzle.Fg', 'inlet.F_ram', 'nozzle.Fl_O:tot:T', 'nozzle.Fl_O:stat:W',
                                             'pod_mach', 'tube_pressure', 'tube_temp'])
        self.add('pod_mach', PodMach(), promotes=['A_tube'])
        self.add('drivetrain', Drivetrain(), promotes=['des_time', 'time_of_flight', 'motor_max_current', 'motor_LD_ratio',
                                                       'inverter_efficiency', 'motor_oversize_factor', 'battery_cross_section_area'])
        self.add('pod_geometry', PodGeometry(), promotes=['A_payload', 'n_passengers', 'S'])
        self.add('levitation_group', LevGroup(), promotes=['mag_drag'])
        self.add('pod_mass', PodMass(), promotes=['pod_mass'])

        # Connects pod group level variables to downstream components
        self.connect('pod_mach', 'pod_mach.M_pod')
        self.connect('tube_pressure', 'pod_mach.p_tube')
        self.connect('tube_temp', 'pod_mach.T_ambient')

        # Connects cycle group outputs to downstream components
        self.connect('cycle.comp_len', 'pod_geometry.L_comp')
        self.connect('cycle.comp_mass', 'pod_mass.comp_mass')
        self.connect('cycle.comp.power', 'drivetrain.design_power')
        self.connect('cycle.comp.trq', 'drivetrain.design_torque')
        self.connect('cycle.comp.Fl_O:stat:area', 'pod_geometry.A_duct')
        
        # Connects Drivetrain outputs to downstream components
        self.connect('drivetrain.battery_mass', 'pod_mass.battery_mass')
        self.connect('drivetrain.battery_length', 'pod_geometry.L_bat')
        self.connect('drivetrain.motor_mass', 'pod_mass.motor_mass')
        self.connect('drivetrain.motor_length', 'pod_geometry.L_motor')

        # Connects Pod Geometry outputs to downstream components
        self.connect('pod_geometry.A_pod', 'pod_mach.A_pod')
        self.connect('pod_geometry.L_pod', ['pod_mach.L', 'pod_mass.pod_len', 'levitation_group.l_pod'])
        self.connect('pod_geometry.BF', 'pod_mach.BF')
        self.connect('pod_geometry.D_pod', ['pod_mass.podgeo_d', 'levitation_group.d_pod'])

        # Connects Levitation outputs to downstream components
        self.connect('levitation_group.m_mag', 'pod_mass.mag_mass')

        # Connects Pod Mass outputs to downstream components
        self.connect('pod_mass', 'levitation_group.m_pod')

if __name__ == "__main__":

    prob = Problem()
    root = prob.root = Group()
    root.add('Pod', PodGroup())

    params = (('comp_inlet_area', 2.3884, {'units': 'm**2'}),
              ('comp_PR', 6.0, {'units': 'unitless'}),
              ('PsE', 0.59344451, {'units': 'psi'}),
              ('des_time', 1.0),
              ('time_of_flight', 2.0),
              ('motor_max_current', 42.0),
              ('motor_LD_ratio', 0.83),
              ('motor_oversize_factor', 1.0),
              ('inverter_efficiency', 1.0),
              ('battery_cross_section_area', 1.0, {'units': 'cm**2'}),
              ('n_passengers', 28),
              ('A_payload', 2.72),
              ('pod_mach_number', .8, {'units': 'unitless'}),
              ('tube_pressure', 850., {'units': 'Pa'}),
              ('tube_temp', 320., {'units': 'K'}))

    prob.root.add('des_vars', IndepVarComp(params))
    prob.root.connect('des_vars.comp_inlet_area', 'Pod.comp_inlet_area')
    prob.root.connect('des_vars.comp_PR', 'Pod.comp.map.PRdes')
    prob.root.connect('des_vars.PsE', 'Pod.nozzle.Ps_exhaust')
    prob.root.connect('des_vars.des_time', 'Pod.des_time')
    prob.root.connect('des_vars.time_of_flight', 'Pod.time_of_flight')
    prob.root.connect('des_vars.motor_max_current', 'Pod.motor_max_current')
    prob.root.connect('des_vars.motor_LD_ratio', 'Pod.motor_LD_ratio')
    prob.root.connect('des_vars.motor_oversize_factor', 'Pod.motor_oversize_factor')
    prob.root.connect('des_vars.inverter_efficiency', 'Pod.inverter_efficiency')
    prob.root.connect('des_vars.battery_cross_section_area', 'Pod.battery_cross_section_area')
    prob.root.connect('des_vars.n_passengers', 'Pod.n_passengers')
    prob.root.connect('des_vars.A_payload', 'Pod.A_payload')
    prob.root.connect('des_vars.pod_mach_number', 'Pod.pod_mach')
    prob.root.connect('des_vars.tube_pressure', 'Pod.tube_pressure')
    prob.root.connect('des_vars.tube_temp', 'Pod.tube_temp')

    prob.setup()
    prob.root.list_connections()
    prob.run()

    print('nozzle.Fg: %f' % prob['Pod.nozzle.Fg'])
    print('inlet.F_ram: %f' % prob['Pod.inlet.F_ram'])
    print('nozzle.Fl_O:tot:T: %f' % prob['Pod.nozzle.Fl_O:tot:T'])
    print('nozzle.Fl_O:stat:W: %f' % prob['Pod.nozzle.Fl_O:stat:W'])
    print('A Tube: %f' % prob['Pod.A_tube'])
    print('Mag Drag :%f' % prob['Pod.mag_drag'])
    print('S :%f' % prob['Pod.S'])
    print('pod_mass :%f' % prob['Pod.pod_mass'])
