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

class PodGroup(Group):
    def __init__(self):
        super(PodGroup, self).__init__()

        self.add('pod_mass', PodMass(), promotes=['pod_mass'])
        self.add('drivetrain', Drivetrain(), promotes=['des_time', 'time_of_flight', 'motor_max_current', 'inverter_efficiency',
        											   'motor_oversize_factor', 'battery_cross_section_area'])
        self.add('levitation_group', LevGroup(), promotes=['w_track', 'mag_drag'])
        self.add('pod_mach', PodMach(), promotes=['p_tube', 'M_pod', 'A_tube', 'prc', 'T_ambient'])
        self.add('cycle', Cycle(), promotes=['nozzle.Fg', 'inlet.F_ram','M_pod', 'p_tunnel', 'nozzle.Fl_O:stat:W',
        									 'nozzle.Fl_O:tot:T'])
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
        self.connect('cycle.comp.Fl_O:stat:area', 'pod_geometry.A_duct')

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