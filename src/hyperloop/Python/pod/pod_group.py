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

        self.add('pod_mass', PodMass())
        self.add('drivetrain', Drivetrain())
        self.add('levitation_group', LevGroup(), promotes=['w_track', 'cost', 'w_track', 'mag_drag'])
        self.add('pod_mach', PodMach(), promotes=['p_tube', 'M_pod', 'A_tube', 'prc'])
        self.add('cycle', Cycle(), promotes=['comp.trq', 'comp.power', 'comp.Nmech', 'inlet.Fl_O:stat:area', 'nozzle.Fg', 
                                              'inlet.F_ram', 'nozzle.Fl_O:tot:T', 'nozzle.Fl_O:stat:W', 'fl_start.Fl_O:stat:P',
                                              'fl_start.Fl_O:stat:T', 'fl_start.Fl_O:tot:P', 'fl_start.Fl_O:tot:T',
                                              'fl_start.Fl_O:stat:rho', 'fl_start.Fl_O:stat:V', 'inlet.Fl_O:stat:MN',
                                              'inlet.Fl_O:stat:P', 'inlet.Fl_O:stat:T', 'inlet.Fl_O:tot:P', 'inlet.Fl_O:tot:T',
                                              'inlet.Fl_O:stat:W', 'comp.Fl_O:stat:MN', 'comp.Fl_O:stat:area',
                                              'comp.Fl_O:stat:P', 'comp.Fl_O:stat:T', 'M_pod', 'p_tunnel', 'T_tunnel'])
        self.add('pod_geometry', PodGeometry(), promotes=['A_payload', 'p_tunnel', 'M_dif', 'M_duct', 'T_tunnel', 'S', 'M_pod'])

        self.connect('pod_geometry.A_pod', 'pod_mach.A_pod')
        self.connect('pod_geometry.L_pod', ['pod_mach.L', 'pod_mass.pod_len'])
        self.connect('drivetrain.motor.mass', 'pod_mass.motor_mass')
        self.connect('drivetrain.battery.battery_mass', 'pod_mass.battery_mass')
        #self.connect('drivetrain.??', 'pod_geometry.L_bat')
        self.connect('drivetrain.motor.motor_size.l_base', 'pod_geometry.L_motor')
        self.connect('pod_mass.pod_mass', 'levitation_group.m_pod')
        self.connect('pod_geometry.L_pod', 'levitation_group.l_pod')
        self.connect('pod_geometry.D_pod', 'pod_mass.podgeo_d')
        self.connect('cycle.comp_mass', 'pod_mass.comp_mass')
        self.connect('pod_mass.BF', ['pod_mach.BF', 'pod_geometry.BF'])

        #npss cycle connections
        #self.connect('cycle.comp.Nmech', 'drivetrain.operating_rpm')
        self.connect('comp.power', 'drivetrain.design_power')
        self.connect('cycle.comp_mass', 'pod_mass.comp_mass')
        self.connect('comp.Fl_O:stat:area', 'pod_geometry.A_inlet')

if __name__ == "__main__":

    prob = Problem()
    root = prob.root = Group()
    root.add('Pod', PodGroup())

    params = (('p_tube', 850.0, {'units' : 'Pa'}),
             ('M_pod', .8, {'units' : 'unitless'}),
             ('A_payload', 1.4),
             ('p_tunnel', 850.0, {'units' : 'Pa'}),
             ('M_dif', 0.6),
             ('M_duct', 0.3),
             ('T_tunnel', 298.0, {'units' : 'K'}),
             ('w_track', 2.0, {'units': 'm'}),
             ('prc', 12.5, {'units' : 'unitless'}))

    prob.root.add('des_vars', IndepVarComp(params))
    prob.root.connect('des_vars.p_tube', 'Pod.p_tube')
    prob.root.connect('des_vars.M_pod', 'Pod.M_pod')
    prob.root.connect('des_vars.A_payload', 'Pod.A_payload')
    prob.root.connect('des_vars.p_tunnel', 'Pod.p_tunnel')
    prob.root.connect('des_vars.M_dif', 'Pod.M_dif')
    prob.root.connect('des_vars.M_duct', 'Pod.M_duct')
    prob.root.connect('des_vars.T_tunnel', 'Pod.T_tunnel')
    prob.root.connect('des_vars.w_track', 'Pod.w_track')

    prob.setup()
    prob.root.list_connections()
    prob.run()

    print('Tube Temperature: %f' % prob['Tube.Thermal.tubetemp'])
    print(':%f' % prob['Cycle.'])
    print(':%f' % prob['Aero.'])
    print('%f' % prob['Geometry.'])
    print('%f' % prob['LevGroup.'])
    print('%f' % prob['Weight.'])