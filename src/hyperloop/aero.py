from openmdao.core.component import Component

class Aero(Component):
    '''Placeholder for real aerodynamic calculations of the capsule'''
    def __init__(self):
        super(Aero, self).__init__()
        self.add_param('coef_drag', 1.0, desc='capsule drag coefficient')
        self.add_param('area_frontal', 1.5, desc='frontal area of capsule', units='m**2')
        self.add_param('velocity_capsule', 600.0, desc='capsule velocity', units='m/s')
        self.add_param('rho', 0.01, desc='tube air density', units='kg/m**3')
        self.add_param('gross_thrust', 1300.0, desc='nozzle gross thrust', units='N')

        self.add_output('net_force', 0.0, desc='net force with drag considerations', units='N')
        self.add_output('drag', 0.0, desc='drag force', units='N')

    def solve_nonlinear(self, params, unknowns, resids):
        # drag = Cd * rho * Velocity ** 2 * Area / 2.0
        unknowns['drag'] = params['coef_drag'] * params['rho'] * params['velocity_capsule'] ** 2 * params['area_frontal'] / 2.0
        unknowns['net_force'] = params['gross_thrust'] - unknowns['drag']


if __name__ == '__main__':
    from openmdao.core.problem import Problem
    from openmdao.core.group import Group

    p = Problem(root=Group())
    p.root.add('aero', Aero())
    p.setup()
    p.root.list_connections()
    p.run()

    print 'drag (N): %f' % p['aero.drag']
    print 'net_force N(kW*hr): %f' % p['aero.net_force']