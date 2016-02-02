from openmdao.core.component import Component

class Mass(Component):
    def __init__(self):
        super(Mass, self).__init__()

        # --- Component Densities
        self.add_param('battery', 2100.0, desc='battery density', units='kg/m**3')
        self.add_param('chassis', 1.3, desc='chassis material density', units='kg/m**3')
        self.add_param('skin', 30.0, desc='skin material density', units='kg/m**3')

        self.add_output('total_mass', 0.0, desc='total mass of system', units='kg')

    def solve_nonlinear(self, params, unknowns, resids):

        # --- External call to goemetry tool (SolidWorks/OpenCSM) to get component volumes


        # --- Sum component masses to compute system masss       
        unknowns['total_mass'] = params['battery'] + params['chassis'] + params['skin']

if __name__ == '__main__':
    from openmdao.core.problem import Problem
    from openmdao.core.group import Group

    p = Problem(root=Group())
    p.root.add('comp', Mass())
    p.setup()
    p.root.list_connections()
    p.run()

    print 'mass (Kg): %f' % p['comp.total_mass']

