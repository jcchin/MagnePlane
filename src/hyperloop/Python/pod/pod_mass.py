from openmdao.api import Component, Group, Problem
from hyperloop.Python.pod.compressor_mass import CompressorMass

class PodMass(Component):
    def __init__(self):
        super(PodMass, self).__init__()
        self.add_param('mag_mass',
                       val=1.,
                       desc='Mass of permanent magnets',
                       units='kg')
        self.add_param('podgeo_mass',
                       val=1.,
                       desc='Mass of permanent magnets',
                       units='kg')
        self.add_param('motor_mass',
                       val=1.,
                       desc='Mass of motor',-
                       units='kg')
        self.add_param('battery_mass',
                       val=1.,
                       desc='Mass of battery',
                       units='kg')
        self.add_param('comp_mass',
                       val=1.,
                       desc='Compressor Mass',
                       units='kg')

        self.add_output('pod_mass',
                        val=1.,
                        desc='Pod Mass',
                        units='kg')

    def solve_nonlinear(self, params, unknowns, resids):
        mag_mass = params['mag_mass']
        podgeo_mass = params['podgeo_mass']
        motor_mass = params['motor_mass']
        battery_mass = params['battery_mass']
        comp_mass = params['comp_mass']

        unknowns['pod_mass'] = mag_mass + podgeo_mass + motor_mass + battery_mass + comp_mass

if __name__== '__main__':
    # set up problem.
    root = Group()
    prob = Problem(root)
    prob.root.add('comp1', PodMass())
    prob.root.add('comp2', CompressorMass())
    prob.root.connect('comp2.comp_mass', 'comp1.comp_mass')
    prob.setup()
    prob.root.list_connections()
    prob.run()

    # prints the following results.
    print('Compressor Mass(kg) : %f' % prob['comp2.comp_mass'])
    print('Pod Mass(kg) : %f' % prob['comp1.pod_mass'])