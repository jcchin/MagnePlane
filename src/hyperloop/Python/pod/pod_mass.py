from openmdao.api import Component, Group, Problem
from hyperloop.Python.pod.cycle.compressor_mass import CompressorMass
import numpy as np

class PodMass(Component):
    """The PodMass Component sums all the mass to find the total mass of the pod
        ----
        mag_mass : float
            Mass of permanent magnets. (kg)
        podgeo_d : float
            Diameter of pod. (m)
        al_rho : float
            Density of the aluminium (kg/m**3)
        motor_mass : float
            Mass of motor (kg)
        battery_mass : float
            Mass of battery. (kg)
        comp_mass : float
            Mass of Compressor. (kg)
        pod_len : float
            Length of pod. (m)
        comp_inletArea : float
            Area of compressor (m**2)
        BF : float
            Blockage factor (unitless)
    Outputs
    -------
        pod_mass : float
            Pod Mass (kg)
    """

    def __init__(self):
        super(PodMass, self).__init__()
        self.add_param('mag_mass',
                       val=1.,
                       desc='Mass of permanent magnets',
                       units='kg')
        self.add_param('podgeo_d',
                       val=2.,
                       desc='Pod Geometry Radius',
                       units='m')
        self.add_param('al_rho',
                       val=2800.,
                       desc='Density of Aluminium',
                       units='kg/m**3')
        self.add_param('motor_mass',
                       val=1.,
                       desc='Mass of motor',
                       units='kg')
        self.add_param('battery_mass',
                       val=1.,
                       desc='Mass of battery',
                       units='kg')
        self.add_param('comp_mass',
                       val=1.,
                       desc='Compressor Mass',
                       units='kg')
        self.add_param('pod_len',
                       val=1.,
                       desc='Length of pod',
                       units='m')
        self.add_param('BF',
                        val=.99,
                        desc='blockage factor of pod',
                        units='unitless')
        self.add_output('pod_mass',
                        val=1.,
                        desc='Pod Mass',
                        units='kg')

    def solve_nonlinear(self, params, unknowns, resids):
        mag_mass = params['mag_mass']
        podgeo_d = params['podgeo_d']
        al_rho = params['al_rho']
        motor_mass = params['motor_mass']
        battery_mass = params['battery_mass']
        comp_mass = params['comp_mass']
        pod_len = params['pod_len']
        BF = params['BF']

        #adds up the mass.
        unknowns['pod_mass'] = mag_mass + np.pi*(podgeo_d/2)**2*pod_len*al_rho*(1-BF) + motor_mass + battery_mass + comp_mass

if __name__== '__main__':
    # set up problem.
    root = Group()
    prob = Problem(root)
    prob.root.add('PodMass', PodMass())
    prob.root.add('CompressorMass', CompressorMass())
    prob.root.connect('CompressorMass.comp_mass', 'PodMass.comp_mass')
    prob.setup()
    prob.root.list_connections()
    prob.run()

    # prints the following results.
    print('Compressor Mass(kg) : %f' % prob['CompressorMass.comp_mass'])
    print('Pod Mass(kg) : %f' % prob['PodMass.pod_mass'])
    print('Blockage Factr(unitless) : %f' % prob['PodMass.BF'])