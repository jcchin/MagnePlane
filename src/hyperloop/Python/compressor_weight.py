import numpy as np
from openmdao.api import Component, Problem, Group



class CompressorWeight(Component):
    """The CompressorWeight class represents a compressor weight component
        in an OpenMDAO model.

        A `CompressorWeight` models weight of a compressor that uses NPSS data
        to obtain enthalpy data,and mass_flow for a particular pressure ratio.
        It also uses a correlation derived by Miceal Tong at NASA Glenn Center
        to obtain Compressor Weight.
    Params
        ----
        comp_eff: float
            Compressor Efficiency. (unitless)
        mass_flow: float
            Mass Flow for Compressor. (kJ/kg)
        h_in: float
            Heat in. (kJ/kg)
        h_out: float
            Heat out. (kJ/kg)
        comp_inletR: float
            Compressor Inlet Radius. (m)
    Outputs
    -------
    comp_weight : float
        Compressor Weight (kg)

    References
    -----
    .. [1] Michael Tong Correlation used.

    .. [2] NASA-Glenn NPSS compressor cycle model.

    """

    def __init__(self):
        """Initializes a `CompressorWeight` object

        Sets up the given Params/Outputs of the OpenMDAO `CompressorWeight`
        component,initializes their shape, and
        sets them to their default values.
        """
        super(CompressorWeight, self).__init__()

        # set input
        self.add_param('comp_eff',
                       val=91.,
                       desc='Compressor Efficiency',
                       units='unitless')
        self.add_param('mass_flow',
                       val=317.52,
                       desc='Mass Flow Rate',
                       units='kg/s')
        self.add_param('h_in',
                       val=0.,
                       desc='Heat-in',
                       units='kJ/kg')
        self.add_param('h_out',
                       val=486.13,
                       desc='Heat-out',
                       units='kJ/kg')
        self.add_param('comp_inletR',
                       val=0.64,
                       desc='Compressor Inlet Radius',
                       units='m')

        # set output
        self.add_output('comp_weight',
                        val=0.1,
                        desc='Compressor Weight',
                        units='kg')

    def solve_nonlinear(self, params, unknowns, resids):
        """Runs the `CompressorWeight` component and sets its respective outputs to their calculated results

        Args
        ----------
        params : `VecWrapper`
            `VecWrapper` containing parameters

        unknowns : `VecWrapper`
            `VecWrapper` containing outputs and states

        resids : `VecWrapper`
            `VecWrapper` containing residuals

        """
        # defines the parameters
        comp_eff = params['comp_eff']
        mass_flow = params['mass_flow']
        h_in = params['h_in']
        h_out = params['h_out']
        comp_inletR = params['comp_inletR']

        # uses correlation to obtain compressor weight
        unknowns['comp_weight'] = 299.2167 * (np.power(comp_inletR,2)) + 0.007418 * (
            (mass_flow * (h_out - h_in)) / (comp_eff / 100)) + 37.15



class PodWeight(Component):
    def __init__(self):
        super(PodWeight, self).__init__()
        self.add_param('mag_weight',
                        val=1.,
                        desc='Mass of permanent magnets',
                        units='kg')
        self.add_param('podgeo_weight',
                       val=1.,
                       desc='Mass of permanent magnets',
                       units='kg')
        self.add_param('motor_weight',
                       val=1.,
                       desc='Mass of motor',
                       units='kg')
        self.add_param('battery_weight',
                       val=1.,
                       desc='Mass of battery',
                       units='kg')
        self.add_param('comp_weight',
                       val=1.,
                       desc='Compressor Weight',
                       units='kg')

        self.add_output('pod_weight',
                       val=1.,
                       desc='Pod Mass',
                       units='kg')


    def solve_nonlinear(self, params, unknowns, resids):
        mag_weight = params['mag_weight']
        podgeo_weight = params['podgeo_weight']
        motor_weight = params['motor_weight']
        battery_weight = params['battery_weight']
        comp_weight = params['comp_weight']


        unknowns['pod_weight'] = mag_weight + podgeo_weight + motor_weight + battery_weight + comp_weight




if __name__ == '__main__':
    # set up problem.
    root = Group()
    prob = Problem(root)
    prob.root.add('comp1', CompressorWeight())
    prob.root.add('comp2', PodWeight())
    prob.root.connect('comp1.comp_weight', 'comp2.comp_weight')
    prob.setup()
    prob.root.list_connections()
    prob.run()

    # prints the following results.
    print('Compressor Weight(kg) : %f' % prob['comp1.comp_weight'])
    print('Pod Weight(kg) : %f' % prob['comp2.pod_weight'])