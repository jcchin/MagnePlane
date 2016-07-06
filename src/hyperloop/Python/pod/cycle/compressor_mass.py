from __future__ import print_function
import numpy as np
from openmdao.api import Component

class CompressorMass(Component):
    """The CompressorMass class represents a compressor mass component
        in an OpenMDAO model.

        A `CompressorMass` models mass of a compressor that uses NPSS data
        to obtain enthalpy data,and mass_flow for a particular pressure ratio.
        It also uses a correlation derived by Miceal Tong at NASA Glenn Center
        to obtain Compressor Mass.
    Params
        ----
        comp_eff : float
            Compressor Efficiency. (unitless)
        mass_flow : float
            Mass Flow for Compressor. (kJ/kg)
        h_in : float
            Heat in. (kJ/kg)
        h_out : float
            Hea    self.t out. (kJ/kg)
        comp_inletR : float
            Compressor Inlet Radius. (m)
    Outputs
    -------
    comp_mass : float
        Compressor Mass (kg)

    References
    -----
    .. [1] Michael Tong Correlation used.

    .. [2] NASA-Glenn NPSS compressor cycle model.

    """

    def __init__(self):
        """Initializes a `CompressorMass` object

        Sets up the given Params/Outputs of the OpenMDAO `CompressorMass`
        component,initializes their shape, and
        sets them to their default values.
        """
        super(CompressorMass, self).__init__()

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
        self.add_output('comp_mass',
                        val=0.1,
                        desc='Compressor Mass',
                        units='kg')

    def solve_nonlinear(self, params, unknowns, resids):
        """Runs the `CompressorMass` component and sets its respective outputs to their calculated results

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

        # uses correlation to obtain compressor mass
        unknowns['comp_mass'] = 299.2167 * (np.power(comp_inletR,2)) + 0.007418 * (
            (mass_flow * (h_out - h_in)) / (comp_eff / 100)) + 37.15



