from __future__ import print_function
import numpy as np
from openmdao.api import Component, Problem, Group

class CompressorLen(Component):
    """
    The CompressorLen class represents a compressor length component
    in an OpenMDAO model.

    A `CompressorLen` models length of a compressor that uses NPSS data
    to obtain enthalpy data,and mass_flow for a particular pressure ratio.
    It also uses a correlation derived by Micheal Tong at NASA Glenn Center
    to obtain Compressor Length.

    Params
    ------
    h_in : float
        Heat in. (kJ/kg)
    h_out : float
        Heat out. (kJ/kg)
    comp_inletArea : float
        Compressor Inlet Area. (m**2)
    h_stage : float
        enthalpy added per stage. (kJ/kg)

    Returns
    -------
    comp_len : float
        Length of Compressor (m)

    References
    -----
    .. [1] Michael Tong Correlation used.
    .. [2] NASA-Glenn NPSS compressor cycle model.
    """
    def __init__(self):
        super(CompressorLen, self).__init__()
        self.add_param('h_in',
                       val=0.,
                       desc='Enthalpy in',
                       units='kJ/kg')
        self.add_param('h_out',
                       val=207.,
                       desc='Enthalpy out',
                       units='kJ/kg')

        self.add_param('comp_inletArea',
                       val=1.287,
                       desc='Compressor Inlet Area',
                       units='m**2')
        self.add_param('h_stage',
                       val=58.2,
                       desc='enthalpy added per stage',
                       units='kJ/kg')
        self.add_output('comp_len',
                    val=1.0,
                    desc='Length of Compressor',
                    units='m')

    def solve_nonlinear(self, params, unknowns, resids):
        comp_inletArea = params['comp_inletArea']
        h_out = params['h_out']
        h_in = params['h_in']
        h_stage = params['h_stage']

        #Calculating Length of Compressor.
        comp_r = np.sqrt(comp_inletArea/np.pi)
        hub_tip_ratio = (np.sqrt(comp_r**2 - (comp_inletArea / 3.1416))) / comp_r
        no_stages = ((h_out - h_in)/ h_stage) + 1


        unknowns['comp_len'] = 0.2 + (0.234 - 0.218*hub_tip_ratio)*(no_stages)*comp_r*2

if __name__ == "__main__":
    top = Problem()
    root = top.root = Group()

    root.add('CompressorLen', CompressorLen())

    top.setup()
    top.run()

    print('Comp_Len %f' % top['CompressorLen.comp_len'])
