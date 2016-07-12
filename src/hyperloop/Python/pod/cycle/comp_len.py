from __future__ import print_function
import numpy as np
from openmdao.api import Component, Problem, Group

class CompressorLen(Component):
    """The CompressorMass class represents a compressor length component
        in an OpenMDAO model.
        A `CompressorMass` models length of a compressor that uses NPSS data
        to obtain enthalpy data,and mass_flow for a particular pressure ratio.
        It also uses a correlation derived by Micheal Tong at NASA Glenn Center
        to obtain Compressor Length.
    Params
        ----
        comp_inletTemp: float
            Compressor Inlet Temperature. (K)
        h_in : float
            Heat in. (kJ/kg)
        h_in : float
            Heat out. (kJ/kg)
        h_out : float
            Hea    self.t out. (kJ/kg)
        comp_inletArea : float
            Compressor Inlet Area. (m**2)
        A_inlet : float
            Inlet area of pod. (m**2)
        comp_mach : float
            Compressor Mach at Inlet. (m/s)
        M_pod : float
            Pod Mach Number. (unitless)
        T_tunnel : float
            Tunnel temperature. (K)
        gam : float
            Ratio of specific heats. (unitless)
        R : float
            Ideal gas constant. (J/(kg*K))
        p_tunnel : float
            tunnel pressure. (Pa)
        air_den : float
            air density. (kg/m**3)
        h_stage : float
            enthalpy added per stage. (kJ/kg)
    Outputs
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
        self.add_param('comp_inletTemp',
                       val=293.,
                       desc='Compressor Inlet Temperature',
                       units='K')
        self.add_param('h_in',
                       val=0.,
                       desc='Heat in',
                       units='kJ/kg')
        self.add_param('h_out',
                       val=207.,
                       desc='Heat out',
                       units='kJ/kg')

        self.add_param('comp_inletArea',
                       val=1.287,
                       desc='Compressor Inlet Area',
                       units='m**2')
        self.add_param('A_inlet',
                       val=1.1,
                       desc='Inlet area of pod',
                       units='m**2')
        self.add_param('comp_mach',
                       val=0.6,
                       desc='Compressor Mach at Inlet',
                       units='m/s')
        self.add_param('M_pod',
                       val=.8,
                       desc='Pod Mach Number',
                       units='unitless')
        self.add_param('T_tunnel',
                       val=298.0,
                       desc='Tunnel temperature',
                       units='K')
        self.add_param('gam',
                       val=1.4,
                       desc='Ratio of specific heats',
                       units='unitless')
        self.add_param('R',
                       val=29.27,
                       desc='Ideal gas constant',
                       units='J/(kg*K)')
        self.add_param('p_tunnel',
                       val=850.0,
                       desc='tunnel pressure',
                       units='Pa')
        self.add_param('h_stage',
                       val=58.2,
                       desc='enthalpy added per stage',
                       units='kg/m**3')
        self.add_output('comp_len',
                    val=1.0,
                    desc='Length of Compressor',
                    units='m')

    def solve_nonlinear(self, params, unknowns, resids):
        p_tunnel = params['p_tunnel']
        R = params['R']
        T_tunnel = params['T_tunnel']
        A_inlet = params['A_inlet']
        M_pod = params['A_inlet']
        comp_inletTemp = params['comp_inletTemp']
        comp_inletArea = params['comp_inletArea']
        gam = params['gam']
        comp_mach = params['comp_mach']
        h_out = params['h_out']
        h_in = params['h_in']
        h_stage = params['h_stage']

        #Calculating Length of Compressor.
        air_den_tunnel = p_tunnel / (R*T_tunnel)
        air_den_comp = ((air_den_tunnel)*(M_pod)*(A_inlet))/ (comp_mach*comp_inletArea)
        comp_r = np.sqrt(comp_inletArea / np.pi)
        rho_tunnel = p_tunnel/(R*T_tunnel)
        #m_dot = 159.
        m_dot = rho_tunnel*A_inlet*M_pod*np.sqrt(gam*R*T_tunnel)
        comp_inlet_flow_area = m_dot / ((air_den_comp)*(comp_mach)*np.sqrt(gam*9.81*R*comp_inletTemp))
        hub_tip_ratio = (np.sqrt(comp_r**2 - (comp_inlet_flow_area / 3.1416))) / comp_r
        no_stages = ((h_out - h_in)/ h_stage) + 1
        print(no_stages)
        unknowns['comp_len'] = 0.2 + (0.234 - 0.218*hub_tip_ratio)*(no_stages)*comp_r*2

if __name__ == "__main__":
    top = Problem()
    root = top.root = Group()

    root.add('CompressorLen', CompressorLen())

    top.setup()
    top.run()

    print('Comp_Len %f' % top['CompressorLen.comp_len'])
