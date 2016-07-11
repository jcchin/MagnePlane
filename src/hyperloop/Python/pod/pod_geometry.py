from __future__ import print_function

import numpy as np
from openmdao.core.component import Component
from openmdao.api import IndepVarComp, Component, Problem, Group
class PodGeometry(Component):

    """
    Notes
    ------

    Computes to corss sectional area, length, and planform area of the pod based on the sizes of internal components
    and the necessary duct area within the po based on compressor peformance.  Assumes isentropic compression and a 
    compressor exit mach number of .3. 

    Params
    ------
    Ratio of specific heats : float
        Ratio of specific heats. Default value is 1.4
    Ideal Gas Constant : float
        Ideal gas constant. Default valut is 287 J/(m*K).
    Blockage factor : float
        ratio of diffused area to pod area. Default value is .9. Value will be taken from pod structure module
    Compressor pressure ratio : float
        Pressure ratio across compressor inlet and outlet.  Default value is 12.5.  Value will be taken from NPSS
    Tube Pressure : float
        Pressure of air in tube.  Default value is 850 Pa.  Value will come from vacuum component
    Ambient Temperature : float
        Tunnel ambient temperature. Default value is 298 K.
    Duct Mach number : float
        Mach number of flow in the duct after exiting the compressor. Default value is .3
    Diffuser Mach number : float
        Maximum Mach number allowed at compressor inlet. Default value is .6
    pod mach : float
        pod Mach number. Default value is .8. value will be set by user
    Pay load area : float
        Cross sectional area of passenger compartment. Default value is 1.4 m**2
    L_comp : float
        length of the compressor. Default value is 1.0 m.
    L_bat : float
        length of battery. Default value is 1.0 m.
    L_inverter : float
        length of inverter. Default value is 1.0 m.
    L_trans : float
        length of transformer. Default value is 1.0 m
    L_p : float
        length of passenger compartment. Default value is 11.2 m
    L_conv : float
        length of the converging section of the nozzle. Default value is .3 m
    L_div : float
        length of the diverging section of the nozzle. Default value is 1.5 m
    Diffuser diameter : float
        Diameter of compressor inlet. Default value is 1.28 m.

    Returns
    -------
    Pod Area : float
        Cross sectional area of pod.
    Pod diameter : float
        Diameter of pod
    Planform area : float
        Planform area of the pod
    Pod length: float
        Length of Pod
    """

    def __init__(self):
        super(PodGeometry, self).__init__()

        self.add_param('L_comp', val=1.0, desc='Length of Compressor', units='m')
        self.add_param('L_bat', val=1.0, desc='Length of Battery', units='m')
        self.add_param('L_motor', val=1.0, desc='Length of Motor', units='m')
        self.add_param('L_inverter', val=1.0, desc='Length of Inverter', units='m')
        self.add_param('L_trans', val=1.0, desc='Length of Transformer', units='m')
        self.add_param('L_p', val=11.2, desc='Payload Length', units='m')
        self.add_param('L_conv', val=.3, desc='Converging Lenth', units='m')
        self.add_param('L_div', val=1.5, desc='Diverging Length', units='m')
        self.add_param('D_dif', val=1.28, desc='Compressor Inlet Radius', units='m')
        self.add_param('BF', val=.9, desc='Blockage Factor', units='unitless')
        self.add_param('prc', val=12.5, desc='Pressure ratio across compressor', units='unitless')
        self.add_param('A_inlet', val=1.1, desc='Inlet area', units='m**2')
        self.add_param('gam', val=1.4, desc='Ratio of specific heats', units='unitless')
        self.add_param('p_tunnel', val=850.0, desc='tunnel pressure', units='Pa')
        self.add_param('R', val=287.0, desc='Ideal gas constant', units='J/(kg*K)')
        self.add_param('T_tunnel', val=298.0, desc='Tunnel temperature', units='K')
        self.add_param('beta', val=.1, desc='Duct blockage coefficient', units='unitless')
        self.add_param('M_dif', val=.6, desc='Mach number at compressor inlet', units='unitless')
        self.add_param('M_duct', val=.3, desc='Mach number of flow exitting compressor', units='unitless')
        self.add_param('A_payload', val=1.4, desc='Cross sectional area of passenger compartment')

        self.add_param('M_pod', val=.8, desc='Pod Mach Number', units='unitless')

        self.add_output('A_pod', val = 0.0, desc = 'Cross sectional area of pod', units = 'm**2')
        self.add_output('D_pod', val=0.0, desc='Pod diametes', units='m')
        self.add_output('S', val=0.0, desc='Planform area of pod', units='m**2')
        self.add_output('L_pod', val=0.0, desc='Length of pod', units='m')
    
    def solve_nonlinear(self, p, u, r):

        A_inlet = p['A_inlet']
        R = p['R']
        T_tunnel = p['T_tunnel']
        D_dif = p['D_dif']
        gam = p['gam']
        prc = p['prc']
        M_pod = p['M_pod']

        D_inlet = np.sqrt((4*A_inlet)/np.pi)                #Calculate inlet diameter from area
        c = ((D_dif/2)-(D_inlet/2))/.0524                   #Calculate length of inlet assuming conical frustrum with 3 degree half angle
        rho_tunnel = p['p_tunnel']/(R*T_tunnel)             #Calculate air density in tunnel from ideal gas law

        T_ratio_dif = (1+((gam-1.0)/2.0)*(M_pod**2))/(1+((gam-1.0)/2.0)*(p['M_dif']**2))        #Calculate temperature ratio across diffuser from isentropic relations
        p_ratio_dif = T_ratio_dif**(gam/(gam-1.0))                                              #Calculate pressure ratio across diffuser from isentropic relations
        rho_ratio_dif = p_ratio_dif**(1.0/gam)                                                  #Calculate density ratio across inlet from isentropic relations
        T_dif = T_ratio_dif*T_tunnel
        p_dif = p_ratio_dif*p['p_tunnel']
        rho_dif = rho_tunnel*rho_ratio_dif

        T_ratio_comp = prc**((gam-1.0)/gam)                 #Calculate Temperature ratio across compressor assuming isentropic compression
        rho_ratio_comp = prc**(1.0/gam)                     #Calculate density ratio across compressor assuming isentropic compression

        T_prime = T_dif*T_ratio_comp
        rho_prime = rho_dif*rho_ratio_comp                  #Calculate desnsity of flow exiting compressor
        U_prime = p['M_duct']*np.sqrt(gam*R*T_prime)        #Calculate sped of air exiting compressor

        m_dot = rho_tunnel*A_inlet*M_pod*np.sqrt(gam*R*T_tunnel)        #Calculate mass flow into the diffuser
        A_duct = m_dot/(rho_prime*U_prime)                              #Calculate duct area using conservation of mass
        A_pod = (A_duct + (1+p['beta'])*p['A_payload'])/p['BF']         #Calculate cross sectional area of the pod
        D_pod = np.sqrt((4*A_pod)/np.pi)                                #Calculate pod diameter

        L_pod = c + p['L_comp'] + p['L_bat'] + p['L_motor'] + p['L_inverter'] + p['L_trans'] + p['L_p'] + p['L_conv'] + p['L_div']
        S = D_pod*L_pod

        u['A_pod'] = A_pod
        u['D_pod'] = D_pod
        u['L_pod'] = L_pod
        u['S'] = S

if __name__ == '__main__':
    top = Problem()
    root = top.root = Group()

    params = (
            ('BF', .9, {'units' : 'unitless'}),
            ('prc', 12.5, {'units' : 'unitless'}),
            ('A_inlet', 1.1, {'units' : 'm**2'}),
            ('p_tunnel', 850.0, {'units' : 'Pa'}),
            ('M_pod', .8, {'units' : 'unitless'}),
            ('D_dif', 1.28, {'units' : 'm'})
        )

    root.add('input_vars', IndepVarComp(params), promotes = ['BF', 'prc', 'A_inlet', 'p_tunnel', 'M_pod', 'D_dif'])
    root.add('p', PodGeometry(), promotes = ['BF', 'prc', 'A_inlet', 'p_tunnel', 'M_pod', 'D_dif'])

    top.setup()
    top.run()

    print('\n')
    print('Pod cross section = %f m^2' % top['p.A_pod'])
    print('Pod Diameter = %f m' % top['p.D_pod'])
    print('Pod Length = %f m' % top['p.L_pod'])
    print('Pod planform area = %f m^2' % top['p.S'])
