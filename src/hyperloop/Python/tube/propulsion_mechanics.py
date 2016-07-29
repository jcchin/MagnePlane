"""
Estimate power requirements for prouplsion sections
Many parameters are currently taken from hyperloop alpha
Can currently be used for LSM or LIM systems
"""

from __future__ import print_function
import numpy as np
from openmdao.api import IndepVarComp, Component, Group, Problem

class PropulsionMechanics(Component):
    """
    Notes
    ------
    Calculate power required to accelerate pod in one boosting section assuming linear acceleration of 1g

    Params
    ------
    p_tube : float
        Pressure of air in tube.  Default value is 100 Pa.  Value will come from vacuum component
    R : float
        Ideal gas constant. Default valut is 287 J/(m*K).
    T_ambient : float
        Tunnel ambient temperature. Default value is 298 K.
    g : float
        Gravitational acceleration. Default value is 9.81 m/s**2
    vf : float
        Top pod speed after boosting section. Default value is 335 m/s. Value will be taken from aero module
    vo : float
        Speed of pod when it enters boosting section. Default value is 324 m/s.
    m_pod : float
        total mass of pod. Default value is 3100 kg. Value will come from weight component
    eta : float
        Efficiency of propulsion system. Default value is .8. value will come from propulsion module.
    Cd : float
        Drag coefficient of pod.  Default value is .2. More accurate results will come from CFD
    S : float
        Reference area of the pod. Default value is 1.4 m**2. Value will be pulled from geometry module
    D_mag : float
        Drag force from magnetic levitation in N. Default value is 150 N.  Value will come from levitation analysis
    nozzle_thrust : float
        Thrust produced by pod compressed air. Default value 21473.92 N. Will pull value from flow_path.py
    ram_drag : float
        Drag produced by inlet ram pressure. Default value is 7237.6

    Returns
    -------
    pwr_req : float
        Computes power required by accelerating segment
    """

    def __init__(self):
        """Establish inputs to equation.  Values initialized as practical values for LSM motors
        Output: Power required"""

        super(PropulsionMechanics, self).__init__()

        self.add_param('p_tube',
                       val=100.0,
                       desc='Ambient Pressure',
                       units='Pa')
        self.add_param('R',
                       val=286.9,
                       desc='Ideal gas constant of air',
                       units='J/(kg * K)')
        self.add_param('T_ambient',
                       val=293.0,
                       desc='Ambient Temperature',
                       units='K')
        self.add_param('g', val=9.81, desc='Gavity', units='m/s**2')
        self.add_param('vf', val=335.0, desc='Top Speed', units='m/s')
        self.add_param('v0', val=324.0, desc='Entrance Speed', units='m/s')
        self.add_param('m_pod',
                       val=3100.0,
                       desc='mass of the pod without the magnets',
                       units='kg')
        self.add_param('eta', val=.8, desc='LSM efficiency')
        self.add_param('Cd', val=.2, desc='Aerodynamic drag coefficient')
        self.add_param('S', val=1.4, desc='Frontal Area', units='m**2')
        self.add_param('D_mag',
                       val=150.0,
                       units='N',
                       desc='Magnetic Drag')
        self.add_param('nozzle_thrust',
                       val=21473.92,
                       units='N',
                       desc='Thrust of Pod Nozzle')
        self.add_param('ram_drag', val = 7237.6, units = 'N', desc = 'Drag from inlet ram pressure')
        self.add_param('theta', val = 0.0, units = 'rad', desc = 'Pod pitch angle')

        self.add_output('D', val = 0.0, units = 'N', desc = 'total pod drag')
        self.add_output('pwr_req', val=0.0)  #Define power as output
        self.add_output('Fg_dP', val=0.0)  #Define Thrust per unit Power output
        self.add_output('m_dP', val=0.0)  #Define mass per unit power as output

    def solve_nonlinear(self, params, unknowns, resids):
        """Evaluate function Preq = (1/eta)*(mg*(1+sin(theta))*(vf-vo)+(1/6)*(Cd*rho*S*(vf^3 - vo^3))+D_mag*(vf-v0))
        Can be optimized in the future.  Friction and magnetic drag are neglected for now.
        """

        eta = params['eta']
        g = params['g']
        vf = params['vf']
        v0 = params['v0']
        Cd = params['Cd']
        S = params['S']
        m_pod = params['m_pod']

        #Calculate intermediate variables
        rho = params['p_tube'] / (params['R'] * params['T_ambient']
                                  )  #Calculate air density, rho = P/(RT)
        pod_thrust = params['nozzle_thrust'] - params['ram_drag']
        L = ((vf**2) - (v0**2)) / (2 * g)  #Calculate necessary track length

        #Evaluate equation
        unknowns['D'] = .5 * rho * (vf**2.0) * S * Cd
        unknowns['pwr_req'] = (1.0 / eta) * (
            (m_pod * g * (1 + np.sin(params['theta']))* (vf - v0)) + (1.0 / 6.0) * (Cd * rho * S * (
                (vf**3.0) - (v0**3.0))) + params['D_mag'] *
            (vf - v0) - pod_thrust * (vf - v0))
        unknowns['Fg_dP'] = (m_pod * g) / unknowns['pwr_req']

        unknowns['m_dP'] = m_pod / unknowns['pwr_req']

if __name__ == '__main__':

    #Set up problem to accept outside inputs for efficiency, eta, and magnet density, rho_pm
    root = Group()
    root.add('p', PropulsionMechanics())
    root.add('p1', IndepVarComp('eta', .8))

    root.connect('p1.eta', 'p.eta')

    #Set up problem
    top = Problem()
    top.root = root

    #Will not specify driver since only default driver is required at this time

    top.setup()

    top['p1.eta'] = .8

    top.run()

    print('\n')
    print('Power Required is %f W' % top['p.pwr_req'])
    print('Thrust per unit power = %f N/W' % top['p.Fg_dP'])
    print('Mass per unit power = %f kg/W' % top['p.m_dP'])
