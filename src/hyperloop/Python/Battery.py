"""
Calculates parameters like Nparallel(number of cells in parallel), Nseries(Number of cells in series), Ncells(total no of cells) and C_Max(max rating)
The calculated parameters are then use to estimate battery weight in BatteryWeight.py
"""

import math, scipy, matplotlib
import numpy as np
from openmdao.core.component import Component
from openmdao.api import IndepVarComp, Component, Problem, Group, ScipyOptimizer, ExecComp, SqliteRecorder

class Battery(Component):
    """
    Params
    ------
    desTime : float
        time until design power point (h)
    timeOfFlight : float
        total mission time (h)
    desPower : float
        design power (W)
    desCurrent : float
        design current (A)
    q_l : float
        discharge limit (unitless)
    E_full : float
        fully charged voltage (V)
    E_nom : float
        voltage at end of nominal voltage (V)
    E_exp : float
        voltage at end of exponential zone (V)
    Q_N : float
        single cell capacity (A*h)
    t_exp : float
        time to reach exponential zone (h)
    t_nom : float
        time to reach nominal zone (h)
    R : float
        resistance of individual battery cell (Ohms)


    Outputs
    -------
    N_cells : integer
        total number battery cells (unitless)

    References
    -----
    .. [1] Gladin, Ali, Collins, "Conceptual Modeling of Electric and Hybrid-Electric Propulsion for UAS Applications"
       Georgia Tech, 2015

    .. [2] D. N. Mavris, "Subsonic Ultra Green Aircraft Research - Phase II," NASA Langley Research Center, 2014

    """

    def __init__(self):

        super(Battery, self).__init__()

        self.add_param('desTime', val=1.0, desc='time until design power point', units='h')
        self.add_param('timeOfFlight', val=2.0, desc='total mission time', units='h')
        self.add_param('desPower', val=7.0, desc='design power', units='W')
        self.add_param('desCurrent', val=1.0, desc='design current', units='A')
        self.add_param('q_l', val=0.1, desc='discharge limit', units='unitless')
        self.add_param('E_full', val=1.0, desc='fully charged voltage', units='V')
        self.add_param('E_nom', val=1.2, desc='voltage at  end of nominal zone', units='V')
        self.add_param('E_exp', val=1.0, desc='voltage at end of exponential zone', units='V')
        self.add_param('Q_N', val=6.8, desc='Single cell capacity', units='A*h')
        self.add_param('t_exp', val=1.0, desc='time to reach exponential zone', units='h')
        self.add_param('t_nom', val=1.0, desc='time to reach nominal zone', units='h')
        self.add_param('R', val=1.0, desc='battery resistance', units='Ohms')

        self.add_output('N_cells', val=1, desc='total number of battery cells', units='unitless')

    def solve_nonlinear(self, params, unknowns, resids):
        E_full = params['E_full']
        Q_N = params['Q_N']
        q_l = params['q_l']
        E_exp = params['E_exp']
        E_nom = params['E_nom']
        desCurrent = params['desCurrent']
        t_nom = params['t_nom']
        t_exp = params['t_exp']
        # R = params['R']
        R = 0.0046
        desPower = params['desPower']
        desTime = params['desTime']
        timeOfFlight = params['timeOfFlight']
        
        capDischarge = self.calculate_total_discharge(timeOfFlight, desCurrent)
        N_parallel = capDischarge / (Q_N * (1 - q_l))
        singleBatCurrent = desCurrent / N_parallel
        singleBatDischarge = self.calculate_total_discharge(desTime, desCurrent) / N_parallel

        # calculate general battery performance curve paramaters
        
        # voltage drop over exponential zone
        A = E_full - E_exp
        # A = 0.144

        # discharge of single cell from full to end of exponential zone
        Q_exp = self.calculate_total_discharge(t_exp, desCurrent) / N_parallel
        
        # time constant of the exponential zone
        B = 3 / Q_exp
        # B = 2.3077

        # polarization voltage
        K = (E_full - E_nom + A * (np.exp(-B * Q_nom) - 1)) * (Q_N - Q_nom)
        # K=0.01875

        # discharge over the nominal zone
        Q_nom = self.calculate_total_discharge(t_nom, desCurrent) / N_parallel

        # no load constant voltage of battery
        # K = polarization voltage, R = resistance,
        E_0 = E_full + K + R * singleBatCurrent - A
        # E_0 = 1.2848


        # general voltage performance curve
        V_batt = E_0 - K * (Q_N / (Q_N - singleBatDischarge)) + A * np.exp(-B * singleBatDischarge) - R * singleBatCurrent

        # single battery power at minimum voltage point
        P_bat = V_batt * singleBatCurrent

        # total number of battery cells
        N_cells = desPower / P_bat

        self.unknowns['N_cells'] = np.ceil(N_cells)

    def calculate_total_discharge(self, time, current):
        # constant current profile
        return time * current


if __name__ == '__main__':
    # set up problem
    root = Group()
    p = Problem(root)
    p.root.add('comp', Battery())
    p.setup()
    p.root.list_connections()
    p.run()

    # print following properties

    print ('Ncells(cells) : %f' % p['comp.N_cells'])

