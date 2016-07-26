import numpy as np
import scipy.interpolate, scipy.integrate
from openmdao.api import Component, Problem, Group
import os


class Battery(Component):
    """The `Battery` class represents a battery component in an OpenMDAO model. 
    
    A `Battery` models battery performance by finding a general battery voltage 
    performance curve ([1]_, [2]_) based on standard cell properties and can be used to
    determine the number of cells and cell configuration needed to meet specification.

    Params
    ------
    des_time : float
        time until design power point (h)
    time_of_flight : float
        total mission time (h)
    battery_cross_section_area : float
        cross_sectional area of battery used to compute length (cm^2)
    des_power : float
        design power (W)
    des_current : float
        design current (A)
    q_l : float
        discharge limit (unitless)
    e_full : float
        fully charged voltage (V)
    e_nom : float
        voltage at end of nominal voltage (V)
    e_exp : float
        voltage at end of exponential zone (V)
    q_n : float
        single cell capacity (A*h)
    t_exp : float
        time to reach exponential zone (h)
    t_nom : float
        time to reach nominal zone (h)
    r : float
        resistance of individual battery cell (Ohms)
    cell_mass : float
        mass of a single cell (g)
    cell_height : float
        height of a single cylindrical cell (mm)
    cell_diameter : float
        diamter of a single cylindrical cell (mm)

    Outputs
    -------
    n_cells : float
        total number battery cells (unitless)
    battery_length : float
        length of battery (cm)
    output_voltage : float
        output voltage of battery configuration (V)
    battery_mass : float
        total mass of cells in battery configuration (kg)
    battery_volume : float
        total volume of cells in battery configuration (cm^3)
    battery_cost : float
        total cost of battery cells in (USD)

    Notes
    -----

    Battery cost based on purchase of 1000 bateries from ecia ([3]_) price listing

    References
    ----------

    .. [1] Gladin, Ali, Collins, "Conceptual Modeling of Electric and Hybrid-Electric Propulsion for UAS Applications"
       Georgia Tech, 2015

    .. [2] D. N. Mavris, "Subsonic Ultra Green Aircraft Research - Phase II," NASA Langley Research Center, 2014

    .. [3] eciaauthorized.com/search/HHR650D

    """

    # TODO rematch battery performance data to 18650 or similar Li-Ion battery instead of
    # outdated Ni-Mh battery
    # TODO account for additional battery containment hardware
    # TODO fix voltage to certain range?

    def __init__(self):
        """Initializes a `Battery` object

        Sets up the given Params/Outputs of the OpenMDAO `Battery` component, initializes their shape, and
        sets them to their default values.
        """

        super(Battery, self).__init__()

        # setup mission characteristics
        self.add_param('des_time',
                       val=1.0,
                       desc='time until design power point',
                       units='h')
        self.add_param('time_of_flight',
                       val=1.0,
                       desc='total mission time',
                       units='h')
        self.add_param('des_power', val=7.0, desc='design power', units='W')
        self.add_param('des_current',
                       val=1.0,
                       desc='design current',
                       units='A')
        self.add_param('q_l',
                       val=0.1,
                       desc='discharge limit',
                       units='unitless')
        # setup battery characteristics
        self.add_param('e_full',
                       val=1.4,
                       desc='fully charged voltage',
                       units='V')
        self.add_param('e_nom',
                       val=1.2,
                       desc='voltage at  end of nominal zone',
                       units='V')
        self.add_param('e_exp',
                       val=1.27,
                       desc='voltage at end of exponential zone',
                       units='V')
        self.add_param('q_n',
                       val=3.5,
                       desc='Single cell capacity',
                       units='A*h')
        self.add_param('t_exp',
                       val=1.0,
                       desc='time to reach exponential zone',
                       units='h')
        self.add_param('t_nom',
                       val=4.3,
                       desc='time to reach nominal zone',
                       units='h')
        self.add_param('r',
                       val=0.0046,
                       desc='battery resistance',
                       units='Ohms')
        self.add_param('battery_cross_section_area',
                       15000.0,
                       desc='cross_sectional area of battery used to compute length',
                       units='cm**2')
        self.add_param('cell_mass',
                       val=170.0,
                       desc='mass of a single cell',
                       units='g')
        self.add_param('cell_height',
                       val=61.0,
                       desc='height a single cylindrical cell',
                       units='mm')
        self.add_param('cell_diameter',
                       val=33.0,
                       desc='diamter of a single  cylindrical cell',
                       units='mm')

        # setup outputs
        self.add_output('n_cells',
                        val=1.0,
                        desc='total number of battery cells',
                        units='unitless')
        self.add_output('output_voltage',
                        val=1000.0,
                        desc='output voltage of battery configuration',
                        units='V')
        self.add_output('battery_mass',
                        val=1.0,
                        desc='total mass of cells in battery configuration',
                        units='kg')
        self.add_output('battery_volume',
                        val=1.0,
                        desc='total volume of cells in battery configuration',
                        units='cm**3')
        self.add_output('battery_cost',
                        val=1.0,
                        desc='total materials cost of battery configuration',
                        units='$')
        self.add_output('battery_length',
                        val=1.0,
                        desc='length of battery',
                        units='cm')

    def solve_nonlinear(self, params, unknowns, resids):
        """Runs the `Battery` component and sets its respective outputs to their calculated results

        Args
        ----------
        params : `VecWrapper`
            `VecWrapper` containing parameters

        unknowns : `VecWrapper`
            `VecWrapper` containing outputs and states

        resids : `VecWrapper`
            `VecWrapper` containing residuals

        """
        print(params['des_power'])
        print(params['des_current'])

        # check representation invariant
        self._check_rep(params, unknowns, resids)

        # FIXME using ceiling for cell calculations, despite advice against
        # need to change to proper way of constraining to integer values as
        # battery falls apart for n_paralell < 0 (i.e. if n_paralell = 0.01 and
        # not 1.0, then we get absurd output voltage levels

        cap_discharge = self._calculate_total_discharge(
            params['time_of_flight'], params['des_current'])
        # print((params['q_n'] * (1 - params['q_l'])))
        # print cap_discharge
        n_parallel = cap_discharge / (params['q_n'] * (1 - params['q_l']))
        # print('usaable cap %f ' % (params['q_n'] * (1 - params['q_l'])))
        # print('cap dis %f' % cap_discharge)
        # print(n_parallel)
        single_bat_current = params['des_current'] / n_parallel
        # print('single bat cur %f' % single_bat_current)
        # single_bat_current = 7
        # single_bat_current = 7
        # n_parallel = -params['des_current'] / single_bat_current
        # single_bat_current = 13
        single_bat_discharge = self._calculate_total_discharge(
            params['des_time'], single_bat_current)

        # # calculate general battery performance curve paramaters
        # # TODO calculate performance curve parameters from Panasonic 18650
        # # battery, currently using parameters from SUGAR paper
        #
        # # voltage drop over exponential zone
        # # a = params['params['e_full']'] - params['e_exp']
        # a = 0.144
        # # discharge of single cell from full to end of exponential zone
        # q_exp = self._calculate_total_discharge(
        #     params['t_exp'], params['des_current']) / n_parallel
        #
        # # time constant of the exponential zone
        # # b = 3 / q_exp
        # b = 2.3077

        # discharge over the nominal zone
        # q_nom = self._calculate_total_discharge(
        #     params['t_nom'], params['des_current']) / n_parallel

        # # polarization voltage
        # # k = (params['params['e_full']'] - params['e_nom'] + a * (np.exp(-b * q_nom) - 1)) * (params['q_n'] - q_nom)
        # k = 0.01875
        #
        # # no load constant voltage of battery
        # # k = polarization voltage, params['r'] = resistance,
        # # e_0 = params['params['e_full']'] + k + params['r'] * single_bat_current - a
        # e_0 = 1.2848
        #
        # # general voltage performance curve
        # v_batt = e_0 - k * (params['q_n'] / (
        #     params['q_n'] - single_bat_discharge)) + a * np.exp(
        #         -b * single_bat_discharge) - params['r'] * single_bat_current

        # single battery power at design power point

        dir = os.path.dirname(__file__)
        filename = os.path.join(dir, '18650.csv')
        data = np.loadtxt(filename, dtype='float', delimiter=',').transpose()

        func = scipy.interpolate.UnivariateSpline(data[0], data[1])
        # print single_bat_discharge
        v_batt = func(single_bat_discharge * 1000)

        p_bat = v_batt * single_bat_current
        # print(v_batt)
        # print('p_bat %f' % p_bat)

        energy_cap = scipy.integrate.quad(func, 0, single_bat_discharge * 1000)[0] / 1000
        print('energy cap %f' % energy_cap)

        # total number of battery cells
        n_cells = params['des_power'] / p_bat
        print('n_cells %f' % n_cells)
        n_cells = np.ceil(n_cells)
        n_parallel = np.ceil(n_parallel)
        n_series = np.ceil(n_cells / n_parallel)

        # assert n_cells >= n_parallel

        unknowns['n_cells'] = n_cells

        # calculate volume of cells accounting for hexagonal packing efficiency of 0.9069 and convert from mm^3 to cm^3
        # unknowns['battery_volume'] =(n_cells * (
        #     params['cell_height'] * np.pi * np.power(params['cell_diameter'] /
        #                                              2, 2)) / 0.9069 / 1000) / 3.7

        # calculate mass of cells and convert to kg
        # including rough approx of li-ion density
        # TODO dev real model
        # unknowns['battery_mass'] = params['cell_mass'] * n_cells / 1000 / 5
        unknowns['battery_mass'] = energy_cap * n_cells / 265
        unknowns['battery_volume'] = energy_cap * n_cells / 730 * 1000 / 0.9069

        # calculate output voltage of battery in the nominal zone
        unknowns['output_voltage'] = n_series * params['e_nom']

        unknowns['battery_cost'] = n_cells * 12.95

        unknowns['battery_length'] = unknowns['battery_volume'] / params['battery_cross_section_area']

        # check representation invariant
        self._check_rep(params, unknowns, resids)

    def _calculate_total_discharge(self, time, current):
        """Calculates the total discharge over a given load profile

        Integrates the load profile from t = 0 to t = time

        Args
        ----------
        time : float
            the upper bound of the integration
        current : float
            the constant load profile

        Returns
        -------
        float
            the total discharge over the load profile

        """
        return time * current

    def _check_rep(self, params, unknowns, resids):
        """Checks that the representation invariant of the `Battery` class holds

        Args
        ----------
        params : `VecWrapper`
            `VecWrapper` containing parameters

        unknowns : `VecWrapper`
            `VecWrapper` containing outputs and states

        resids : `VecWrapper`
            `VecWrapper` containing residuals
        """
        # assert params['q_l'] > 0
        # assert params['des_time'] > 0
        # assert params['time_of_flight'] > 0
        # assert params['des_power'] > 0
        # assert params['des_current'] > 0
        # assert params['e_full'] > 0
        # assert params['q_n'] > 0
        # assert params['e_exp'] > 0
        # assert params['e_nom'] > 0
        # assert params['t_nom'] > 0
        # assert params['t_exp'] > 0
        # assert params['r'] > 0
        # assert unknowns['n_cells'] > 0


if __name__ == '__main__':
    # set up problem
    root = Group()
    p = Problem(root)
    p.root.add('comp', Battery())
    p.setup()
    p.root.list_connections()
    p.run()

    # print following properties

    print('Ncells(cells) : %f' % p['comp.n_cells'])
    print('mass: %f' % p['comp.battery_mass'])
    print('volume: %f' % p['comp.battery_volume'])
    print('voltage: %f' % p['comp.output_voltage'])
    print('cost : %f' % p['comp.battery_cost'])
    print(p['comp.battery_length'])
