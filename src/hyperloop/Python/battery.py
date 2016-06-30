import numpy as np
from openmdao.api import Component, Problem, Group


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

    Outputs
    -------
    n_cells : float
        total number battery cells (unitless)

    References
    -----
    .. [1] Gladin, Ali, Collins, "Conceptual Modeling of Electric and Hybrid-Electric Propulsion for UAS Applications"
       Georgia Tech, 2015

    .. [2] D. N. Mavris, "Subsonic Ultra Green Aircraft Research - Phase II," NASA Langley Research Center, 2014

    """

    # TODO rematch battery performance data to 18650 or similar Li-Ion battery instead of
    # outdated Ni-Mh battery

    def __init__(self):
        """Initializes a `Battery` object

        Sets up the given Params/Outputs of the OpenMDAO `Battery` component, initializes their shape, and
        sets them to their default values.
        """

        super(Battery, self).__init__()

        # setup inputs
        self.add_param('des_time',
                       val=1.0,
                       desc='time until design power point',
                       units='h')
        self.add_param('time_of_flight',
                       val=2.0,
                       desc='total mission time',
                       units='h')
        self.add_param('des_power', val=7, desc='design power', units='W')
        self.add_param('des_current', val=1, desc='design current', units='A')
        self.add_param('q_l',
                       val=0.1,
                       desc='discharge limit',
                       units='unitless')
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
                       val=6.8,
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

        # setup outputs
        self.add_output('n_cells',
                        val=1.0,
                        desc='total number of battery cells',
                        units='unitless')

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

        # check representation invariant
        self._check_rep(params, unknowns, resids)

        cap_discharge = self._calculate_total_discharge(
            params['time_of_flight'], params['des_current'])
        n_parallel = cap_discharge / (params['q_n'] * (1 - params['q_l']))
        single_bat_current = params['des_current'] / n_parallel
        single_bat_discharge = self._calculate_total_discharge(
            params['des_time'], params['des_current']) / n_parallel

        # calculate general battery performance curve paramaters

        # voltage drop over exponential zone
        # a = params['params['e_full']'] - params['e_exp']
        a = 0.144
        # discharge of single cell from full to end of exponential zone
        q_exp = self._calculate_total_discharge(
            params['t_exp'], params['des_current']) / n_parallel
        # time constant of the exponential zone
        # b = 3 / q_exp
        b = 2.3077
        # discharge over the nominal zone
        q_nom = self._calculate_total_discharge(
            params['t_nom'], params['des_current']) / n_parallel
        # polarization voltage
        # k = (params['params['e_full']'] - params['e_nom'] + a * (np.exp(-b * q_nom) - 1)) * (params['q_n'] - q_nom)
        k = 0.01875
        # no load constant voltage of battery
        # k = polarization voltage, params['r'] = resistance,
        # e_0 = params['params['e_full']'] + k + params['r'] * single_bat_current - a
        e_0 = 1.2848

        # general voltage performance curve
        v_batt = e_0 - k * (params['q_n'] / (
            params['q_n'] - single_bat_discharge)) + a * np.exp(
                -b * single_bat_discharge) - params['r'] * single_bat_current

        # single battery power at design power point
        p_bat = v_batt * single_bat_current

        # total number of battery cells
        n_cells = params['des_power'] / p_bat

        self.unknowns['n_cells'] = np.ceil(n_cells)

        # check representation invariant
        assert n_cells >= n_parallel
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
        assert params['q_l'] > 0
        assert params['des_time'] > 0
        assert params['time_of_flight'] > 0
        assert params['des_power'] > 0
        assert params['des_current'] > 0
        assert params['e_full'] > 0
        assert params['q_n'] > 0
        assert params['e_exp'] > 0
        assert params['e_nom'] > 0
        assert params['t_nom'] > 0
        assert params['t_exp'] > 0
        assert params['r'] > 0
        assert unknowns['n_cells'] > 0


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
