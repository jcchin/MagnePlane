"""
Current calculation to determine total number of pumps needed and their respective cost per year.
The National average for Electricity runs $.13 cents per kilowatt hour.
"""
from openmdao.core.component import Component
from math import pi

import numpy as np


class Vacuum(Component):
    """
    Params
    ------
    pinit : float
        Operating Pressure of system evacuated. Default value is 760.2.
    pfinal : float
        Desired pressure within tube. Default value is 7.0.
    speed : float
        Pumping speed. Default value is 163333.3.
    rad : float
        Radius of the tube. Default value is 5.0.
    len : float
        Length of the tube. Default value is 5000.0.
    pwr : float
        Motor rating. Default value is 18.5.
    eprice : float
        Cost of electricity per kilowatt hour. Default value is 0.13.
    tdown : float
        Desired pump down time. Default value is 300.0.
    gamma : float
        Operational percentage of the pump per day. Default value is 0.8.
    pumpweight : float
        Weight of one pump. Default value is 715.0.

    Returns
    -------
    totpwr : float
        Total power consumption. Default value is 0.0.
    n : float
        Number of pumps. Default value is 1.0.
    volft : float
        Volume of tube in feet cubed. Default value is 0.0.
    vol : float
        Volume of tube in liters. Default value is 0.0.
    etot : float
        Total energy required to run the pumps. Default value is 0.0.
    cost : float
        Total cost of pumps. The cost of purchasing the pumps and running them per year in USD.
    weighttot: float
        Total weight of the pumps throughout the track in kg.

    Notes
    -----
    [1] Laughlin, Robert B., Prof. "Energy Information Administration - Electricity Price." EIA.
    Stanford University, 30 Dec. 2008. Web. 24 June 2016.
    <http://large.stanford.edu/publications/power/references/voltprice/>
    Umrath, Walter, Dr. Fundamentals of Vacuum Technology. N.p.: Oerlikon Leybold Vacuum, n.d. Print.
    """

    def __init__(self):
        super(Vacuum, self).__init__()

        # Inputs
        self.add_param('pinit',
                       760.2,
                       desc='operating pressure of system evacuated',
                       units='torr')
        self.add_param('pfinal',
                       7.0,
                       desc='desired pressure within the tube',
                       units='torr')
        self.add_param('speed', 163333.3, desc='Pumping speed', units='L/min')
        self.add_param('rad', 5.0, desc='radius of the tube', units='ft')
        self.add_param('len', 5000.0, desc='length of the tube', units='ft')
        self.add_param('pwr', 18.5, desc='motor rating', units='(W*1000)')
        self.add_param('eprice',
                       0.13,
                       desc='cost of electricity per kilowatt hour',
                       units='USD/(W*1000*h)')
        self.add_param('tdown',
                       300.0,
                       desc='desired pump down time',
                       units='min')
        self.add_param('gamma',
                       .8,
                       desc='operational percentage of the pump per day')
        self.add_param('pumpweight',
                       715.0,
                       desc='weight of one pump',
                       units='kg')
        # self.add_param('opt',100000.0, desc= 'operating time of the motor', units='mins')

        # Outputs
        self.add_output('totpwr',
                        1.0,
                        desc='total power consumption',
                        units='kW')
        self.add_output('n', 1.0, desc='number of pumps')
        self.add_output('volft',
                        2.0,
                        desc='volume of the tube in feet cubed',
                        units='ft**3')
        self.add_output('vol',
                        2.0,
                        desc='volume of the tube in Liters',
                        units='L')
        self.add_output('etot',
                        1.0,
                        desc='total energy required to run the pumps',
                        units='J*1000')
        self.add_output('cost',
                        1.0,
                        desc='total cost to run the vacuums per year',
                        units='USD/yr')
        self.add_output('weighttot',
                        1.0,
                        desc='total weight of the pumps',
                        units='kg')

    def solve_nonlinear(self, params, unknowns, resids):

        volft = pi * (params['rad']**
                      2.) * params['len']  # Volume of the tube in cubic feet

        vol = volft * 28.3168  # Volume of the tube in liters

        # Number of pumps needed
        n = (vol / params['speed']) * np.log(
            params['pinit'] / params['pfinal']) * 2.0 * (1.0 / params['tdown'])

        # Energy Consumption of a Single Pump in one day
        etot = params['pwr'] * n * (params['gamma'] * 86400.0)

        # Cost to Run the Vacuum for One Year
        unknowns['cost'] = etot * 365.0 * params['eprice'] / (
            1000.0 * 60.0 * 60.0 * (1.0 / 1000.0))

        # Total weight of all of the pumps.
        unknowns['weighttot'] = params['pumpweight'] * n

        unknowns['vol'] = vol
        unknowns['volft'] = volft
        unknowns['etot'] = etot
        unknowns['n'] = n


if __name__ == '__main__':

    from openmdao.core.problem import Problem
    from openmdao.core.group import Group

    p = Problem(root=Group())
    p.root.add('comp', Vacuum())
    p.setup()
    # p.root.list_connections()
    p.run()

    print('Total weight of the pumps (kg): %f' %
          (p['comp.weighttot']))  # Print total weight
    print('Total cost($): %f' % (p['comp.cost']))  # Print total cost
    print('Total number of pumps (#): %f' %
          (p['comp.n']))  # Print total number of required pumps
