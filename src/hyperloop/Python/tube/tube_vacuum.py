"""
Current calculation to determine total number of vacuum pumps needed and their respective cost per year.
The National average for Electricity runs $.13 cents per kilowatt hour.
"""
from openmdao.core.component import Component
from math import pi

import numpy as np

class Vacuum(Component):
    """
    Params
    ------
    pressure_initial : float
        initial Pressure before the pump down . Default value is 760.2.
    pressure_final : float
        Desired pressure within tube. Default value is 7.0.
    speed : float
        Pumping speed. Default value is 163333.3.
    tube_Area : float
        Area of the tube. Default value is 5.0.
    tube_length : float
        Length of the tube. Default value is 5000.0.
    pwr : float
        Motor rating. Default value is 18.5.
    electricity_price : float
        Cost of electricity per kilowatt hour. Default value is 0.13.
    time_down : float
        Desired pump down time. Default value is 300.0.
    gamma : float
        Operational percentage of the pump per day. Default value is 0.8.
    pump_weight : float
        Weight of one pump. Default value is 715.0.

    Returns
    -------
    number_pumps : float
        Number of pumps. Default value is 1.0.
    cost_annual : float
        Total cost of pumps. The cost of purchasing the pumps and running them per year in USD.
    weight_tot: float
        Total weight of the pumps throughout the track in kg.
    pwr_tot: float
        Total power of the pumps in kW.
    energy_tot: float
        Total energy consumed by the pumps in one day in kJ.
        
    References
    ----------
    [1] Laughlin, Robert B., Prof. "Energy Information Administration - Electricity Price." EIA.
    Stanford University, 30 Dec. 2008. Web. 24 June 2016.
    <http://large.stanford.edu/publications/power/references/voltprice/>
    Umrath, Walter, Dr. Fundamentals of Vacuum Technology. N.p.: Oerlikon Leybold Vacuum, n.d. Print.
    """

    def __init__(self):
        super(Vacuum, self).__init__()

        # Inputs
        self.add_param('pressure_initial',
                       760.2,
                       desc='initial Pressure before the pump down',
                       units='torr')
        self.add_param('pressure_final',
                       7.0,
                       desc='desired pressure within the tube',
                       units='torr')
        self.add_param('speed', 163333.3, desc='Pumping speed', units='L/min')
        self.add_param('tube_area', 78.5, desc='Area of the tube', units='ft**2')
        self.add_param('tube_length', 5000.0, desc='Length of the tube', units='ft')
        self.add_param('pwr', 18.5, desc='motor rating', units='kW')
        self.add_param('electricity_price',
                       0.13,
                       desc='cost of electricity per kilowatt hour',
                       units='USD/(kW*h)')
        self.add_param('time_down',
                       300.0,
                       desc='desired pump down time',
                       units='min')
        self.add_param('gamma',
                       .8,
                       desc='operational percentage of the pump per day')
        self.add_param('pump_weight',
                       715.0,
                       desc='weight of one pump',
                       units='kg')
        # self.add_param('opt',100000.0, desc= 'operating time of the motor', units='mins')

        # Outputs

        self.add_output('number_pumps', 1.0, desc='number of pumps')
        # self.add_output('volft',
        #                 2.0,
        #                 desc='volume of the tube in feet cubed',
        #                 units='ft**3')
        # self.add_output('vol',
        #                 2.0,
        #                 desc='volume of the tube in Liters',
        #                 units='L')
        self.add_output('energy_tot',
                        1.0,
                        desc='total energy required to run the pumps',
                        units='kJ')
        self.add_output('cost_annual',
                        1.0,
                        desc='total cost to run the vacuums per year',
                        units='USD/yr')
        self.add_output('weight_tot',
                        1.0,
                        desc='total weight of the pumps',
                        units='kg')
        self.add_output('pwr_tot',
                        1.0,
                        desc='total pwr of the pumps',
                        units='kW')

    def solve_nonlinear(self, params, unknowns, resids):

        volft = params['tube_area'] * params['tube_length']  # Volume of the tube in cubic feet

        vol = volft * 28.3168  # Volume of the tube in liters

        # Number of pumps needed
        n = (vol / params['speed']) * np.log(
            params['pressure_initial'] / params['pressure_final']) * 2.0 * (1.0 / params['time_down'])

        # Energy Consumption of a Single Pump in one day
        energy_tot = params['pwr'] * n * (params['gamma'] * 86400.0)

        # Cost to Run the Vacuum for One Year
        unknowns['cost_annual'] = energy_tot * 365.0 * params['electricity_price'] / (
            1000.0 * 60.0 * 60.0 * (1.0 / 1000.0))

        # Total weight of all of the pumps.
        unknowns['weight_tot'] = params['pump_weight'] * n

        # unknowns['vol'] = vol
        # unknowns['volft'] = volft
        unknowns['energy_tot'] = energy_tot
        unknowns['number_pumps'] = n

        unknowns['pwr_tot'] = params['pwr'] * n

if __name__ == '__main__':

    from openmdao.core.problem import Problem
    from openmdao.core.group import Group

    p = Problem(root=Group())
    p.root.add('comp', Vacuum())
    p.setup()
    # p.root.list_connections()
    p.run()

    print('Total weight of the pumps (kg): %f' %
          (p['comp.weight_tot']))  # Print total weight
    print('Total cost($): %f' % (p['comp.cost_annual']))  # Print total cost
    print('Total number of pumps (#): %f' %
          (p['comp.number_pumps']))  # Print total number of required pumps
    print('Total power (kW): %f' %
          (p['comp.pwr_tot']))  # Print total power
    print('Total energy per day (kJ): %f' %
          (p['comp.energy_tot']))  # Print total energy consumed per day