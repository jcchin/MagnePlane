from openmdao.api import IndepVarComp,Component,Group, Problem
import numpy as np

class TubePower(Component):
    """
    Computes the total power requirement for all tube components:
    Vacuum, TubeAndPylon, PropulsionMechanics

    Params
    ------
    vac_power : float
        Power requirement to run vacuum pumps (kW)
    vac_energy : float
        Energy requirement to run vacuum pumps for 1 day (kJ)
    prop_power : float
        Power required to accelerate pod to 1G once (W)
    num_thrust : float
        Number of propulsion thrusts required for trip (unitless)
    time_thrust : float
        Time required to accelerate pod to 1G (s)
    tube_temp : float
        Tube temperature (K)
    elec_price : float
        Cost of electricity per kiloWatt-hour (USD/(kW*h))

    Outputs
    -------
    tot_power : float
        Total power requirement for tube components (kW)
    tot_energy : float
        Total energy requirement for tube components (kJ)
    cost_pwr : float
        Cost for tube power requirements (USD)

    Notes
    -----
    The national average for electricity runs $.13 cents per kilowatt hour.
    Power requirement to cool the tube is not currently calculated in this component. Params to calculate
    that power in the future are commented out for the meantime.

    References
    ----------
    [1] Laughlin, Robert B., Prof. "Energy Information Administration - Electricity Price." EIA.
    Stanford University, 30 Dec. 2008. Web. 24 June 2016.
    <http://large.stanford.edu/publications/power/references/voltprice/>
    Umrath, Walter, Dr. Fundamentals of Vacuum Technology. N.p.: Oerlikon Leybold Vacuum, n.d. Print.


    TODO: add in calculations for refrigeration power requirement?

    """

    def __init__(self):
        super(TubePower, self).__init__()
        self.add_param('vac_power',
                       val=40.0,
                       desc='Vacuum power requirement for entire length',
                       units='kW')
        self.add_param('vac_energy_day',
                       val=40.0*24.0*60.0*60.0,
                       desc='Energy requirement for vacuums to run 1 day',
                       units='kJ')
        self.add_param('prop_power',
                       val=300000.0,
                       desc='Power required to accelerate pod once',
                       units='W')
        self.add_param('num_thrust',
                       val=5.0,
                       desc='Number of thrusts required for one trip',
                       units='unitless')
        self.add_param('time_thrust',
                       val=1.5,
                       desc='Time required to accelerate pod to 1G',
                       units='s')
        self.add_param('tube_temp',val=320.0,desc='Tube temperature',units='K')
        self.add_param('elec_price',
                       val = 0.13,
                       desc='cost of electricity per kilowatt hour',
                       units='USD/(kW*h)')
        self.add_output('tot_power', val=0.0, desc='Total tube power output', units='kW')
        self.add_output('tot_energy', val=0.0, desc='Total tube energy output', units='kJ')
        self.add_output('cost_pwr',
                        val=0.0,
                        desc='Cost for tube power requirements',
                        units='USD')

    def solve_nonlinear(self, params, unknowns, resids):
        #Propulsion power and energy requirements
        tot_prop_power = (params['prop_power']/1000.0)*params['num_thrust'] #kW
        prop_energy = tot_prop_power*(params['time_thrust']/3600.0) #kW*h

        #Cooling power requirement
        #cooling_power =

        unknowns['tot_power'] = params['vac_power'] + tot_prop_power # + cooling_power #kW
        unknowns['tot_energy'] = params['vac_energy_day']/24.0 + prop_energy #kW*h
        unknowns['cost_pwr'] = unknowns['tot_power']*params['elec_price'] #kW*(USD/kWh)

if __name__ == '__main__':
    
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', TubePower())
    prob.setup()
    prob.run()

    print('Total Tube Power Required [kW]: %f' % prob['comp.tot_power'])
    print('Total Tube Energy Required [kJ]: %f' % prob['comp.tot_energy'])
    print('Tube Power Cost [USD/h]: %f' % prob['comp.cost_pwr'])

