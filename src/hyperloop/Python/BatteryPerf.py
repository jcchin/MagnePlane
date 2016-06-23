"""Notes
    ----
   Allows stand-alone batteries to be simulated without an electric circuit within NPSS for use in validating the procedure.

Parameters
    ----
    Ncells: float
         Number of cells in the battery Stack in cells. Default value is 378.0
    N_parallel: float
         Number of cells in parallel in the battery stack in cells.Default value is 100.0
    Exp_zone_amp : float
        Voltage lost over the exponential zone of battery in Volts. Default value is 0.284.
    Exp_zone_time_const : float
        Time constant for exponential zone of discharge curve in Amp-hours^-1 . Default value is 1.171.
    Polarization_voltage: float
        Voltage lost due to polarization in Volts. Default value is 0.0036
    Capacity: float
        Nominal capacity of one cell in Amp-hrs. Default value is 8.0
    No_load_voltage: float
        No-load constant voltage of the battery in Volts. Default value is 1.28
    Resistance: float
        No-load constant voltage of the battery in Volts. Default value is 0.035
    State_of_charge: float
        No-load constant voltage of the battery in Volts. Default value is 100.0
    Old_State_of_charge: float
        No-load constant voltage of the battery in Volts. Default value is 100.0
    timeStep: float
        No-load constant voltage of the battery in Volts. Default value is 0.05
    k_Peukert: float
        No-load constant voltage of the battery in Volts. Default value is 1.01193

Returns
    ----
    Current : float
        Current drain applied to the battery in Amps. Default value is 0.0.

    Voltage : float
        Voltage output of battery based on current draw in Volts. Default value is 0.0.

References
    ----
   Main Source : 'Conceptual Modeling of Electric and Hybrid-Electric Propulsion for UAS Applications, published by Georgia Tech
   Good explanation of capacity: http://www.powerstream.com/battery-capacity-calculations.htm
    """



import math, numpy, scipy
from openmdao.core.component import Component
from openmdao.api import IndepVarComp, Component, Problem, Group, ScipyOptimizer, ExecComp, SqliteRecorder


class Battery_perf(Component):
    def __init__(self):
        '''input params '''
        super(Battery_perf,self).__init__()
        self.add_param('N_cells', val=1.0, desc='Number of cells in the battery Stack', units='cells')
        self.add_param('N_parallel', val= 1.0, desc='Number of cells in parallel in the battery stack', units='cells')
        self.add_param('Exp_zone_amp', val=0.2838, desc='Voltage lost over the exponential zone of battery', units='Volts') #A in modeling plan
        self.add_param('Exp_zone_time_const', val=1.1708, desc='Time constant for exponential zone of discharge curve', units='Amp-hours^-1') #B in modeling plan
        self.add_param('Polarization_voltage', val=0.0361, desc='Voltage lost due to polarization', units='Volts') #K in modeling plan
        self.add_param('Capacity', val=45.0, desc='Nominal capacity of one cell', units='Amp-hours')
        self.add_param('No_load_voltage', val=4.2, desc='No-load constant voltage of the battery', units='Volts')#V_0 in modeling plan
        self.add_param('Resistance', val=0.0006058, desc='Internal resistance of one cell', units='Ohms')
        self.add_param('State_of_charge', val=100., desc='Percent of charge left in battery', units='percentage')
        self.add_param('Old_State_of_charge', val=100., desc='Percent of charge left in battery', units='percentage')
        self.add_param('timeStep', val=6, desc='Time step for model', units='seconds')
       # self.add_param('k_Peukert', val=1.01193, desc='Peukert Coefficient', units='none')
        self.add_param('Current', val=200., desc='Current drain applied to the battery', units='Amps')
        self.add_param('Voltage', val=1.0, desc='Voltage output of battery based on current draw', units='Volts')



    def solve_nonlinear(self, params, unknowns, resids):
        #inputs
        N_cells = params['N_cells']
        N_parallel = params['N_parallel']
        Exp_zone_amp = params['Exp_zone_amp']
        Exp_zone_time_const = params['Exp_zone_time_const']
        Polarization_voltage = params['Polarization_voltage']
        Capacity = params['Capacity']
        No_load_voltage = params['No_load_voltage']
        Resistance = params['Resistance']
        State_of_charge = params['State_of_charge']
        Old_State_of_charge =  params['Old_State_of_charge']

        timeStep = params['timeStep']

        #outputs
        Current = params['Current']
        Voltage = params['Voltage']


        params['Voltage'] = self.voltage_calc(Current,N_cells, N_parallel,Old_State_of_charge, Capacity,No_load_voltage, Polarization_voltage,Exp_zone_amp,Exp_zone_time_const, Resistance, timeStep)



    def voltage_calc(self,Current,N_cells, N_parallel, Old_State_of_charge, Capacity,No_load_voltage, Polarization_voltage,Exp_zone_amp,Exp_zone_time_const, Resistance, timeStep):
        CurrBatt = Current / N_parallel #the capacity of the battery
        Capacity_1 = (Old_State_of_charge / 100) * Capacity #Capacity remaining in series, the time dependent instantaneous charge of the battery


        s = Capacity_1 / Capacity

        #The voltage as a function of s and the instantaneous current I.

        VoltageBatt = No_load_voltage - (Polarization_voltage * (1 / s)) + Exp_zone_amp * (numpy.e ** ( -Exp_zone_time_const * (1 - s)*Capacity )) - (Resistance * CurrBatt)

        State_of_charge = (Capacity_1 - timeStep /60* CurrBatt) / Capacity * 100

        #Total Voltage (voltage adds in series)
        Voltage = N_cells /N_parallel * VoltageBatt

        return Voltage


if __name__ == '__main__':
    # set up problem
    root = Group()
    p = Problem(root)
    p.root.add('comp', Battery_perf())
    p.setup()
    p.root.list_connections()
    p.run()

    # print following properties
    print 'Voltage(volts) : %f' % p['comp.Voltage']
    print 'Current(amps) : %f' % p['comp.Current']







