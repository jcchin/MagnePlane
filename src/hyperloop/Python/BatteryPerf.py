'''Main Source: 'Conceptual Modeling of Electric and Hybrid-Electric Propulsion for UAS Applications, published by Georgia Tech' '''

import math, numpy, scipy
from openmdao.core.component import Component
from openmdao.api import IndepVarComp, Component, Problem, Group, ScipyOptimizer, ExecComp, SqliteRecorder

    ##'''Allows stand-alone batteries to be simulated without an electric circuit within NPSS for use in validating the procedure. '''
class Battery_perf(Component):
    def __init__(self):
        '''input params '''
        super(Battery_perf,self).__init__()
        self.add_param('N_cells', val=378.0, desc='Number of cells in the battery Stack', units='cells')
        self.add_param('N_parallel', val= 100.0, desc='Number of cells in parallel in the battery stack', units='cells')

        self.add_param('Exp_zone_amp', val=0.284, desc='Voltage lost over the exponential zone of battery', units='Volts') #A in modeling plan
        self.add_param('Exp_zone_time_const', val=1.171, desc='Time constant for exponential zone of discharge curve', units='Amp-hours^-1') #B in modeling plan
        self.add_param('Polarization_voltage', val=0.0036, desc='Voltage lost due to polarization', units='Volts') #K in modeling plan
        self.add_param('Capacity', val=8.0, desc='Nominal capacity of one cell', units='Amp-hours')
        self.add_param('No_load_voltage', val=1.28, desc='No-load constant voltage of the battery', units='Volts')#V_0 in modeling plan
        self.add_param('Resistance', val=0.035, desc='Internal resistance of one cell', units='Ohms')
        self.add_param('State_of_charge', val=100.0, desc='Percent of charge left in battery', units='percentage')
        self.add_param('Old_State_of_charge', val=100.0, desc='Percent of charge left in battery', units='percentage')
        self.add_param('Current', val=1000.0, desc='Current drain applied to the battery', units='Amps')
        self.add_param('Voltage', val=0.0, desc='Voltage output of battery based on current draw', units='Volts')
        self.add_param('timeStep', val=0.05, desc='Time step for model', units='minutes')
        self.add_param('k_Peukert', val=1.01193, desc='Peukert Coefficient', units='none')



    def solve_nonlinear(self, params, unknowns, resids):
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
        Current = params['Current']
        Voltage = params['Voltage']
        k_Peukert = params['k_Peukert']
        timeStep = params['timeStep']


        params['Voltage'] = self.voltage_calc(Current,N_cells, N_parallel,Old_State_of_charge,k_Peukert, Capacity,No_load_voltage, Polarization_voltage,Exp_zone_amp,Exp_zone_time_const, Resistance, timeStep)



    def voltage_calc(self,Current,N_cells, N_parallel, Old_State_of_charge,k_Peukert, Capacity,No_load_voltage, Polarization_voltage,Exp_zone_amp,Exp_zone_time_const, Resistance, timeStep):
        CurrBatt = Current / N_parallel #the capacity of the battery
        Capacity_1 = (Old_State_of_charge / 100) * Capacity #Capacity remaining in series, the time dependent instantaneous charge of the battery


        s = Capacity_1 / Capacity

        #The voltage as a function of s and the instantaneous current I.

        VoltageBatt = No_load_voltage - (Polarization_voltage * (1 / s)) + Exp_zone_amp * (numpy.e ** ( -Exp_zone_time_const * (1 - s)*Capacity )) - (Resistance * CurrBatt)

        State_of_charge = ((Capacity_1 - (timeStep / 60) * CurrBatt**k_Peukert) / Capacity) * 100

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







