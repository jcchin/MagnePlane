import math, numpy, scipy
from openmdao.core.component import Component
from openmdao.api import IndepVarComp, Component, Problem, Group, ScipyOptimizer, ExecComp, SqliteRecorder

class Battery_perf(Component):

    """Notes
        ----
        Allows stand-alone batteries to be simulated without an electric circuit within NPSS for use in validating the procedure.

    Parameters
        ----
        Ncells: float
             Number of cells in the battery Stack in cells. Default value is 1.0
        Nparallel: float
             Number of cells in parallel in the battery stack in cells.Default value is 1.0
        Exp_zone_amp : float
            Voltage lost over the exponential zone of battery in V. Default value is 0.2838.
        Exp_zone_time_const : float
            Time constant for exponential zone of discharge curve in A*h^-1 . Default value is 1.171.
        Polarization_voltage: float
            Voltage lost due to polarization in V. Default value is 0.0036
        Capacity: float
            Nominal capacity of one cell in A*h. Default value is 45.0
        No_load_voltage: float
            No-load constant voltage of the battery in V. Default value is 4.2
        Resistance: float
            Internal resistance of one cell in ohm. Default value is 0.0006058
        State_of_charge: float
            Percent of charge left in battery in percent. Default value is 100.0
        Old_State_of_charge: float
            Percent of charge left in battery in percent. Default value is 100.0
        timeStep: float
            Time step for model in s. Default value is 6.0
        k_Peukert: float
            Peukert Coefficient in none. Default value is 1.01193

    Returns
        ----
        Current : float
            Current drain applied to the battery in A. Default value is 0.0.

        Voltage : float
            Voltage output of battery based on current draw in V. Default value is 0.0.

    References
        ----
        Main Source : 'Conceptual Modeling of Electric and Hybrid-Electric Propulsion for UAS Applications, published by Georgia Tech
        Good explanation of capacity: http://www.powerstream.com/battery-capacity-calculations.htm
    """
    def __init__(self):
        super(Battery_perf,self).__init__()
        self.add_param('Ncells', val=1.0, desc='Number of cells in the battery Stack', units='unitless')
        self.add_param('Nparallel', val= 1.0, desc='Number of cells in parallel in the battery stack', units='unitless')
        self.add_param('Exp_zone_amp', val=0.2838, desc='Voltage lost over the exponential zone of battery', units='V') #A in modeling plan
        self.add_param('Exp_zone_time_const', val=1.1708, desc='Time constant for exponential zone of discharge curve', units='A*h^-1') #B in modeling plan
        self.add_param('Polarization_voltage', val=0.0361, desc='Voltage lost due to polarization', units='V') #K in modeling plan
        self.add_param('Capacity', val=45.0, desc='Nominal capacity of one cell', units='A*h')
        self.add_param('No_load_voltage', val=4.2, desc='No-load constant voltage of the battery', units='V')#V_0 in modeling plan
        self.add_param('Resistance', val=0.0006058, desc='Internal resistance of one cell', units='ohm')
        self.add_param('State_of_charge', val=100., desc='Percent of charge left in battery', units='percent')
        self.add_param('Old_State_of_charge', val=100., desc='Percent of charge left in battery', units='percent')
        self.add_param('timeStep', val=6, desc='Time step for model', units='s')
        self.add_param('k_Peukert', val=1.01193, desc='Peukert Coefficient', units='unitless')
        self.add_output('Current', val=200., desc='Current drain applied to the battery', units='A')
        self.add_output('Voltage', val=1.0, desc='Voltage output of battery based on current draw', units='V')

    def solve_nonlinear(self, params, unknowns, resids):
        #inputs
        Ncells = params['Ncells']
        Nparallel = params['Nparallel']
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
        Current = unknowns['Current']
        Voltage = unknowns['Voltage']

        unknowns['Voltage'] = self.voltage_calc(Current,Ncells, Nparallel,Old_State_of_charge, Capacity,No_load_voltage, Polarization_voltage,Exp_zone_amp,Exp_zone_time_const, Resistance, timeStep)

    def voltage_calc(self,Current,Ncells, Nparallel, Old_State_of_charge, Capacity,No_load_voltage, Polarization_voltage,Exp_zone_amp,Exp_zone_time_const, Resistance, timeStep):
        CurrBatt = Current / Nparallel #the capacity of the battery
        Capacity_1 = (Old_State_of_charge / 100) * Capacity #Capacity remaining in series, the time dependent instantaneous charge of the battery

        s = Capacity_1 / Capacity

        #The voltage as a function of s and the instantaneous current I.
        VoltageBatt = No_load_voltage - (Polarization_voltage * (1 / s)) + Exp_zone_amp * (numpy.e ** ( -Exp_zone_time_const * (1 - s)*Capacity )) - (Resistance * CurrBatt)
        State_of_charge = (Capacity_1 - timeStep /60* CurrBatt) / Capacity * 100

        #Total Voltage (voltage adds in series)
        Voltage = Ncells /Nparallel * VoltageBatt

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
    print ('Voltage(volts) individual: %f' % p['comp.Voltage'])
    print ('Current(amps) individual: %f' % p['comp.Current'])
    