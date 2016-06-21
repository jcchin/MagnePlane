


"""Notes
        ----
       Derives important parameters from given parameters from the battery discharge curve which later feeds into BatteryPerf.py
       Capacity required is key factor, as it drives the number of cells required, voltage and current drawn.

    Parameters
        ----
        V_full: float
            Fully Charged Voltage in Volts. Default value is 2.0.
        V_exp: float
            End of Exponential Zone Voltage in Volts. Default value is 2.0
        V_nom: float
            End of Nominal Zone Voltage in Volts. Default value is 2.0
        Q_exp: float
            Charge at end of Exponential Curve in Amp-hrs. Default value is 2.0
        Q_nom: float
            Charge at end of Nominal Zone in Amp-hrs. Default value is 2.0
        Q_n: float
             Default value is 2.0
        V_n: float
             Default value is 2.0
        I_n: float
             Default value is 2.0
        n: float
            Efficiency of battery in percent. Default value is 0.8
        Current: float
            Current drain applied to the battery in Amps. Default value is 1000.0
        s_limit: float
            State of Charge limiting factor in percent. Default value is 0.2
        DesPower: float
            Design Power Load in Watts. Default value is 100.0
        DesVoltage: float
            Design Voltage in Volts. Default value is 10.0
        FlightTime: float
            Time since battery is discharging in minutes. Default value is 8.0

    Returns
        ----
        Capacity_required : float
            Total Capacity required for design in Amp-hrs. Default value is 0.0.

        N_parallel : float
            Number of cells in parallel in the battery stack in cells. Default value is 0.0.

        Exp_zone_amp : float
            Voltage lost over the exponential zone of battery in Volts. Default value is 0.0.
        Exp_zone_time_const : float
            Time constant for exponential zone of discharge curve in Amp-hours^-1 . Default value is 0.0.

        Polarization_voltage : float
            Internal Resistance in Volts. Default value is 0.0.
        int_R : float
            Voltage lost due to polarization in Ohms. Default value is 0.0.
        No_load_voltage : float
            No-load constant voltage of the battery in Volts. Default value is 0.0.
    References
        ----
       Main Source : 'Conceptual Modeling of Electric and Hybrid-Electric Propulsion for UAS Applications, published by Georgia Tech
       Good explanation of capacity: http://www.powerstream.com/battery-capacity-calculations.htm
        """



import math, numpy, scipy
from openmdao.core.component import Component
from openmdao.api import IndepVarComp, Component, Problem, Group, ScipyOptimizer, ExecComp, SqliteRecorder

class Battery_discharge_curve_params(Component):
    def __init__(self):
        '''input params '''
        super(Battery_discharge_curve_params,self).__init__()
        self.add_param('V_full', val=0.0, desc='Fully Charged Voltage', units='Volts')
        self.add_param('V_exp', val=2.0, desc='End of Exponential Zone Voltage', units='Volts')
        self.add_param('V_nom', val=2.0, desc='End of Nominal Zone Voltage', units='Volts')
        self.add_param('Q_exp', val=2.0, desc='Charge at end of Exponential Curve', units='Amp-hrs')
        self.add_param('Q_nom', val=2.0, desc='Charge at end of Nominal Zone', units='Amp-hrs')
        self.add_param('Q_n', val=2.0, desc='', units='Amp-hrs')
        self.add_param('V_n', val=2.0, desc='', units='Volts')
        self.add_param('I_n', val=2.0, desc='', units='Amps')
        self.add_param('n', val=0.8, desc='Efficiency of battery', units='percent')
        self.add_param('Current', val=1000.0, desc='Current drain applied to the battery', units='Amps')
        self.add_param('s_limit', val=0.2, desc='State of Charge limiting factor', units='percent') #s_limit in the modeling plan
        self.add_param('DesPower', val=100.0, desc='Design Power Load', units='W')
        self.add_param('DesVoltage', val=10.0, desc='Design Voltage', units='Voltage')
        self.add_param('FlightTime', val=8, desc='Time since battery is discharging', units='minutes')
        self.add_output('Capacity_required', val=0.0, desc='Total Capacity required for design', units='Amp-hr') #Q_dis in the modeling plan
        self.add_output('N_parallel', val=0.0, desc='Number of cells in parallel in the battery stack', units='cells')
        self.add_output('Exp_zone_amp', 0.0, desc='Voltage lost over the exponential zone of battery', units='Volts') #A in modeling plan
        self.add_output('Exp_zone_time_const', 0.0, desc='Time constant for exponential zone of discharge curve', units='Amp-hours^-1') #B in modeling plan
        self.add_output('Polarization_voltage', 0.0, desc='Voltage lost due to polarization', units='Volts') #K in modeling plan
        self.add_output('int_R', 0.035, desc='Internal Resistance', units='Ohms') #R in modeling Plan
        self.add_output('No_load_voltage', 0.0, desc='No-load constant voltage of the battery', units='Volts') #V_0 in modeling plan

    def solve_nonlinear(self, params, unknowns, resids):
        V_full = params['V_full']
        V_exp = params['V_exp']
        V_nom = params['V_nom']
        Q_exp = params['Q_exp']
        Q_nom = params['Q_nom']
        Q_n = params['Q_n']
        V_n = params['V_n']
        I_n  = params['I_n']
        n = params['n']
        Current = params['Current']
        s_limit = params['s_limit']
        DesPower = params['DesPower']
        DesVoltage = params['DesVoltage']
        FlightTime = params['FlightTime']


        unknowns['Capacity_required'] = self.capacity_required_calc(DesPower,DesVoltage,FlightTime)
        unknowns['N_parallel'] = self.N_parallel_calc(unknowns['Capacity_required'], s_limit)
        unknowns['Exp_zone_amp']  = self.Exp_zone_amp_calc(V_full, V_exp )
        unknowns['Exp_zone_time_const'] = self.Exp_zone_time_const_calc(Q_exp )
        unknowns['Polarization_voltage'] = self.Polarization_voltage_calc( V_full, V_nom, Q_nom, Q_n, unknowns['Exp_zone_amp'], unknowns['Exp_zone_time_const'])
        unknowns['int_R'] = self.int_R_calc(V_n, I_n, n)
        unknowns['No_load_voltage']  = self. No_load_voltage_calc(V_full, Current, unknowns['N_parallel'],unknowns['Polarization_voltage'],unknowns['int_R'],unknowns['Exp_zone_amp'])






    def Exp_zone_amp_calc(self, V_full, V_exp ):
        Exp_zone_amp = V_full - V_exp

        return Exp_zone_amp


    def Exp_zone_time_const_calc(self,Q_exp ):
        Exp_zone_time_const = 3 / Q_exp
        return Exp_zone_time_const


    #calculating the polarization voltage
    def Polarization_voltage_calc(self, V_full, V_nom, Q_nom, Q_n, Exp_zone_amp, Exp_zone_time_const):
        Polarization_voltage = (V_full - V_nom + Exp_zone_amp*(numpy.e**(- Exp_zone_time_const * Q_nom) - 1)) * (Q_n - Q_nom)

        return Polarization_voltage

    #calculating the internal resistance
    def int_R_calc(self, V_n, I_n, n):
        int_R = V_n*(1/I_n)* (1 - n)

        return int_R

    #Calculating the no-load voltage
    def No_load_voltage_calc(self,V_full, Current, N_parallel,Polarization_voltage,int_R,Exp_zone_amp ):
        CurrBatt = Current / N_parallel
        No_load_voltage = V_full + Polarization_voltage + CurrBatt*int_R - Exp_zone_amp

        return No_load_voltage

    #Calculating number of cells in parallel
    def N_parallel_calc(self, Capacity_required, s_limit):
        N_parallel = numpy.ceil(Capacity_required / (1 - s_limit))
        return N_parallel


    #Calculating capcity required from design voltage, design power and mission time.
    def capacity_required_calc(self, DesPower,DesVoltage, FlightTime):
        Capacity_required = (DesPower / DesVoltage) * FlightTime
        return Capacity_required


if __name__ == '__main__':
    # set up problem
    root = Group()
    p = Problem(root)
    p.root.add('comp', Battery_discharge_curve_params())
    p.setup()
    p.root.list_connections()
    p.run()

    # print following properties
    print 'Exp_zone_amp(volts) : %f' % p['comp.Exp_zone_amp']
    print 'Exp_zone_time_const(Amp-hours^-1) : %f' % p['comp.Exp_zone_time_const']
    print 'Polarization_voltage(volts) : %f' % p['comp.Polarization_voltage']
    print 'int_R(ohms) : %f' % p['comp.int_R']
    print 'No_load_voltage(volts) : %f' % p['comp.No_load_voltage']
    print 'N_parallel(cells) : %f' % p['comp.N_parallel']
    print 'Capacity Required(Amp-hrs) : %f' % p['comp.Capacity_required']