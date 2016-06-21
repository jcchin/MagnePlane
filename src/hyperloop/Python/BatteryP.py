
"""Notes
        ----
       Calculates parameters like Nparallel(number of cells in parallel), Nseries(Number of cells in series), Ncells(total no of cells) and C_Max(max rating)
       The calculated parameters are then use to estimate battery weight in BatteryWeight.py

    Parameters
        ----
        DesPower: float
            Fully Charged Voltage in Volts. Default value is 2.0.
        PqPdes_Max: float
            End of Exponential Zone Voltage in Volts. Default value is 2.0
        FlightTime: float
            End of Nominal Zone Voltage in Volts. Default value is 2.0
        Current: float
            Charge at end of Exponential Curve in Amp-hrs. Default value is 2.0
        CapDisLimit: float
            Charge at end of Nominal Zone in Amp-hrs. Default value is 2.0
        TimeDesPower: float
             Default value is 2.0
        Ncells: float
             Number of cells necessary to perform that mission in cells. Default value is 2.0
        Nparallel: float
             Calculated numbers of cells in parallel in cells.Default value is 10.0
        Nseries: float
            Calculated number of cells in series in cells. Default value is 0.0
        Capacity: float
            Current drain applied to the battery in Amps. Default value is 1000.0
        C_rating: float
            State of Charge limiting factor in percent. Default value is 0.2
        C_max: float
            Design Power Load in Watts. Default value is 100.0
        DesVoltage: float
            Design Voltage in Volts. Default value is 10.0
        FlightTime: float
            Time since battery is discharging in minutes. Default value is 8.0

    Returns
        ----
        StackWeight : float
            Total Capacity required for design in Amp-hrs. Default value is 0.0.

        StackVol : float
            Number of cells in parallel in the battery stack in cells. Default value is 0.0.

        CapDis : float
            Voltage lost over the exponential zone of battery in Volts. Default value is 0.0.
        CapDisBattDesPower : float
            Time constant for exponential zone of discharge curve in Amp-hours^-1 . Default value is 0.0.

        VoltageBatt: float
            Internal Resistance in Volts. Default value is 0.0.
        Voltage: float
            Voltage lost due to polarization in Ohms. Default value is 0.0.
        CurrBatt : float
            No-load constant voltage of the battery in Volts. Default value is 0.0.
    References
        ----
       Main Source : 'Conceptual Modeling of Electric and Hybrid-Electric Propulsion for UAS Applications, published by Georgia Tech
       Good explanation of capacity: http://www.powerstream.com/battery-capacity-calculations.htm
        """


import math, numpy, scipy
from openmdao.core.component import Component
from openmdao.api import IndepVarComp, Component, Problem, Group, ScipyOptimizer, ExecComp, SqliteRecorder


class BatteryP(Component):
    def __init__(self):

        super(BatteryP, self).__init__()

        # input to the equation
        self.add_param('DesPower', val=100.0, desc='Design Power Load', units='W')
        self.add_param('PqPdes_Max', val=1.5, desc='Maximum Power to Design Load Ratio', units='W')
        self.add_param('FlightTime', val=88, desc='Time since battery is discharging', units='minutes')
        self.add_param('Current', val=2.3, desc='Current Drain Applied to battery sack', units='Amps')
        self.add_param('CapDisLimit', val=0.2, desc='Percent of battery capacity left for reserves', units='percent')
        self.add_param('TimeDesPower', val=1.0, desc='Time from beginning of flight to moment of design power load', units='minutes')
        self.add_param('Ncells', val=1.0, desc='Number of cells necessary to perform that mission', units='none')
        self.add_param('Nparallel', val=100.0, desc='Calculated numbers of cells in parallel', units='none')
        self.add_param('Nseries', val=0.0, desc='Calculated number of cells in series', units='none')
        self.add_param('Capacity', val=1.0, desc='Single cell Nominal Capacity', units='Amp-hrs')
        self.add_param('C_rating', val=0.3, desc='C rating of the battery at which capacity is measured', units='1/hr')
        self.add_param('C_max', val=4.1, desc='Maximum rating the battery can run', units='1/hr')
        self.add_param('ExpZoneAmp', val=0.180, desc='Voltage lost over the exponential zone of battery', units='volts')
        self.add_param('ExpZoneTimeConst', val=2.1065, desc='Time constant for exponential zone of discharge curve',units='Amp-hours^-1')
        self.add_param('PolarizationVoltage', val=0.0243, desc='Voltage lost due to polarization', units='volts')
        self.add_param('NoLoadVoltage', val=1.431, desc='No-load constant voltage of battery', units='volts')
        self.add_param('Resistance', val=0.0095, desc='Internal resistance of battery', units='ohms')
        self.add_param('k_1', val=1, desc='Technology factor on the polarization voltage (K)', units='none')
        self.add_param('k_2', val=1, desc='Technology factor on the battery capacity', units='none')
        self.add_param('k_3', val=1, desc='Technology factor on the exponential amplitude (A)', units='none')
        self.add_param('k_4', val=1, desc='Technology factor on the exponential time constant (B)', units='none')
        self.add_param('k_5', val=1, desc='Technology factor on the internal resistance', units='none')
        self.add_param('State_of_Charge', val=100.0, desc='Initial state of charge of the battery', units='percent')
        self.add_param('NewStateOfCharge', val=100.0, desc='Final state of charge of the battery', units='percent')
        self.add_param('dischargeInterval', val=100.0, desc='Time interval for the discharge of the battery',units='Minutes')
        self.add_param('switch_Des', val=True, desc="Design State", units='none')
        self.add_param('StackDesignVoltage', val=300.0, desc='Design stack voltage', units='volts')
        self.add_param('k_Peukert', val=1.01193, desc='Peukert Coefficient', units='none')
        self.add_param('switchDes', val="DESIGN", desc='Design State', units='boolean')
        self.add_output('StackWeight', val=0.0, desc='Weight of the battery stack', units='kg')
        self.add_output('StackVol', val=0.0, desc='Volume of the battery stack', units='m^3')
        self.add_output('CapDis', val=0.0, desc='Calculated capacity necessary', units='Amp-hours')
        self.add_output('CapDisBattDesPower', val=0.0, desc='Calculated discharge before design power time', units='Amp-hours')
        self.add_output('VoltageBatt', val=0.0, desc='Voltage output of a single cell', units='volts')
        self.add_output('Voltage', val=1.0, desc='Voltage output of battery stack', units='volts')
        self.add_output('CurrBatt', val=0.0, desc='Current draw on a single cell', units='amps')


    def solve_nonlinear(self, params, unknowns, resids):

        switchDes = params['switchDes']
        DesPower = params['DesPower']
        PqPdes_Max = params['PqPdes_Max']
        FlightTime = params['FlightTime']
        Current = params['Current']
        CapDisLimit = params['CapDisLimit']

        TimeDesPower = params['TimeDesPower']
        Ncells = params['Ncells']
        Nparallel = params['Nparallel']
        Nseries = params['Nseries']
        Capacity = params['Capacity']
        C_rating = params['C_rating']
        C_max = params['C_max']

        ExpZoneAmp = params['ExpZoneAmp']
        ExpZoneTimeConst = params['ExpZoneTimeConst']
        PolarizationVoltage = params['PolarizationVoltage']
        NoLoadVoltage = params['NoLoadVoltage']
        Resistance = params['Resistance']
        k_1 = params['k_1']
        k_2 = params['k_2']
        k_3 = params['k_3']
        k_4 = params['k_4']
        k_5 = params['k_5']
        StackWeight = unknowns['StackWeight']
        StackVol = unknowns['StackVol']
        State_of_Charge = params['State_of_Charge']
        NewStateOfCharge = params['NewStateOfCharge']
        dischargeInterval = params['dischargeInterval']
        switch_Des = params['switch_Des']
        CapDis = unknowns['CapDis']
        CapDisBattDesPower = unknowns['CapDisBattDesPower']
        VoltageBatt = unknowns['VoltageBatt']
        Voltage = unknowns['Voltage']
        CurrBatt = unknowns['CurrBatt']
        StackDesignVoltage = params['StackDesignVoltage']
        k_Peukert = params['k_Peukert']



        unknowns['Voltage'], params['Current'], params['Nseries'], params['Ncells'], params['C_max'] = self.params_calc(switchDes, Capacity,Resistance, ExpZoneAmp, ExpZoneTimeConst, C_rating, k_Peukert, NoLoadVoltage, k_1,k_2, k_3, k_4, k_5, PolarizationVoltage,CapDisLimit, Nparallel, DesPower,PqPdes_Max,StackDesignVoltage,Current, State_of_Charge, dischargeInterval)


    def params_calc(self,switchDes, Capacity,Resistance, ExpZoneAmp, ExpZoneTimeConst, C_rating, k_Peukert, NoLoadVoltage, k_1,k_2, k_3, k_4, k_5, PolarizationVoltage,CapDisLimit, Nparallel, DesPower,PqPdes_Max,StackDesignVoltage,Current, State_of_Charge, dischargeInterval):


            if switchDes == "DESIGN":
                CapDis = Nparallel * (k_2 * Capacity * (1 - CapDisLimit))
                C_max = DesPower * PqPdes_Max / StackDesignVoltage / Capacity / Nparallel
                CurrBatt = DesPower / StackDesignVoltage / Nparallel # Caculating current of battery
                CapDisBattDesPower = 0.0

                Capacity_1 = k_2 * Capacity - CapDisBattDesPower

            PeukCap = (Capacity * C_rating) ** k_Peukert / C_rating #Caculating total current independent capacity of cell at design point



            if switchDes == "OFFDESIGN":
                CurrBatt = Current / Nparallel
                Capacity_1 = State_of_Charge / 100 * k_2 * PeukCap
                NewStateOfCharge = (Capacity_1 - dischargeInterval / 60. * CurrBatt ** k_Peukert) / PeukCap * 100
                StateOfCharge = NewStateOfCharge



            VoltageBatt = NoLoadVoltage - k_1 * PolarizationVoltage * (k_2 * PeukCap / Capacity_1) + k_3 * ExpZoneAmp * numpy.e ** (-k_4 * ExpZoneTimeConst * (k_2 * PeukCap - Capacity_1)) - k_5 * Resistance * CurrBatt

            if switchDes == "DESIGN":

                PBattDesPower = VoltageBatt * CurrBatt
                Ncells = math.ceil(DesPower / PBattDesPower)
                Nseries = math.ceil(Ncells / Nparallel)
                Ncells = Nseries * Nparallel
                Voltage = Nseries * VoltageBatt



            return Voltage, Current, Nseries,Ncells, C_max


if __name__ == '__main__':
    # set up problem
    root = Group()
    p = Problem(root)
    p.root.add('comp', BatteryP())
    p.setup()
    p.root.list_connections()
    p.run()

    # print following properties
    print 'Voltage(volts) : %f' % p['comp.Voltage']
    print 'Current(amps) : %f' % p['comp.Current']
    print 'Nseries(cells) : %f' % p['comp.Nseries']
    print 'Nparallel(cells) : %f' % p['comp.Nparallel']
    print 'Ncells(cells) : %f' % p['comp.Ncells']
