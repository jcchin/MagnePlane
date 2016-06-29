"""
Calculates parameters like Nparallel(number of cells in parallel), Nseries(Number of cells in series), Ncells(total no of cells) and C_Max(max rating)
The calculated parameters are then use to estimate battery weight in BatteryWeight.py
"""

import math, numpy, scipy
from openmdao.core.component import Component
from openmdao.api import IndepVarComp, Component, Problem, Group, ScipyOptimizer, ExecComp, SqliteRecorder

class BatteryP(Component):
    """
    Params
    ------
    DesPower : float
        Fully Charged Voltage in V. Default value is 2.0.
    PqPdes_Max : float
        End of Exponential Zone Voltage in V. Default value is 2.0
    FlightTime : float
        End of Nominal Zone Voltage in V. Default value is 2.0
    CapDisLimit : float
        Charge at end of Nominal Zone in A*h. Default value is 2.0
    TimeDesPower : float
         Default value is 2.0
    Nparallel : float
         Calculated numbers of cells in parallel in cells.Default value is 10.0
    Capacity : float
        Current drain applied to the battery in A. Default value is 1000.0
    C_rating : float
        State of Charge limiting factor in percent. Default value is 0.2
    ExpZoneAmp : float
        Design Voltage in V. Default value is 0.2838
    ExpZoneTimeConst : float
        Design Voltage in A*h^-1. Default value is 1.1708
    PolarizationVoltage : float
        Design Voltage in V. Default value is 0.0361
    Resistance : float
        Design Voltage in ohm. Default value is 0.0006058
    k_1 : float
        Technology factor on the polarization voltage (K) in none. Default value is 1.0
    k_2 : float
        Technology factor on the battery capacity in none. Default value is 1.0
    k_3 : float
        Technology factor on the exponential amplitude (A) in none. Default value is 1.0
    k_4 : float
        Technology factor on the exponential time constant (B) in none. Default value is 1.0
    k_5: float
        Technology factor on the internal resistance. in none Default value is 1.0
    State_of_Charge : float
        Initial state of charge of the battery in percent. Default value is 100.0
    NewStateOfCharge : float
        Final state of charge of the battery in percent. Default value is 100.0
    dischargeInterval : float
        Time interval for the discharge of the battery in s. Default value is 3.0
    StackDesignVoltage : float
        Design stack voltage in V. Default value is 300.0
    k_Peukert : float
        Peukert Coefficient in none. Default value is 1.01193

    Returns
    -------
    StackWeight : float
        Total Capacity required for design in A*h. Default value is 1.0.
    StackVol : float
        Number of cells in parallel in the battery stack in cells. Default value is 1.0.
    CapDis : float
        Voltage lost over the exponential zone of battery in V. Default value is 0.0.
    CapDisBattDesPower : float
        Time constant for exponential zone of discharge curve in A*h^-1 . Default value is 0.0.
    VoltageBatt : float
        Internal Resistance in V. Default value is 0.0.
    Voltage : float
        Voltage lost due to polarization in ohms. Default value is 0.0.
    CurrBatt : float
        No-load constant voltage of the battery in V. Default value is 0.0.
    Ncells : float
         Number of cells necessary to perform that mission in cells. Default value is 2.0
    C_max : float
        Design Power Load in W. Default value is 100.0
    Nseries : float
        Calculated number of cells in series in cells. Default value is 0.0
    Current : float
        Charge at end of Exponential Curve in A*h. Default value is 2.0

    Notes
    -----
    [1] Conceptual Modeling of Electric and Hybrid-Electric Propulsion for UAS Applications, published by Georgia Tech
    Good explanation of capacity: http://www.powerstream.com/battery-capacity-calculations.htm
    """

    def __init__(self):

        super(BatteryP, self).__init__()

        # input to the equation
        self.add_param('DesPower', val=65000.0, desc='Design Power Load', units='W')
        self.add_param('PqPdes_Max', val=1.4, desc='Maximum Power to Design Load Ratio', units='W')
        self.add_param('FlightTime', val=750.0, desc='Time since battery is discharging', units='s')
        self.add_param('CapDisLimit', val=0.1, desc='Percent of battery capacity left for reserves', units='percent')
        self.add_param('TimeDesPower', val=0.0, desc='Time from beginning of flight to moment of design power load', units='h')
        self.add_param('Nparallel', val=2.0, desc='Calculated numbers of cells in parallel', units='unitless')
        self.add_param('Capacity', val=45.0, desc='Single cell Nominal Capacity', units='A*h')
        self.add_param('C_rating', val=0.3, desc='C rating of the battery at which capacity is measured', units='1/h')
        self.add_param('ExpZoneAmp', val=0.2838, desc='Voltage lost over the exponential zone of battery', units='V')
        self.add_param('ExpZoneTimeConst', val=1.1708, desc='Time constant for exponential zone of discharge curve',units='A*h^-1')
        self.add_param('PolarizationVoltage', val=0.0361, desc='Voltage lost due to polarization', units='V')
        self.add_param('NoLoadVoltage', val=4.2, desc='No-load constant voltage of battery', units='V')
        self.add_param('Resistance', val=0.0006058, desc='Internal resistance of battery', units='ohm')
        self.add_param('k_1', val=1., desc='Technology factor on the polarization voltage (K)', units='unitless')
        self.add_param('k_2', val=1., desc='Technology factor on the battery capacity', units='unitless')
        self.add_param('k_3', val=1., desc='Technology factor on the exponential amplitude (A)', units='unitless')
        self.add_param('k_4', val=1., desc='Technology factor on the exponential time constant (B)', units='unitless')
        self.add_param('k_5', val=1., desc='Technology factor on the internal resistance', units='unitless')
        self.add_param('C_max_in', val=0.0, desc='Maximum rating the battery can run', units='1/h')
        self.add_param('NumCells', val=1.0, desc='Guess at number of cells necessary to perform mission', units='unitless')
        self.add_param('State_of_Charge', val=100.0, desc='Initial state of charge of the battery', units='percent')
        self.add_param('NewStateOfCharge', val=100.0, desc='Final state of charge of the battery', units='percent')
        self.add_param('dischargeInterval', val=3.0, desc='Time interval for the discharge of the battery',units='s')
        self.add_param('StackDesignVoltage', val=300.0, desc='Design stack voltage', units='V')
        self.add_param('k_Peukert', val=1.01193, desc='Peukert Coefficient', units='unitless')
        self.add_output('StackWeight', val=1.0, desc='Weight of the battery stack', units='kg')
        self.add_output('StackVol', val=1.0, desc='Volume of the battery stack', units='m^3')
        self.add_output('CapDis', val=0.0, desc='Calculated capacity necessary', units='A*h')
        self.add_output('CapDisBattDesPower', val=0.0, desc='Calculated discharge before design power time', units='A*h')
        self.add_output('VoltageBatt', val=0.0, desc='Voltage output of a single cell', units='V')
        self.add_output('Voltage', val=0.0, desc='Voltage output of battery stack', units='V')
        self.add_output('CurrBatt', val=0.0, desc='Current draw on a single cell', units='A')
        self.add_output('C_max', val=0.0, desc='Maximum rating the battery can run', units='1/h')
        self.add_output('Ncells', val=0.0, desc='Number of cells necessary to perform that mission', units='unitless')
        self.add_output('Nseries', val=0.0, desc='Calculated number of cells in series', units='unitless')
        self.add_output('Current', val=200., desc='Current Drain Applied to battery sack', units='A')

    def solve_nonlinear(self, params, unknowns, resids):

        switchDes = "DESIGN"
        DesPower = params['DesPower']
        PqPdes_Max = params['PqPdes_Max']
        FlightTime = params['FlightTime']
        Current = unknowns['Current']
        CapDisLimit = params['CapDisLimit']

        TimeDesPower = params['TimeDesPower']
        NumCells = params['NumCells']
        Ncells = unknowns['Ncells']
        Nparallel = params['Nparallel']
        Nseries = unknowns['Nseries']
        Capacity = params['Capacity']
        C_rating = params['C_rating']
        C_max_in = params['C_max_in']

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
        CapDis = unknowns['CapDis']
        CapDisBattDesPower = unknowns['CapDisBattDesPower']
        VoltageBatt = unknowns['VoltageBatt']
        Voltage = unknowns['Voltage']
        CurrBatt = unknowns['CurrBatt']
        StackDesignVoltage = params['StackDesignVoltage']
        k_Peukert = params['k_Peukert']
        C_max = unknowns['C_max']

        unknowns['Voltage'], unknowns['Current'], unknowns['Nseries'], unknowns['Ncells'], unknowns['C_max'] = self.params_calc(switchDes, Capacity,Resistance, ExpZoneAmp, ExpZoneTimeConst, C_rating, k_Peukert, NoLoadVoltage, k_1,k_2, k_3, k_4, k_5, PolarizationVoltage,CapDisLimit, Nparallel, DesPower,PqPdes_Max,StackDesignVoltage,Current, State_of_Charge, dischargeInterval)


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


            print(NoLoadVoltage, k_1, k_2, k_3, k_4, k_5, PeukCap, Capacity_1, ExpZoneAmp, Resistance, CurrBatt, PolarizationVoltage)
            VoltageBatt = NoLoadVoltage - \
                          (k_1 * PolarizationVoltage) * \
                          (k_2 * PeukCap / Capacity_1) + \
                          (k_3 * ExpZoneAmp) * \
                          (numpy.exp(((-k_4) * ExpZoneTimeConst * (k_2 * PeukCap - Capacity_1)))) - \
                          (k_5 * Resistance * CurrBatt)

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
    print ('Voltage of stack : %f' % p['comp.Voltage'])
    print ('Current of stack: %f' % p['comp.Current'])
    print ('Nparallel : %f' % p['comp.Nparallel'])
    print ('Ncells(cells) : %f' % p['comp.Ncells'])
    print ('C_max(volt) : %f' % p['comp.C_max'])
