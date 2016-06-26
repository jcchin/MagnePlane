

"""
Notes
    ----
   Allows sizing of battery base	d on design power load and necessary capacity


Parameters
    ----
    SpecEnergy: float
         Specific Energy of Battery in Wh/kg. Default value is 100.0
    PowerDensity: float
         Power Density of Battery in 'W/m^3. Default value is 1.0
    SpecPower : float
        Specific Power of Battery in W/kg. Default value is 1.0.
    PowerBattNom : float
        Nominal Power Output of Battery in W. Default value is 1.0.
    VoltageNominal : float
        Nominal Voltage of Battery in Volts . Default value is 3.6.
    SpecEnergy1: float
        Specific Energy 1 in W-hrs/kg. Default value is 175.0
    SpecEnergy2: float
        Specific Energy 2 in W-hrs/kg. Default value is 128.79
    SpecEnergy3: float
        Specific Energy 3 in W-hrs/kg. Default value is 93.28
    SpecEnergy4: float
        Specific Energy 4 in W-hrs/kg. Default value is 61.94
    SpecEnergy5: float
        Specific Energy 5 in W-hrs/kg. Default value is 41.24
    SpecEnergy6: float
        Specific Energy 6 in W-hrs/kg. Default value is 11.37
    r_accuracy: float
        tolerance for data from ragone in percent. Default value is 0.0001
    DesPower: float
        Design Power Load in Watts. Default value is 100.0
    PqPdes_Max: float
        Maximum Power to Design Load Ratio in Watts. Default value is 1.5
    Capacity: float
       Single cell Nominal Capacity in Amp-hrs. Default value is 8.0
    Ncells: float
        Number of cells necessary to perform that mission in cells. Default value is 18900.0
    C_Max: float
        Maximum rating the battery can run in Amp-hrs. Default value is 0.25

Returns
    ----
    PowerDensityR: float
        Power Density in W/m^3. Default value is 0.0
    StackWeight: float
        StackWeight in kg. Default value is 0.0
    StackVol: float
        Volume of Stack in m^3. Default value is 0.0

References
    ----
   Main Source : 'Conceptual Modeling of Electric and Hybrid-Electric Propulsion for UAS Applications, published by Georgia Tech
   Good explanation of capacity: http://www.powerstream.com/battery-capacity-calculations.htm

"""




import math, numpy, scipy
import matplotlib.pyplot as plt
from openmdao.core.component import Component
from openmdao.api import IndepVarComp, Component, Problem, Group, ScipyOptimizer, ExecComp, SqliteRecorder


class BatteryWeight(Component):
    def __init__(self):
        super(BatteryWeight, self).__init__()
        self.add_param('SpecEnergy', val=100.0, desc='Specific Energy of Battery', units='Wh/kg')
        self.add_param('PowerDensity', val= 400.0, desc='Power Density of Battery', units='W/m^3;')
        self.add_param('SpecPower', val=1.0, desc='Specific Power of Battery', units='W/kg')
        self.add_param('PowerBattNom', val=97500, desc='Nominal Power Output of Battery', units='W')
        self.add_param('VoltageNominal', val=3.09, desc='Nominal Voltage of Battery', units='V')
        self.add_param('SpecEnergy1', val=175.0, desc='Specific Energy 1', units='W-hrs/m^3')
        self.add_param('SpecEnergy2', val=128.79, desc='Specific Energy 2', units='W-hrs/m^3')
        self.add_param('SpecEnergy3', val=93.28, desc='Specific Energy 3', units='W-hrs/m^3')
        self.add_param('SpecEnergy4', val=61.94, desc='Specific Energy 4', units='W-hrs/m^3')
        self.add_param('SpecEnergy5', val=41.24, desc='Specific Energy 5', units='W-hrs/m^3')
        self.add_param('SpecEnergy6', val=11.37, desc='Specific Energy 6', units='W-hrs/m^3')
        self.add_param('r_accuracy', val=0.0001, desc='tolerance for data from ragone', units='')
        self.add_param('DesPower', val=65000.0, desc='Design Power Load', units='W')
        self.add_param('PqPdes_Max', val=1.4, desc='Maximum Power to Design Load Ratio', units='W')
        self.add_param('Capacity', val=45.0, desc='Single cell Nominal Capacity', units='Amp-hrs')
        self.add_param('Ncells', val=146.0, desc='Number of cells necessary to perform that mission', units='none')
        self.add_param('C_Max', val=3.37037, desc='Maximum rating the battery can run', units='1/hr')
        self.add_output('PowerDensityR', 0.0, desc='Power Density', units='W/m^3')
        self.add_output('StackWeight', 0.0, desc='Weight of Stack', units='kg')
        self.add_output('StackVol', 1.0, desc='Volume of Stack', units='m^3')

    def solve_nonlinear(self, params, unknowns, resids):
        SpecEnergy = params['SpecEnergy'] #upper bound -- far right
        SpecEnergy1 = params['SpecEnergy1']
        SpecEnergy2 = params['SpecEnergy2']
        SpecEnergy3 = params['SpecEnergy3']
        SpecEnergy4 = params['SpecEnergy4']
        SpecEnergy5 = params['SpecEnergy5']
        SpecEnergy6 = params['SpecEnergy6'] #lower bound -- far left
        r_accuracy = params['r_accuracy']
        DesPower = params['DesPower']
        PqPdes_Max = params['PqPdes_Max']
        Capacity = params['Capacity']
        VoltageNominal = params['VoltageNominal']
        Ncells = params['Ncells']
        C_Max = params['C_Max']
        PowerBattNom = params['PowerBattNom']
        PowerDensity = params['PowerDensity']

        #unknowns['PowerDensityR'] = self.calc_power_density(SpecEnergy,SpecEnergy1,SpecEnergy2,SpecEnergy3,SpecEnergy4,SpecEnergy5,SpecEnergy6)
        #unknowns['theta_R'] = self.calc_theta_R(SpecEnergy, unknowns['PowerDensityR'])

        unknowns['StackWeight'], unknowns['StackVol'] = self.calc_stack(SpecEnergy,SpecEnergy1,SpecEnergy2,SpecEnergy3, SpecEnergy4, SpecEnergy5, SpecEnergy6, r_accuracy, DesPower, PqPdes_Max, Capacity, VoltageNominal, Ncells,C_Max, PowerBattNom, PowerDensity)



    def calc_power_density(self,SpecEnergy ,SpecEnergy1,SpecEnergy2,SpecEnergy3,SpecEnergy4,SpecEnergy5,SpecEnergy6 ):

        if SpecEnergy < SpecEnergy1 and SpecEnergy >= SpecEnergy2:
            PowerDensity_ret = (0.053*SpecEnergy**2) - (21.14*SpecEnergy) + 2084.1
        elif SpecEnergy <= SpecEnergy2 and SpecEnergy and SpecEnergy >= SpecEnergy3:
            PowerDensity_ret = (0.1942*SpecEnergy**2) - (69.348*SpecEnergy) + 5975.6

        elif SpecEnergy <= SpecEnergy3 and SpecEnergy >= SpecEnergy4:
            PowerDensity_ret = (0.3887*SpecEnergy**2) - (132.23 * SpecEnergy) + 10197

        elif (SpecEnergy <= SpecEnergy4 and SpecEnergy >= SpecEnergy5):
            PowerDensity_ret = (-0.3048 * SpecEnergy ** 2) - (96.666 * SpecEnergy) + 10441

        elif (SpecEnergy <= SpecEnergy5 and SpecEnergy >= SpecEnergy6):
            PowerDensity_ret = (-2.5546 * SpecEnergy ** 2) - (174.67 * SpecEnergy) + 17510

        elif (SpecEnergy >= SpecEnergy1):
            SpecEnergy = SpecEnergy1
            PowerDensity_ret = (0.053 * SpecEnergy ** 2) - (21.14 * SpecEnergy) + 2084.1

        elif (SpecEnergy < SpecEnergy6):
            SpecEnergy = SpecEnergy6
            PowerDensity_ret = (-2.5546 * SpecEnergy ** 2) - (174.67 * SpecEnergy) + 17510

        return PowerDensity_ret

    def calc_theta_R(self, specificEnergy, PowerDensityR):
        theta_R = numpy.arctan(PowerDensityR / specificEnergy)
        return theta_R

    def calc_stack(self, SpecEnergy,SpecEnergy1,SpecEnergy2,SpecEnergy3,SpecEnergy4,SpecEnergy5,SpecEnergy6, r_accuracy, DesPower, PqPdes_Max, Capacity, VoltageNominal, Ncells,C_Max, PowerBattNom, PowerDensity):

        PowerDensityR = self.calc_power_density(SpecEnergy, SpecEnergy1,SpecEnergy2,SpecEnergy3,SpecEnergy4,SpecEnergy5,SpecEnergy6)
        thetaR = self.calc_theta_R(SpecEnergy,PowerDensityR)
        print 'PowerDensityR : %f' % PowerDensityR


        Power = DesPower * PqPdes_Max
        Energy = Capacity * VoltageNominal * Ncells
        theta = numpy.arctan(C_Max)
        stack_weight_temp = 0.0
        count = 0

        #Calculates specific energy
        while abs((theta - thetaR)) > r_accuracy:
            SpecEnergy = min((SpecEnergy * (1 + .1 * ((thetaR - theta) / theta))), 175.)
            PowerDensityR = self.calc_power_density(SpecEnergy, SpecEnergy1,SpecEnergy2,SpecEnergy3,SpecEnergy4,SpecEnergy5,SpecEnergy6)
            thetaR = numpy.arctan(PowerDensityR/SpecEnergy)
            print 'thetaR : %f.........PowerDensityR: %f' % (thetaR, PowerDensityR)

            #no clue what this conditional is for. just ported over the logic from the C++ code
            if (SpecEnergy == 175. and theta <= thetaR):
                break
            count+=1
            if (count > 500):
                break

        SpecPower = PowerDensityR
        print("SpecEnergy: %f" %SpecEnergy)

        #calculates battery weight and volume
        StackWeight = Energy / SpecEnergy
        StackVol = Ncells*PowerBattNom/PowerDensity

        return StackWeight, StackVol


if __name__ == '__main__':
    # set up problem
    root = Group()
    p = Problem(root)
    p.root.add('comp', BatteryWeight())
    p.setup()
    p.root.list_connections()
    p.run()

    # plotting and polyfitting power density vs specific energy of 5th order
    power_den = []
    power_den2 = []
    for i in range(11, 175):
        power_den.append(BatteryWeight().calc_power_density(i, 175.0, 128.79, 93.28, 61.94, 41.24, 11.37))
        power_den2.append(numpy.polyval([3.08157099e-07, -8.98578291e-05, -4.02194358e-03, 3.97380319e+00, -4.98700200e+02, 2.10772320e+04], i))
    i = range(11, 175)
    z = numpy.polyfit(i, power_den, 5)
    print "The nodes for 5th order polyfit are: %f" %z
    plt.plot(i, power_den)
    plt.plot(i, power_den2)
    plt.show()

    # print following properties
    print ('StackWeight(kg) : %f' % p['comp.StackWeight'])
    print ('StackVol(m^3) : %f' % p['comp.StackVol'])
