'''Battery Weight Determination Class modeled after
Main Source : 'Conceptual Modeling of Electric and Hybrid-Electric Propulsion for UAS Applications, published by Georgia Tech' '''

#Allows sizing of battery base	d on design power load and necessary capacity

import math, numpy, scipy
from openmdao.core.component import Component
from openmdao.api import IndepVarComp, Component, Problem, Group, ScipyOptimizer, ExecComp, SqliteRecorder


class BatteryWeight(Component):
    def __init__(self):
        super(BatteryWeight, self).__init__()
        self.add_param('SpecEnergy', val=100.0, desc='Specific Energy of Battery', units='Wh/kg')
        self.add_param('PowerDensity', val= 1.0, desc='Power Density of Battery', units='W/m^3')
        self.add_param('SpecPower', val=1.0, desc='Specific Power of Battery', units='W/m^3')
        self.add_param('PowerBattNom', val=1.0, desc='Nominal Power Output of Battery', units='W')
        self.add_param('VoltageNominal', val=3.6, desc='Nominal Voltage of Battery', units='V')

        self.add_param('SpecEnergy1', val=175.0, desc='Specific Energy 1', units='W/m^3')
        self.add_param('SpecEnergy2', val=128.79, desc='Specific Energy 2', units='W/m^3')
        self.add_param('SpecEnergy3', val=93.28, desc='Specific Energy 3', units='W/m^3')
        self.add_param('SpecEnergy4', val=61.94, desc='Specific Energy 4', units='W/m^3')
        self.add_param('SpecEnergy5', val=41.24, desc='Specific Energy 5', units='W/m^3')
        self.add_param('SpecEnergy6', val=11.37, desc='Specific Energy 6', units='W/m^3')
        self.add_param('r_accuracy', val=0.0001, desc='tolerance for data from ragone', units='')

        self.add_param('DesPower', val=100.0, desc='Design Power Load', units='W')
        self.add_param('PqPdes_Max', val=1.5, desc='Maximum Power to Design Load Ratio', units='W')

        self.add_param('Capacity', val=8.0, desc='Single cell Nominal Capacity', units='Amp-hrs')
        self.add_param('Ncells', val=18900.0, desc='Number of cells necessary to perform that mission', units='none')
        self.add_param('C_Max', val=0.25, desc='Maximum rating the battery can run', units='1/hr')






        self.add_output('PowerDensityR', 0.0, desc='Power Density', units='TBD')
        self.add_output('StackWeight', 0.0, desc='Weight of Stack', units='kg')
        self.add_output('StackVol', 0.0, desc='Volume of Stack', units='m^3')

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





        #unknowns['PowerDensityR'] = self.calc_power_density(SpecEnergy,SpecEnergy1,SpecEnergy2,SpecEnergy3,SpecEnergy4,SpecEnergy5,SpecEnergy6)
        #unknowns['theta_R'] = self.calc_theta_R(SpecEnergy, unknowns['PowerDensityR'])

        unknowns['StackWeight'], unknowns['StackVol'] = self. calc_stack(SpecEnergy,SpecEnergy1,SpecEnergy2,SpecEnergy3, SpecEnergy4, SpecEnergy5, SpecEnergy6, r_accuracy, DesPower, PqPdes_Max, Capacity, VoltageNominal, Ncells,C_Max, PowerBattNom)









#


    def calc_power_density(self,SpecEnergy ,SpecEnergy1,SpecEnergy2,SpecEnergy3,SpecEnergy4,SpecEnergy5,SpecEnergy6 ):

        if SpecEnergy < SpecEnergy1 and SpecEnergy >= SpecEnergy2:
            PowerDensity_ret = (0.053*SpecEnergy**2) - (21.14*SpecEnergy) + 2084.1
        elif SpecEnergy < SpecEnergy2 and SpecEnergy and SpecEnergy >= SpecEnergy3:
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

    def calc_theta_R(self, SpecEnergy, PowerDensityR):
        theta_R = numpy.arctan(PowerDensityR / SpecEnergy)
        return theta_R



    def calc_stack(self, SpecEnergy,SpecEnergy1,SpecEnergy2,SpecEnergy3,SpecEnergy4,SpecEnergy5,SpecEnergy6, r_accuracy, DesPower, PqPdes_Max, Capacity, VoltageNominal, Ncells,C_Max, PowerBattNom):
        PowerDensityR = self.calc_power_density(SpecEnergy, SpecEnergy1,SpecEnergy2,SpecEnergy3,SpecEnergy4,SpecEnergy5,SpecEnergy6 )
        thetaR = self.calc_theta_R(SpecEnergy,PowerDensityR )

        Power = DesPower * PqPdes_Max
        Energy = Capacity * VoltageNominal * Ncells
        theta = numpy.arctan(C_Max)


        #Calculates specific energy
        while abs((theta - thetaR) > r_accuracy):
            SpecEnergy = min((SpecEnergy * (1 + .1 * ((thetaR - theta) / theta))), 175.)
            PowerDensityR = self.calc_power_density(SpecEnergy)
            thetaR = math.arctan(PowerDensityR/SpecEnergy)
            count = 0.0

            #no clue what this conditional is for. just ported over the logic from the C++ code
            if (SpecEnergy == 175. and theta <= thetaR):
                break
            count = count + 1
            if (count > 500):
                break


        #calculates battery weight and volume
        StackWeight = Energy / SpecEnergy
        StackVol = Ncells*PowerBattNom / PowerDensityR
        return StackWeight, StackVol



if __name__ == '__main__':
    # set up problem
    root = Group()
    p = Problem(root)
    p.root.add('comp', BatteryWeight())
    p.setup()
    p.root.list_connections()
    p.run()

    # print following properties
    print 'StackWeight(kg) : %f' % p['comp.StackWeight']
    print 'StackVol(m^3) : %f' % p['comp.StackVol']
