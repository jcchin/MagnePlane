

"""
Notes
    ----
   Allows sizing of battery base	d on design power load and necessary capacity


Parameters
    ----
    SpecEnergy: float.
         Specific Energy of Battery in W*h/kg. Default value is 100.0
    PowerDensity: float
         Power Density of Battery in 'W/m^3. Default value is 1.0.
    PowerBattNom : float
        Nominal Power Output of Battery in W. Default value is 1.0.
    VoltageNominal : float
        Nominal Voltage of Battery in V. Default value is 3.6.
    SpecEnergy1: float
        Specific Energy 1 in W*h/kg. Default value is 175.0
    SpecEnergy2: float
        Specific Energy 2 in W*h/kg. Default value is 128.79
    SpecEnergy3: float
        Specific Energy 3 in W*h/kg. Default value is 93.28
    SpecEnergy4: float
        Specific Energy 4 in W*h/kg. Default value is 61.94
    SpecEnergy5: float
        Specific Energy 5 in W*h/kg. Default value is 41.24
    SpecEnergy6: float
        Specific Energy 6 in W*h/kg. Default value is 11.37
    DesPower: float
        Design Power Load in W. Default value is 65000.0
    PqPdes_Max: float
        Maximum Power to Design Load Ratio in W. Default value is 1.4
    Capacity: float
       Single cell Nominal Capacity in A*h. Default value is 45.0
    Ncells: float
        Number of cells necessary to perform that mission in cells. Default value is 146.0
    C_max: float
        Maximum rating the battery can run in A*h. Default value is 3.37037

Returns
    ----
    PowerDensity: float
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




import math, numpy, scipy
from openmdao.core.component import Component
from openmdao.api import IndepVarComp, Component, Problem, Group, ScipyOptimizer, ExecComp, SqliteRecorder


class BatteryWeight(Component):
    def __init__(self):
        super(BatteryWeight, self).__init__()


        self.add_param('PowerBattNom', val=97500, desc='Nominal Power Output of Battery', units='W')
        self.add_param('VoltageNominal', val=3.09, desc='Nominal Voltage of Battery', units='V')
        self.add_param('SpecEnergy1', val=175.0, desc='Specific Energy 1', units='W*h/kg')
        self.add_param('SpecEnergy2', val=128.79, desc='Specific Energy 2', units='W*h/kg')
        self.add_param('SpecEnergy3', val=93.28, desc='Specific Energy 3', units='W*h/kg')
        self.add_param('SpecEnergy4', val=61.94, desc='Specific Energy 4', units='W*h/kg')
        self.add_param('SpecEnergy5', val=41.24, desc='Specific Energy 5', units='W*h/kg')
        self.add_param('SpecEnergy6', val=11.37, desc='Specific Energy 6', units='W*h/kg')
        self.add_param('DesPower', val=65000.0, desc='Design Power Load', units='W')
        self.add_param('PqPdes_Max', val=1.4, desc='Maximum Power to Design Load Ratio', units='W')
        self.add_param('Capacity', val=45.0, desc='Single cell Nominal Capacity', units='A*h')
        self.add_param('Ncells', val=146.0, desc='Number of cells necessary to perform that mission', units='none')
        self.add_param('C_max', val=3.37037, desc='Maximum rating the battery can run', units='1/h')

        self.add_state('SpecEnergy', val=120., desc='specific energy', units='W*h/kg', upper=175. )
        self.add_output('PowerDensity', 0.0, desc='Power Density', units='W/m^3')
        self.add_output('StackWeight', 0.0, desc='Weight of Stack', units='kg')
        self.add_output('StackVol', 1.0, desc='Volume of Stack', units='m^3')

    def _compute_outputs(self, params, unknowns, resids):
        SpecEnergy = unknowns['SpecEnergy'] #upper bound -- far right
        SpecEnergy1 = params['SpecEnergy1']
        SpecEnergy2 = params['SpecEnergy2']
        SpecEnergy3 = params['SpecEnergy3']
        SpecEnergy4 = params['SpecEnergy4']
        SpecEnergy5 = params['SpecEnergy5']
        SpecEnergy6 = params['SpecEnergy6'] #lower bound -- far left
        DesPower = params['DesPower']
        PqPdes_Max = params['PqPdes_Max']
        Capacity = params['Capacity']
        VoltageNominal = params['VoltageNominal']
        Ncells = params['Ncells']
        C_max = params['C_max']
        PowerBattNom = params['PowerBattNom']

        PowerDensity = self.calc_power_density(SpecEnergy, SpecEnergy1,SpecEnergy2,SpecEnergy3,SpecEnergy4,SpecEnergy5,SpecEnergy6)

        #calculates battery weight and volume
        Energy = Capacity * VoltageNominal * Ncells
        StackWeight = Energy / SpecEnergy
        StackVol = Ncells*PowerBattNom/PowerDensity # Possible Error: used to be PowerDensity??

        return StackWeight, StackVol, PowerDensity

    def apply_nonlinear(self, p, u ,r):
        # Linear solver drives residual to 0 to find Stack Weight and Stack Volume

        sw, sv, pdr = self._compute_outputs(p, u, r)
        theta = numpy.arctan(p['C_max'])
        thetaR = numpy.arctan(pdr/u['SpecEnergy'])
        r['SpecEnergy'] = ((thetaR - theta) / theta)

        r['StackWeight'] = sw - u['StackWeight']
        r['StackVol'] = sv - u['StackVol']

    def solve_nonlinear(self, p, u, r):
        sw, sv, pdr = self._compute_outputs(p,u,r)
        u['StackWeight'] = sw
        u['StackVol'] = sv


    def calc_power_density(self,SpecEnergy ,SpecEnergy1,SpecEnergy2,SpecEnergy3,SpecEnergy4,SpecEnergy5,SpecEnergy6 ):
        #Caculates Power Density

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


if __name__ == '__main__':
    from openmdao.api import Newton, ScipyGMRES, NLGaussSeidel
    import matplotlib.pyplot as plt
    # set up problem
    root = Group()
    p = Problem(root)
    p.root.add('comp', BatteryWeight())

    p.root.deriv_options['type'] = 'fd'
    p.root.nl_solver = Newton()
    p.root.ln_solver = ScipyGMRES()

    p.setup()
    p.run()

    # print following properties
    print ('SpecEnergy: %f' % p['comp.SpecEnergy'])
    print ('StackWeight(kg) : %f' % p['comp.StackWeight'])
    print ('StackVol(m^3) : %f' % p['comp.StackVol'])



    #poly-fitting power-density calc


    power_den = []
    power_den2 = []
    for i in range(11, 175):
        power_den.append(BatteryWeight().calc_power_density(i, 175.0, 128.79, 93.28, 61.94, 41.24, 11.37))
        power_den2.append(numpy.polyval([3.08157099e-07, -8.98578291e-05, -4.02194358e-03, 3.97380319e+00, -4.98700200e+02, 2.10772320e+04], i))
    i = range(11, 175)
    z = numpy.polyfit(i, power_den, 5)
    print  z
    plt.plot(i, power_den)
    plt.plot(i, power_den2)
    plt.show()