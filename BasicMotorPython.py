from __future__ import print_function
import numpy
from openmdao.api import IndepVarComp, Component, Problem, Group

"""This code takes in desired torque, RPM, maximum power for a BLDC motor and outputs
motor size (D*D*L), weight, maximum current, and maximum voltage.
"""

class BasicMotor(Component):

    def __init__(self):
        super(BasicMotor, self).__init__()

        #Designated inputs
        self.add_param('Torque', val=500.0, desc='Output torque', units='ft-lb')
        self.add_param('Max_RPM', val=4600.0, desc='max rpm of motor', units='rpm')
        self.add_param('DesignPower', val=50.0, desc='Design value of motor', units='hp')

        #Additional inputs
        self.add_param('Kv', val=0.1, desc='Speed/volt', units='rad/s/V')
        self.add_param('Speed', val=50.0, desc='Output shaft mechanical speed', units='rad/s')
        self.add_param('imax', val=500.0, desc='Max motor phase current', units='A')
        self.add_param('PolePairs', val=6, desc='Number of pole pairs in motor', units='none')
        self.add_param('Inductance', val=0.0, desc='Motor inductance', units='Henrys')
        self.add_param('kappa', val=0.6, desc='Base speed/max speed', units='none')
        self.add_param('LDratio', val=1.5, desc='Length to diameter ratio of motor', units='none')
        self.add_param('R0', val=0.2, desc='Motor Phase Internal Resistance at 0degC', units='Ohms')
        self.add_param('I0', val=0.0, desc='Motor No-load current', units='A')
        self.add_param('I0_Des', val=0.0, desc='Motor No-load Current at Nbase', units='A')
        self.add_param('nphase', val=3, desc='Number of motor phases', units='none')


        #Desired outputs
        self.add_output('Current', val=2.0, desc='AC `input current magnitude', units='Amps')
        self.add_output('maxCurrent', val=0.0, desc='Max input current',units='Amps')
        self.add_output('Voltage', val=500.0, desc='AC voltage across motor', units='V')
        self.add_output('Mass', val=0.0, desc='Mass of motor', units='')
        self.add_output('Volume',val=0.0,desc='Volume of motor modeled as a cylinder',units='ft^3')
        self.add_output('D2L',val=0.0,desc='D-squared*L parameter which is ~ to Torque',units='ft^3')

        #Additional outputs
        self.add_output('Resistance', val=0.0, desc='Resistance of stator', units='Ohms')

    def solve_nonlinear(self,params,unknowns,resids):

        Torque = params['Torque']
        Max_RPM = params['Max_RPM']
        DesignPower = params['DesignPower']

        #additional params

        Kv = params['Kv']
        Speed = params['Speed']
        imax = params['imax']
        PolePairs = params['PolePairs']
        Inductance = params['Inductance']
        kappa = params['kappa']
        LDratio = params['LDratio']
        R0 = params['R0']
        I0 = params['I0']
        I0_Des = params['I0_Des']
        nphase = params['nphase']


        #Desired unknowns
        unknowns['Current'] = self.Current_calc(Torque,Kv)
        Current = unknowns['Current']

        unknowns['maxCurrent'] = self.maxCurrent_calc(Speed,Max_RPM,kappa,imax,Current,I0,I0_Des,R0,Kv,DesignPower)
        maxCurrent = unknowns['maxCurrent']

        #unknowns['D2L'] = self.D2L_calc(DesignPower, Max_RPM, kappa)
        unknowns['D2L'] = self.D2L_calc(Torque)
        D2L = unknowns['D2L']

        unknowns['Mass'] = self.Mass_calc(DesignPower,LDratio,D2L)
        Mass = unknowns['Mass']

        unknowns['Volume'] = self.Size_calc(LDratio,D2L)
        Volume = unknowns['Volume']



        #Additional unknowns
        unknowns['Resistance'] = self.R_calc(imax,D2L,LDratio,nphase)
        Resistance = unknowns['Resistance']

        unknowns['Voltage'] = self.Voltage_calc(Speed, PolePairs, Inductance, Current, Kv, Resistance)
        Voltage = unknowns['Voltage']

    def Current_calc(self,Torque,Kv):
        Kt = .73756214837/Kv
        Current = Torque/Kt
        return Current

    def maxCurrent_calc(self,Speed,Max_RPM,kappa,imax,Current,I0,I0_Des,R0,Kv,DesignPower):
        w = Speed*2*numpy.pi/60.0
        wmax = Max_RPM*2*numpy.pi/60.0
        wbase = kappa*wmax
        Pmax = DesignPower*745.7
        Tmax = Pmax/wbase #Max phase torque in N-m
        Kv = (imax - I0_Des)/Tmax * (30.0/numpy.pi)
        Kt = (30.0/numpy.pi) * (1.0/Kv)
        maxCurrent = Tmax*1.355818/Kt + I0
        Voltage = Current*R0 + w / (Kv * numpy.pi/30.0)
        return maxCurrent
    
    """def D2L_calc(self,DesignPower,Max_RPM,kappa):
        Pmax = DesignPower*746.0
        wmax = Max_RPM*2*numpy.pi/60.0
        wbase = kappa*wmax
        Tmax = Pmax/wbase
        D2L = 293722.0*((Tmax)**(0.7592))
        return D2L
    """

    def D2L_calc(self,Torque):
        D2L = 293722.0*(Torque**0.7592)
        return D2L

    def R_calc(self,imax,D2L,LDratio,nphase):
        As = 688.7*imax
        Dbase = (D2L/LDratio)**(1.0/3.0)/1000
        Tph = As*3.14159 * Dbase / imax / nphase / 2.0 #Calculate number of coil turns required
        rpert = 48.8387296964863*imax**(-1.00112597971171) #Calculate resistance per km per turn
        lm = Dbase * 3.14159 #Calculate length of a single winding
        Rpturn = lm * rpert / 1000.0 #Calculate total resistance per turn

        Resistance = Rpturn*Tph*nphase
        return Resistance

    def Voltage_calc(self,Speed,PolePairs,Inductance,Current,Kv,Resistance):
        
        Frequency = Speed*PolePairs/(2*numpy.pi)
        inductorImpedance = Frequency*Inductance 
        speedVoltage = Speed/Kv
        resistorVoltage = Current*Resistance
        inductorVoltage = Current*inductorImpedance #*j
        realVoltage = speedVoltage + resistorVoltage
        
        Voltage = numpy.sqrt(inductorVoltage**2+realVoltage**2)
        return Voltage
    

    def Size_calc(self,LDratio,D2L):
        
        Dbase = (D2L/LDratio)**(1.0/3.0)/1000
        Lbase = LDratio*Dbase
        Volume = (numpy.pi/4)*(Dbase**2)*Lbase
        return Volume

    def Mass_calc(self,DesignPower,LDratio,D2L):
        Pmax = DesignPower*746.0
        Dbase = (D2L/LDratio)**(1.0/3.0)/1000
        
        Mass = 0.0000070646*D2L**0.9386912061
        return Mass


if __name__== '__main__':
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', BasicMotor())
    prob.setup()
    prob.run()



    print ('Current: %f' %prob['comp.Current'])
    print ('Max Current [A]: %f' %prob['comp.maxCurrent'])
    print ('Max Voltage [V]: %f' %prob['comp.Voltage'])
    print ('Motor Size (Volume) [m^3]: %f' %prob['comp.Volume'])
    print ('Motor Size (D^2*L) [mm^3]: %f x10e-6' %prob['comp.D2L'])
    print ('Motor Weight: %f [kg]' %prob['comp.Mass'])

