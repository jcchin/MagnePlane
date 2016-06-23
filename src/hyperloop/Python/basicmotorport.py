from __future__ import print_function
import numpy
from openmdao.api import IndepVarComp, Component, Problem, Group

#
"""
This code models a Brushless DC (BLDC) motor component with the following inputs and outputs:
Inputs: Desired Torque [N-m], RPM, Maximum Power [hp]
Outputs: Max Current [A], Max Voltage [V],Motor Size (Volume), Motor size [D^2*L], Motor Weight [kg]

Relations were taken from Georgia Tech ASDL paper: "Conceptual Modeling of Electric and
Hybrid-Electric Propulsion for UAS Applications" (Gladin, Ali, Collins)

NPSS model of an electric motor for modelling of the electrical systems of a hybrid electric propulsion system.
All values are nominal for 8000 hp motor calibrated to GE SUGAR data.
"""


class BasicMotor(Component):
    def __init__(self):
        super(BasicMotor, self).__init__()

        #Inputs/PArams
        self.add_param('Torque', val=400*0.737, desc='Output torque', units='N-m')
        self.add_param('Max_RPM', val=4000.0, desc='max rpm of motor', units='rpm')
        self.add_param('DesignPower', val=150000.0/746.0, desc='Design value of motor', units='hp')

        self.add_param('Resistance', val=0.0,desc='Resistance of Stator',units='Ohms')
        self.add_param('Inductance', val=0.0, desc='Motor inductance', units='Henrys')
        self.add_param('Kv', val=0.1, desc='Speed/volt', units='rad/s/V')
        #self.add_param('Kt', val=10.0, desc='Torque/amp', units='ft-lb/A')
        self.add_param('Speed', val=2100.0, desc='Output shaft mechanical speed', units='rad/s')
        self.add_param('imax', val=300, desc='Max motor phase current', units='A')
        self.add_param('PolePairs', val=6.0, desc='Number of pole pairs in motor', units='none')  # f=w*PP/2*pi
        self.add_param('kappa', val=0.5, desc='Base speed/max speed', units='none')
        self.add_param('LDratio', val=0.78, desc='Length to diameter ratio of motor', units='none')
        self.add_param('R0', val=0.003, desc='Motor Phase Internal Resistance at 0degC',units='Ohms')  # total internal resistance
        self.add_param('I0', val=0.0, desc='Motor No-load current', units='A')
        self.add_param('I0_Des', val=0.0, desc='Motor No-load Current at Nbase', units='A')
        self.add_param('nphase', val=3.0, desc='Number of motor phases', units='none')
        self.add_param('CoreRadiusRatio', val=1.2, desc='ratio of inner diameter of core to outer', units='')
        self.add_param('B_p', val=1.7, desc='Peak Magnetic Field', units='Tesla')

        #self.add_param('H_c',val=4.2,desc='Material Coercive Force',units='A/m')
        #self.add_param('V_max',val=10000.0,desc='Max phase voltage', units='V')
        #self.add_param('As',val=95000.0,desc='Electrical Loading',units='')
        #self.add_param('Dbase',val=0.48,desc='Base 8000hp diameter for scaling purposes',units='m')
        #self.add_param('Lbase',val=0.4,desc='Base 8000hp length for scaling purposes',units='m')
        #self.add_param('k_Friction',val=1.0,desc'Friction coefficient calibration factor',units='')
        #self.add_param('Rd',val=0.0,desc='D-axis resistance per motor phase at very high speed (short circuit)',units='')
        #self.add_param('efficiency',val=0.0,desc='Motor efficiency',units='')
        #self.add_param('P_mech',val=0.0,desc='Mechanical output power',units='')

        # Desired outputs
        self.add_output('Current', val=2.0, desc='AC `input current magnitude', units='Amps')
        self.add_output('phaseVoltage', val=500.0, desc='AC voltage across motor', units='V')
        self.add_output('Mass', val=0.0, desc='Mass of motor', units='')
        self.add_output('Volume', val=0.0, desc='Volume of motor modeled as a cylinder', units='L')
        self.add_output('D2L', val=0.0, desc='D-squared*L parameter which is ~ to Torque', units='mm^3')
        self.add_output('phaseCurrent', val=0.0, desc='Max Current', units='Amps')
        # Additional outputs
        self.add_output('Phase',val=0.0, desc='phase offset between Current and Voltage',units='rad')
        self.add_output('R_calc', val=0.0, desc='Resistance of stator', units='Ohms')
        self.add_output('Tmax', val=0.0, desc='Max Torque', units='N-m')
        self.add_output('Frequency', val=60.0, desc='Frequency of Electric output waveform', units='Hz')
        self.add_output('Kt', val=10.0, desc='Torque/amp', units='ft-lb/A')

    def solve_nonlinear(self, params, unknowns, resids):
        # Desired Params
        Torque = params['Torque']
        Max_RPM = params['Max_RPM']
        DesignPower = params['DesignPower']

        # additional params
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
        CoreRadiusRatio = params['CoreRadiusRatio']


        #unknowns['Current'] = self.Current_calc(Torque, Kv, nphase)
        #Current = unknowns['Current']

        unknowns['Tmax'] = self.Tmax_calc(DesignPower, Max_RPM, kappa)
        Tmax = unknowns['Tmax']

        params['Kv'] = (imax - I0_Des)/Tmax * (30./numpy.pi)
        Kv = params['Kv']

        unknowns['Kt'] = self.Kt_calc(imax, I0_Des, Tmax, Kv)
        Kt = unknowns['Kt']

        unknowns['phaseCurrent'] = self.phaseCurrent_calc(DesignPower,Max_RPM,nphase,Kt,I0)
        phaseCurrent = unknowns['phaseCurrent']

        unknowns['phaseVoltage'] = self.phaseVoltage_calc(Speed, phaseCurrent, nphase, R0, Kv)
        phaseVoltage = unknowns['phaseVoltage']

        unknowns['D2L'] = self.D2L_calc(Tmax)
        #unknowns['D2L'] = self.D2L_calc(Torque)
        D2L = unknowns['D2L']

        unknowns['Mass'] = self.Mass_calc(DesignPower, LDratio, D2L)
        Mass = unknowns['Mass']

        unknowns['Volume'] = self.Size_calc(LDratio, D2L,CoreRadiusRatio)
        Volume = unknowns['Volume']

        # unknowns['Resistance'] = self.R_calc(imax, D2L, LDratio, nphase)
        # Resistance = unknowns['Resistance']

        unknowns['Frequency'] =Speed*PolePairs/60

    """def Current_calc(self, Torque, Kv, nphase):
        # Calculates current from input Torque
        Kt = .73756214837/Kv
        Current = Torque*Kt
        Current = Current/nphase
        return Current
    """
    def Tmax_calc(self, DesignPower, Max_RPM, kappa):
        Pmax = DesignPower * 746.0  # hp to Watts
        wmax = Max_RPM * 2 * numpy.pi / 60.0  # RPM to rad/s
        wbase = kappa * wmax  # finds base speed
        Tmax = Pmax / wbase  # Power = Torque*omega
        return Tmax

    def Kt_calc(self,imax,I0_Des,Tmax,Kv):
        Kt = (30.0 / numpy.pi) * (1.0 / Kv)
        return Kt

    def phaseCurrent_calc(self, DesignPower,Max_RPM,nphase,Kt,I0):
        # Calculates max Current from max Torque
        Pmax = DesignPower*746.0
        wmax = Max_RPM * 2 * numpy.pi / 60.0
        Torque= Pmax/wmax
        phaseCurrent = Torque * 1.355818 / Kt + I0
        phaseCurrent = phaseCurrent/nphase
        return phaseCurrent

    def phaseVoltage_calc(self,Speed,phaseCurrent,nphase,R0,Kv):
        w= Speed*2*numpy.pi/60.0
        Voltage = (phaseCurrent*nphase)*R0 + w/(Kv*numpy.pi/30.0)
        phaseVoltage = Voltage*numpy.sqrt(3.0/2.0)
        return phaseVoltage

    def D2L_calc(self, Tmax):
        D2L = 293722.0 * (Tmax ** 0.7592)  # mm^3
        return D2L

    def Size_calc(self, LDratio, D2L, CoreRadiusRatio):
        Dbase = ((D2L / LDratio) ** (1.0 / 3.0)) / 1000.0  # meters
        Lbase = LDratio * Dbase  # meters
        Volume = numpy.pi * Lbase * (Dbase / 2) ** 2 * (1-CoreRadiusRatio**2)
        return Volume

    def Mass_calc(self, DesignPower, LDratio, D2L):
        Pmax = DesignPower * 746.0
        Dbase = ((D2L / LDratio) ** (1 / 3)) / 1000  # meters
        Mass = 0.0000070646 * (D2L ** 0.9386912061)
        return Mass
"""
    def R_calc(self, imax, D2L, LDratio, nphase):
        As = 688.7 * imax
        Dbase = ((D2L / LDratio) ** (1 / 3)) / 1000  # meters
        Tph = As * numpy.pi * Dbase / imax / nphase / 2.0  # Calculate number of coil turns required
        rpert = 48.8387296964863 * imax ** (-1.00112597971171)  # Calculate resistance per km per turn
        lm = Dbase * 3.14159  # Calculate length of a single winding
        Rpturn = lm * rpert / 1000.0  # Calculate total resistance per turn
        Resistance = Rpturn * Tph * nphase
        return Resistance

    def Voltage_calc(self, Speed, PolePairs, Inductance, Current, Kv, Resistance):
        # Frequency = Speed*PolePairs/(2*numpy.pi)
        Frequency = Speed * PolePairs / 60.0
        inductorImpedance = Frequency * Inductance  # j (j=sqrt(-1))
        speedVoltage = Kv * Speed
        resistorVoltage = Current * Resistance
        inductorVoltage = Current * inductorImpedance  # *j
        realVoltage = speedVoltage + resistorVoltage
        # Phase = numpy.atan2(inductorVoltage,realVoltage)
        Voltage = numpy.sqrt(inductorVoltage ** 2 + realVoltage ** 2)
        Voltage = Voltage * numpy.sqrt(3. / 2.)
        return Voltage
"""

    def print_data():
        print('kappa: %f' % prob['comp.kappa'])
        print('imax: %f' %prob['comp.imax'])
        print('Max_RPM: %f' % prob['comp.Max_RPM'])
        print('DesignPower: %f' %prob['comp.DesignPower'])
        print('LDratio:%f' %prob['comp.LDratio'])
        print('R0: %f' % prob['comp.R0'])
        print('B_p: %f' % prob['comp.B_p'])
        print('CoreRadiusRatio: %f' % prob['comp.CoreRadiusRatio'])
        print('I0: %f' % prob['comp.I0'])
        print('Torque: %f' %prob['comp.Torque'])

        print('-----------------------------')

        #print('Current: %f' % prob['comp.Current'])
        print('Phase Current [A]: %f' % prob['comp.phaseCurrent'])
        print('Phase Voltage [V]: %f' % prob['comp.phaseVoltage'])
        print('Frequency [Hz]: %f ' % prob['comp.Frequency'])
        print('Phase: %f' % prob['comp.Phase'])
        # print('Tmax: %f [N-m]' % prob['comp.Tmax'])
        # print('Kt: %f' %prob['comp.Kt'])
        print('Kv [rad/s/V]: %f' % prob['comp.Kv'])

        print('Motor Size (Volume) [m^3] [L]: %f' % prob['comp.Volume'])
        # print ('Max Torque: %f' %prob['comp.Tmax'])
        print('Motor Size (D^2*L) [mm^3]: %f x10e-6' % prob['comp.D2L'])
        print('Motor Weight [kg]: %f ' % prob['comp.Mass'])
