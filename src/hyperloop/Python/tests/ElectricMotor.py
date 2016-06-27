from __future__ import print_function
import numpy
from openmdao.api import IndepVarComp, Component, Problem, Group



class BasicMotor(Component):
    def __init__(self):
        super(BasicMotor, self).__init__()

        #Inputs/Params
        self.add_param('Resistance', val=0.0, desc='Resistance of Stator', units='ohm')
        self.add_param('Inductance', val=0.0, desc='Motor inductance', units='H')
        self.add_param('Speed',val=1900.0,desc='Output shaft mechanical speed',units='rad/s')
        self.add_param('Torque', val=310.35*0.737, desc='Output torque', units='N*m')
        self.add_param('Kv', val=0.1, desc='Speed/volt', units='rad/s/V')
        self.add_param('Kt', val=10.0, desc='Torque/amp', units='N*m/A')
        self.add_param('PolePairs', val=6.0, desc='Number of pole pairs in motor', units='none')  # f=w*PP/2*pi
        #self.add_param('H_c',val=4.2,desc='Material Coercive Force',units='A/m')
        self.add_param('B_p', val=1.5, desc='Peak Magnetic Field', units='T') #Tesla

        self.add_param('R0', val=0.004, desc='Motor Phase Internal Resistance at 0degC',units='ohm') #ohms  # total internal resistance
        self.add_param('I0', val=0.0, desc='Motor No-load current', units='A')
        self.add_param('I0_Des', val=0.0, desc='Motor No-load Current at Nbase', units='A')
        self.add_param('imax', val=450.0, desc='Max motor phase current', units='A')
        self.add_param('nphase', val=3.0, desc='Number of motor phases', units='none')
        self.add_param('DesignPower', val=110000.0 / 746.0, desc='Design value of motor', units='hp')
        self.add_param('Max_RPM', val=2500.0, desc='max rpm of motor', units='rpm')
        self.add_param('kappa', val=0.5, desc='Base speed/max speed', units='none')
        #self.add_param('V_max',val=10000.0,desc='Max phase voltage', units='V')
        #self.add_param('As',val=95000.0,desc='Electrical Loading',units='')
        self.add_param('Dbase',val=0.48,desc='Base 8000hp diameter for scaling purposes',units='m')
        self.add_param('Lbase',val=0.4,desc='Base 8000hp length for scaling purposes',units='m')
        self.add_param('LDratio', val=0.83, desc='Length to diameter ratio of motor', units='none')
        self.add_param('CoreRadiusRatio', val=0.7, desc='ratio of inner diameter of core to outer', units='none')
        #self.add_param('k_Friction',val=1.0,desc'Friction coefficient calibration factor',units='none')
        #self.add_param('Rd',val=0.0,desc='D-axis resistance per motor phase at very high speed (short circuit)',units='ohm')
        #self.add_param('efficiency',val=0.0,desc='Motor efficiency',units='none')
        self.add_param('Pmax',val=0.0,desc='Conversion of DesignPower in hp to Watts',units='W')
        self.add_param('D2L', val=0.0, desc='D-squared*L parameter which is ~ to Torque', units='mm**3')
        self.add_param('D2L_ft',val=0.0,desc='D-squared*L parameter converted to ft^3', units='ft**3')
        self.add_param('w',val=0.0,desc='speed in rad/s',units='rad/s')
        self.add_param('wmax',val=0.0,desc='Max speed in rad/s',units='rad/s')

        # Desired outputs
        self.add_output('Current', val=2.0, desc='AC `input current magnitude', units='Amps')
        self.add_output('phaseCurrent',val=0.0,desc='Phase current',units='A')
        self.add_output('phaseVoltage', val=500.0, desc='AC voltage across motor', units='V')
        self.add_output('Phase', val=0.0, desc='phase offset between Current and Voltage', units='rad')
        self.add_output('Frequency', val=60.0, desc='Frequency of Electric output waveform', units='Hz')
        #self.add_output('P_mech',val=0.0,desc='Mechanical output power',units='hp')
        #self.add_output('P_copper',val=0.0,desc='Copper losses',units='hp')
        #self.add_output('P_iron',val=0.0,desc='Iron Losses',units='hp')
        #self.add_output('P_windage',val=0.0,desc='Windage Losses',units='hp')
        #self.add_output('P_input',val=0.0,desc='Total Power input needed',units='hp')
        self.add_output('Mass', val=0.0, desc='Mass of motor', units='kg')

        self.add_output('Volume', val=0.0, desc='Volume of motor modeled as a cylinder', units='m**3')
        self.add_output('Volume_ft',val=0.0,desc='Volume of motor modeled as a cylinder in ft',units='ft**3')
        self.add_output('R_calc', val=0.0, desc='Resistance of stator', units='ohm')
        self.add_output('Tmax', val=0.0, desc='Max Torque', units='N*m')

    def solve_nonlinear(self, params, unknowns, resids):

        Torque = params['Torque']
        Max_RPM = params['Max_RPM']
        DesignPower = params['DesignPower']

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

        params['Pmax'] = DesignPower*746.0
        Pmax = params['Pmax']

        params['w'] = Speed*2*numpy.pi/60.0
        w = params['w']
        params['wmax'] = Max_RPM * 2 * numpy.pi / 60.0
        wmax = params['wmax'] # RPM to rad/s

        #unknowns['Current'] = self.Current_calc(Torque, Kv, nphase)
        #Current = unknowns['Current']

        unknowns['Tmax'] = self.Tmax_calc(Pmax, wmax, kappa)
        Tmax = unknowns['Tmax']

        params['Kv'] = (imax - I0_Des)/Tmax * (30./numpy.pi)
        Kv = params['Kv']

        params['Kt'] = (30.0 / numpy.pi) * (1.0 / Kv)
        Kt = params['Kt']

        params['D2L'] = 293722.0 * (Tmax ** 0.7592)  # mm^3
        D2L = params['D2L']

        params['D2L_ft'] = D2L*3.53147*(10**-8)
        D2L_ft = params['D2L_ft']

        params['Dbase'] = ((D2L / LDratio) ** (1.0 / 3.0)) / 1000.0
        Dbase = params['Dbase']

        unknowns['phaseCurrent'] = self.phaseCurrent_calc(Pmax,Max_RPM,wmax,nphase,Kt,I0)
        phaseCurrent = unknowns['phaseCurrent']

        unknowns['phaseVoltage'] = self.phaseVoltage_calc(w,phaseCurrent,nphase,R0,Kv)
        phaseVoltage = unknowns['phaseVoltage']

        unknowns['Mass'] = 0.0000070646 * (D2L ** 0.9386912061)
        Mass = unknowns['Mass']

        unknowns['Volume'] = self.Size_calc(LDratio, Dbase,CoreRadiusRatio)
        Volume = unknowns['Volume']

        unknowns['Volume_ft'] = Volume*35.3147
        Volume_ft = unknowns['Volume_ft']

        params['Lbase'] = LDratio * Dbase
        Lbase = params['Lbase']

        unknowns['Frequency'] = Speed*PolePairs/60
        Frequency = unknowns['Frequency']

        # unknowns['Resistance'] = self.R_calc(imax, D2L, LDratio, nphase)
        # Resistance = unknowns['Resistance']
        #
        # unknowns['P_iron']=self.Iron_loss(Lbase,Dbase,CoreRadiusRatio,B_p,Frequency)
        # P_iron = unknowns['P_iron']
        #
        # unknowns['P_windage']=self.P_windage_calc(Dbase,CoreRadiusRatio,w,k_Friction,Lbase)
        # P_windage = unknowns['P_windage']
        #
        # unknowns['P_input'] = self.P_input_calc(Pmax,w,wmax,Current,Rtotal,nphase,P_iron,P_windage)
        # P_input = unknowns['P_input']
        #
        # unknowns['P_mech'] = w*Torque*1.355818
        # P_mech=unknowns['P_mech']
        #
        # unknowns['efficiency'] =P _input/P_mech


    def Current_calc(self, Torque, Kv, nphase):
    #Calculates Current for Total Voltage calculation
        Kt = .73756214837/Kv
        Current = Torque*Kt
        Current = Current/nphase
        return Current




    def Tmax_calc(self,Pmax,wmax,kappa):
        #Calculates max torque to later obtain D2L and Kt at max current
        wbase = kappa*wmax
        Tmax=Pmax/wbase
        return Tmax

    def phaseCurrent_calc(self, Pmax,Max_RPM,wmax,nphase,Kt,I0):
        # Calculates phase Current
        #wmin = 0.086*wmax
        Torque= Pmax/wmax
        Current = Torque * 1.355818 / Kt + I0
        phaseCurrent = Current/nphase
        return phaseCurrent

    def phaseVoltage_calc(self,w,phaseCurrent,nphase,R0,Kv):
        #Calculates phase voltage
        Voltage = (phaseCurrent*nphase)*R0 + w/(Kv*numpy.pi/30.0)
        phaseVoltage = Voltage*numpy.sqrt(3.0/2.0)
        return phaseVoltage

    def Size_calc(self, LDratio, Dbase,CoreRadiusRatio):
        #Calculates volume by representing motor as a cylinder: V=pi(r^2)*L = pi*((d/2)^2)*L
        Lbase = LDratio * Dbase  # meters
        Volume = numpy.pi * Lbase * (Dbase / 2) ** 2 * (1-CoreRadiusRatio**2)
        return Volume

"""
    def R_calc(self, Speed,imax, Dbase, w,wmax,nphase):
        #Calculates total resistance
        wbase = kappa * wmax
        As = 688.7 * imax
        Tph = As * numpy.pi * Dbase / imax / nphase / 2.0  # Calculate number of coil turns required
        rpert = 48.8387296964863 * imax ** (-1.00112597971171)  # Calculate resistance per km per turn
        lm = Dbase * 3.14159  # Calculate length of a single winding
        R = lm * rpert / 1000.0  # Calculate total resistance per turn
        Rcalc = R * Tph * nphase
<<<<<<< HEAD
=======

>>>>>>> ProjectMaster/master
        if w>wbase:
            Rtotal = R*Tph+Rd*(1-wbase/w)**2
        else:
            Rtotal = R*Tph
<<<<<<< HEAD
        return Rtotal
=======

        return Rtotal

>>>>>>> ProjectMaster/master
    def Voltage_calc(self, Speed, PolePairs, Inductance, Current, Kv, Resistance):
        Frequency = Speed*PolePairs/(2*numpy.pi)
        Current = Torque*Kt
        resistorVoltage = Current * Resistance
        inductorImpedance = Frequency * Inductance  # j (j=sqrt(-1))
        inductorVoltage = Current * inductorImpedance  # *j
        speedVoltage = Kv * Speed
        realVoltage = speedVoltage + resistorVoltage
        Voltage = numpy.sqrt(inductorVoltage ** 2 + realVoltage ** 2)
<<<<<<< HEAD
        Phase = numpy.atan2(inductorVoltage,realVoltage)
        return Voltage
=======

        Phase = numpy.atan2(inductorVoltage,realVoltage)
        return Voltage

>>>>>>> ProjectMaster/master
    def Iron_loss(self,Lbase,Dbase,CoreRadiusRatio,B_p,Frequency):
        #Calculates iron losses
        Kh=0.0275
        Kc=0.0000183
        Ke=0.0000277
        Volume = numpy.pi*Lbase*(Dbase/2.0)**2.0*(1-CoreRadiusRatio**2)
        Core_Weight = 7650.0*Volume
        P_iron = (Kh * B_p * B_p * Frequency + Kc*(B_p*Frequency)**2.0 + Ke*(B_p*Frequency)**(1.5)) * Core_Weight
        #Scales iron volume on power (assuming speed range is close to baseline)
<<<<<<< HEAD
=======

>>>>>>> ProjectMaster/master
    def P_windage_calc(self,Dbase,CoreRadiusRatio,w,k_Friction,Lbase):
        c_friction=0.1
        P_disc_wind = 0.5 * c_friction * 1.2041 * w**3. * ((Dbase/2)**5.)*(1-CoreRadiusRatio**5.0)
        P_windage=k_Friction * (c_friction * numpy.pi * (w**3.0) * (Dbase/2)**4 * Lbase * 1.2041 + P_disc_wind)
        return P_windage
<<<<<<< HEAD
=======

>>>>>>> ProjectMaster/master
    def P_input_calc(self,Pmax,w,wmax,Current,Rtotal,nphase,P_iron,P_windage):
        #Calculates total Power input required with losses and friction included
        Torque = Pmax/wmax
        P_mech=w*Torque*1.355818
<<<<<<< HEAD
        P_copper_phase=Current*Current*Rtotal
        P_copper=P_copper_phase*nphase
=======

        P_copper_phase=Current*Current*Rtotal
        P_copper=P_copper_phase*nphase

>>>>>>> ProjectMaster/master
        P_input = P_mech + P_copper + P_iron + P_windage
        return P_input
"""

if __name__ == '__main__':
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', BasicMotor())
    prob.setup()
    prob.run()

    #Printing various input parameters to check with C++ model/inputs
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

    #Outputs
    #print('Current: %f' % prob['comp.Current'])
    print('Phase Current [A]: %f' % prob['comp.phaseCurrent'])
    print('Phase Voltage [V]: %f' % prob['comp.phaseVoltage'])
    print('Frequency [Hz]: %f ' % prob['comp.Frequency'])
    print('Phase: %f' % prob['comp.Phase'])
    # print('Tmax: %f [N-m]' % prob['comp.Tmax'])
    # print('Kt: %f' %prob['comp.Kt'])
    print('Kv [rad/s/V]: %f' % prob['comp.Kv'])

    #print('Dbase: %f' %prob['comp.Dbase'])
    #print('Lbase: %f' % prob['comp.Lbase'])
    # print ('Max Torque: %f' %prob['comp.Tmax'])
    print('Motor Weight [kg]: %f ' % prob['comp.Mass'])
    print('Motor Size (D^2*L) [mm^3]: %f ' % prob['comp.D2L'])
    print('Motor Size (D^2*L) [ft^3]: %f ' % prob['comp.D2L_ft'])
    print('Motor Size (Volume=pi*radius^2*L) [m^3]: %f' % prob['comp.Volume'])
    print('Motor Size (Volume=pi*radius^2*L) [ft^3]: %f' % prob['comp.Volume_ft'])

