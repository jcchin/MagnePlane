import numpy as np
from os import remove

from openmdao.core.group import Group, Component, IndepVarComp
from openmdao.solvers.newton import Newton
from openmdao.api import NLGaussSeidel
from openmdao.solvers.scipy_gmres import ScipyGMRES
from openmdao.units.units import convert_units as cu
from openmdao.api import Problem, LinearGaussSeidel

from pycycle.components import Compressor, Shaft, FlowStart, Inlet, Nozzle, Duct, Splitter, FlightConditions
from pycycle.species_data import janaf
from pycycle.connect_flow import connect_flow
from pycycle.constants import AIR_FUEL_MIX, AIR_MIX
from pycycle.constants import R_UNIVERSAL_ENG, R_UNIVERSAL_SI

from openmdao.solvers.ln_gauss_seidel import LinearGaussSeidel
from openmdao.solvers.ln_direct import DirectSolver
from openmdao.api import SqliteRecorder

C_IN2toM2 = 144.*(3.28084**2.)
HPtoKW = 0.7457
tubeLen = 563270.0  # // 350 miles in meters
teslaPack = 90.0  # // kw-hours


class Balance(Component):

    def __init__(self):
        super(Balance, self).__init__()
        # independents (params)
        # self.add_param('A_in', val=1.3)
        self.add_param('pwr', val=100.)
        self.add_param('Ts_in', val=275.)
        self.add_param('TsTube', val=300.)

        self.add_param('Adiff', val=1.4)
        self.add_param('Acmprssd', val=1.)

        self.add_param('AtubeB', val=1.3)
        self.add_param('AtubeC', val=1.3)
        self.add_param('Abypass', val=1.3)
        #self.add_param('Apod', val=1.3)

        # dependents (states)
        # self.add_state('Apax', val=1.4)
        # self.add_state('pwr', val=200.)
        self.add_state('Pt', val=5.566211746)
        self.add_state('Tt', val=441.3225037)
        self.add_state('W', val=298.16)
        self.add_state('BPR', val=1.4)


    def solve_nonlinear(self, params, unknowns, resids):
        pass

    def apply_nonlinear(self, p, u, resids):

        resids['Pt'] = p['pwr'] + 3000.  # pwr = -3000
        resids['Tt'] = p['Ts_in'] - p['TsTube']
        resids['W'] = (p['Adiff']-p['Acmprssd']) - 1.4  # Apax = 1.4
        resids['BPR'] = (p['AtubeB']+p['AtubeC']) - (p['Abypass'] + p['Adiff'])

        print "Pt ", u['Pt']
        print "Tt ", u['Tt']
        print "W ", u['W']
        print "BPR ", u['BPR']


    def apply_linear(self, params, unknowns, dparams, dunknowns, dresids, mode):
        pass



class CompressionCycle(Group):
    """A group that models an inlet->compressor->duct->nozzle->shaft"""

    def __init__(self):
        super(CompressionCycle, self).__init__()

        # initiate components
        #self.add('fc', FlightConditions())
        self.add('fl_start', FlowStart(thermo_data=janaf, elements=AIR_MIX))
        self.add('inlet', Inlet(thermo_data=janaf, elements=AIR_MIX))
        self.add('splitter', Splitter(thermo_data=janaf, elements=AIR_MIX))
        self.add('comp', Compressor(thermo_data=janaf, elements=AIR_MIX))
        self.add('duct', Duct(thermo_data=janaf, elements=AIR_MIX))
        self.add('nozzle', Nozzle(thermo_data=janaf, elements=AIR_MIX))
        self.add('shaft', Shaft(1))

        self.add('bypass', Duct(thermo_data=janaf, elements=AIR_MIX))

        # connect components
        connect_flow(self,'fl_start.Fl_O', 'inlet.Fl_I')
        connect_flow(self,'inlet.Fl_O', 'splitter.Fl_I')
        connect_flow(self, 'splitter.Fl_O1', 'comp.Fl_I')
        connect_flow(self,'comp.Fl_O', 'duct.Fl_I')
        connect_flow(self,'duct.Fl_O', 'nozzle.Fl_I')

        connect_flow(self, 'splitter.Fl_O2', 'bypass.Fl_I')

        self.connect('comp.trq', 'shaft.trq_0')
        self.connect('shaft.Nmech', 'comp.Nmech')

        # self.nl_solver = Newton()
        # self.nl_solver.options['rtol'] = 1.4e-8
        # self.nl_solver.options['maxiter'] = 75
        # self.nl_solver.options['iprint'] = 1

class Sim(Group):
    """drive indep/dep pairs"""

    def __init__(self):
        super(Sim, self).__init__()


        conv = self.add("conv", Group())
        conv.add('cycle', CompressionCycle())
        conv.add('balance', Balance(), promotes=['TsTube'])

        self.connect('conv.cycle.fl_start.Fl_O:stat:T','conv.balance.Ts_in')
        self.connect('conv.cycle.splitter.Fl_O1:stat:area','conv.balance.AtubeB')
        self.connect('conv.cycle.splitter.Fl_O2:stat:area','conv.balance.AtubeC')
        self.connect('conv.cycle.bypass.Fl_O:stat:area','conv.balance.Abypass')
        self.connect('conv.cycle.inlet.Fl_O:stat:area','conv.balance.Adiff')
        self.connect('conv.cycle.duct.Fl_O:stat:area','conv.balance.Acmprssd')
        self.connect('conv.cycle.comp.power','conv.balance.pwr')

        self.connect('conv.balance.Pt','conv.cycle.fl_start.P')
        self.connect('conv.balance.Tt','conv.cycle.fl_start.T')
        self.connect('conv.balance.W','conv.cycle.fl_start.W')
        self.connect('conv.balance.BPR','conv.cycle.splitter.BPR')

        self.nl_solver = NLGaussSeidel()
        self.nl_solver.options['atol'] = 1e-5
        self.nl_solver.options['iprint'] = 1
        self.nl_solver.options['rtol'] = 1e-5
        self.nl_solver.options['maxiter'] = 50

        self.ln_solver = LinearGaussSeidel()
        self.ln_solver.options['atol'] = 1e-6
        self.ln_solver.options['maxiter'] = 100
        #self.ln_solver.options['restart'] = 100

        conv.ln_solver = ScipyGMRES()
        conv.ln_solver.options['atol'] = 1e-6
        conv.ln_solver.options['maxiter'] = 100
        conv.ln_solver.options['restart'] = 100

if __name__ == "__main__":

    prob = Problem()
    prob.root = Sim()

    recorder = SqliteRecorder('cycle')
    recorder.options['record_params'] = True
    recorder.options['record_metadata'] = True
    prob.driver.add_recorder(recorder)

    params = (
        ('vehicleMach', 0.6),
        ('inlet_MN', 0.6),
        #('P', 5.566211746, {'units':'psi'}),
        #('T', 441.3225037, {'units':'degR'}),
        #('W', 298.16, {'units':'kg/s'}),
        ('PsE', 14.7, {'units':'psi'}),
        ('cmpMach', 0.6),
        ('PsTube', 0.507313), # // 75000 ft std alt pressure
        ('TsTube', 524.0),
        #('Atube, Dtube, AtubeB, AtubeC'),
        #('Apod, Abypass, blockage', 0.90),
        #('Apax, Adiff, Acmprssd'),
        #('g')
    )

    prob.root.add('des_vars', IndepVarComp(params))
    #prob.root.connect('des_vars.P', 'fl_start.P')
    #prob.root.connect('des_vars.T', 'fl_start.T')
    #prob.root.connect('des_vars.W', 'fl_start.W')
    # prob.root.connect('des_vars.alt', 'fc.alt')
    # prob.root.connect('des_vars.W', 'fc.fs.W')
    prob.root.connect('des_vars.vehicleMach', 'conv.cycle.fl_start.MN_target')
    prob.root.connect('des_vars.inlet_MN', 'conv.cycle.inlet.MN_target')
    # prob.root.connect('fc.ambient.Ps', 'nozzle.Ps_exhaust')
    prob.root.connect('des_vars.PsE', 'conv.cycle.nozzle.Ps_exhaust')

    prob.setup(check=True)
    prob.root.list_connections()

    # Flight Conditions
    prob['conv.balance.W'] = 298.1585042  # 550 lbm/s

    # Inlet Conditions
    prob['conv.cycle.inlet.ram_recovery'] = 0.99
    if prob['des_vars.inlet_MN'] > prob['des_vars.vehicleMach']:
        prob['des_vars.inlet_MN'] = prob['des_vars.vehicleMach']

    # Splitter Conditions
    #prob['conv.cycle.splitter.BPR'] = 0.0001
    prob['conv.cycle.splitter.MN_target1'] = prob['des_vars.inlet_MN']
    prob['conv.cycle.splitter.MN_target2'] = prob['des_vars.inlet_MN']

    # Compressor Conditions
    prob['conv.cycle.comp.map.PRdes'] = 1.5
    prob['conv.cycle.comp.map.effDes'] = 1.0
    prob['conv.cycle.comp.MN_target'] = 0.6

    # Duct
    prob['conv.cycle.duct.MN_target'] = 0.6
    prob['conv.cycle.duct.dPqP'] = 0.

    # Nozzle Conditions
    prob['conv.cycle.nozzle.Cfg'] = 1.0
    prob['conv.cycle.nozzle.dPqP'] = 0.

    prob['des_vars.PsE'] = 2.7

    # Shaft
    prob['conv.cycle.shaft.Nmech'] = 10000.

    #prob.root.duct.list_connections()

    import time
    t = time.time()
    prob.run()
    print time.time() - t

    #Atube = prob['inlet.Fl_O:stat:area']/(3.28084**2.)/(144.)
    #AtubeC = prob['splitter.Fl_O1:stat:area']/(3.28084**2.)/(144.)
    #AtubeB = prob['splitter.Fl_O2:stat:area']/(3.28084**2.)/(144.)
    batteries = (-prob['conv.cycle.comp.power']*HPtoKW*(tubeLen/(prob['conv.cycle.fl_start.Fl_O:stat:V']*0.3048)/3600.0))/teslaPack;


    astar = np.sqrt(prob['conv.cycle.fl_start.Fl_O:stat:gamma']*R_UNIVERSAL_SI*(cu(prob['conv.cycle.fl_start.Fl_O:stat:T'], 'degR', 'degK')))
    ustar = astar*prob['conv.cycle.fl_start.Fl_O:stat:MN']
    dstar = prob['conv.cycle.fl_start.Fl_O:stat:gamma']*cu(prob['conv.cycle.fl_start.Fl_O:stat:P'],'psi','Pa')/astar**2
    mustar = 0.00001716*(cu(prob['conv.cycle.fl_start.Fl_O:stat:T'], 'degR', 'degK')/273.15)**1.5*(273.15+110.4)/(cu(prob['conv.cycle.fl_start.Fl_O:stat:T'], 'degR', 'degK')+110.4) # --- Sutherlands Law
    #Re = dstar*ustar/mustar*Lstar_Lref
    print mustar

    print ""
    print "--- Output ----------------------"

    print "--- Freestream Static Conditions ---"
    print "Mach No.:    %.6f " % (prob['conv.cycle.fl_start.Fl_O:stat:MN'])
    print "Ambient Ps:  %.6f psi" % (prob['conv.cycle.fl_start.Fl_O:stat:P'])
    print "Ambient Ts:  %.6f R" % (prob['conv.cycle.fl_start.Fl_O:stat:T'])
    print "Ambient Pt:  %.6f psi" % (prob['conv.cycle.fl_start.Fl_O:tot:P'])
    print "Ambient Tt:  %.6f R" % (prob['conv.cycle.fl_start.Fl_O:tot:T'])
    print "Ambient Rho: %.6f kg/m^3" % (cu(prob['conv.cycle.fl_start.Fl_O:stat:rho'], 'lbm/ft**3', 'kg/m**3'))
    print "Ambient Viscosity %.8f kg/(m-s)" % (mustar) #*1.48816394
    print "Pod Velocity:   %.6f m/s" % (cu(prob['conv.cycle.fl_start.Fl_O:stat:V'], 'ft/s', 'm/s'))
    print "Reynolds No.=  %.6f  -/grid unit" % ((cu(prob['conv.cycle.fl_start.Fl_O:stat:rho'],'lbm/ft**3','kg/m**3')*cu(prob['conv.cycle.fl_start.Fl_O:stat:V'],'ft/s','m/s'))/(mustar))
    print ""

    print "--- Fan Conditions ---"
    print "Compressor Mach No.:   %.6f " % (prob['conv.cycle.inlet.Fl_O:stat:MN'])
    print "Compressor Area:       %.6f m^2" % (cu(prob['conv.cycle.inlet.Fl_O:stat:area'], 'inch**2', 'm**2'))
    print "Compressor Radius:     %.6f m" % (np.sqrt((cu(prob['conv.cycle.inlet.Fl_O:stat:area'], 'inch**2', 'm**2'))/np.pi))
    print "Compressor Ps:         %.6f psi" % (prob['conv.cycle.inlet.Fl_O:stat:P'])
    print "Compressor Ts:         %.6f degR" % (prob['conv.cycle.inlet.Fl_O:stat:T'])
    print "Compressor Pt:         %.6f psi" % (prob['conv.cycle.inlet.Fl_O:tot:P'])
    print "Compressor Tt:         %.6f degR" % (prob['conv.cycle.inlet.Fl_O:tot:T'])
    print "Compressor MFR:        %.6f kg/s" % (cu(prob['conv.cycle.inlet.Fl_O:stat:W'], 'lbm/s', 'kg/s'))
    print "Compressor SPR:        %.6f " % (prob['conv.cycle.inlet.Fl_O:stat:P']/prob['conv.cycle.fl_start.Fl_O:stat:P'])
    print "Compressor Power Reqd: %.6f hp" % (prob['conv.cycle.comp.power'])
    print ""

    print "--- Nozzle Plenum Conditions ---"
    print "Nozzle Plenum Area:   %.6f m^2" % (cu(prob['conv.cycle.duct.Fl_O:stat:area'], 'inch**2', 'm**2'))
    print "Nozzle Plenum Radius: %.6f m  " % (np.sqrt((cu(prob['conv.cycle.duct.Fl_O:stat:area'], 'inch**2', 'm**2'))/np.pi))
    print "Nozzle Plenum Ps:     %.6f psi " % (prob['conv.cycle.duct.Fl_O:stat:P'])
    print "Nozzle Plenum Ts:     %.6f degR " % (prob['conv.cycle.duct.Fl_O:stat:T'])
    print "Nozzle Plenum Pt:     %.6f psi " % (prob['conv.cycle.duct.Fl_O:tot:P'])
    print "Nozzle Plenum Tt:     %.6f degR " % (prob['conv.cycle.duct.Fl_O:tot:T'])
    print "Nozzle Plenum TPR     %.6f" % (prob['conv.cycle.duct.Fl_O:tot:P']/prob['conv.cycle.fl_start.Fl_O:stat:P'])
    print "Nozzle Plenum TTR     %.6f" % (prob['conv.cycle.duct.Fl_O:tot:T']/prob['conv.cycle.fl_start.Fl_O:stat:T'])
    print ""

    print "--- Nozzle Exit Conditions ---"
    print "Mach No.:            %.6f " % (prob['conv.cycle.nozzle.Fl_O:stat:MN'])
    print "Nozzle Exit Area:    %.6f m^2" % (cu(prob['conv.cycle.nozzle.Fl_O:stat:area'], 'inch**2', 'm**2'))
    print "Nozzle Exit Radius:  %.6f m  " % (np.sqrt((cu(prob['conv.cycle.nozzle.Fl_O:stat:area'], 'inch**2', 'm**2'))/np.pi))
    print "Nozzle Exit Ps:      %.6f psi" % (prob['conv.cycle.nozzle.Fl_O:stat:P'])
    print "Nozzle Exit Ts:      %.6f degR" % (prob['conv.cycle.nozzle.Fl_O:stat:T'])
    print "Nozzle Exit Pt:      %.6f psi" % (prob['conv.cycle.nozzle.Fl_O:tot:P'])
    print "Nozzle Exit Tt:      %.6f degR" % (prob['conv.cycle.nozzle.Fl_O:tot:T'])
    print "Nozzle Exit MFR:     %.6f kg/s" % (cu(prob['conv.cycle.nozzle.Fl_O:stat:W'], 'lbm/s', 'kg/s'))
    print "Nozzle Gross Thrust: %.6f lb" % prob['conv.cycle.nozzle.Fg']
    print "Inlet Ram Drag:      %.6f lb" % prob['conv.cycle.inlet.F_ram']

    print ""
    print "--- Force/Power Balances ---"
    print "comp pwr out: ", prob['conv.cycle.comp.power']
    print "comp trq out: ", prob['conv.cycle.comp.trq']
    print "net trq: ", prob['conv.cycle.shaft.trq_net']
    print
    # print 'resid', prob['conv.cycle.pwr_balance.pwr_net']
    print "comp.Fl_O:tot:P", prob['conv.cycle.comp.Fl_O:tot:P']
    print "comp.Fl_O:tot:T", prob['conv.cycle.comp.Fl_O:tot:T']
    print "comp.Fl_O:tot:h", prob['conv.cycle.comp.Fl_O:tot:h']

    print "BPR ", prob['conv.cycle.splitter.BPR']
    print "AtubeB ", prob['conv.balance.AtubeB']
    print "AtubeC ", prob['conv.balance.AtubeC']
    print "Apax ", prob['conv.balance.Adiff']-prob['conv.balance.Acmprssd']
    import sqlitedict
    from pprint import pprint

    db = sqlitedict.SqliteDict('cycle', 'openmdao' )
    data = db['Driver/1']
    u = data['Unknowns']
    #pprint(u)
    prob.cleanup()
    remove('./cycle')
