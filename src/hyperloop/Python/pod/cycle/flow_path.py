"""
A group that models an inlet->compressor->duct->nozzle->shaft
using pycycle v2 https://github.com/OpenMDAO/pycycle2.
Calculates flow properties at the front and back of each thermodynamic
element, compressor power required, some geometry, and drag/thrust.
"""
from __future__ import print_function
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

C_IN2toM2 = 144. * (3.28084**2.)
HPtoKW = 0.7457
tubeLen = 563270.0  # // 350 miles in meters
teslaPack = 90.0  # // kw-hours

class FlowPath(Group):
    """
    Params
    ------
    fl_start.P : float
        Tube total pressure
    fl_start.T : float
        Tube total temperature
    fl_start.W : float
        Tube total mass flow
    fl_start.MN_target : float
        Vehicle mach number
    comp.map.PRdes : float
        Pressure ratio of compressor
    nozzle.Ps_exhaust : float
        Exit pressure of nozzle

    Returns
    -------
    comp.torque : float
        Total torque required by motor
    comp.power : float
        Total power required by motor
    comp.Fl_O:stat:area : float
        Area of the duct
    nozzle.Fg : float
        Nozzle thrust
    inlet.F_ram : float
        Ram drag
    nozzle.Fl_O:tot:T : float
        Total temperature at nozzle exit
    nozzle.Fl_O:stat:W : float
        Total mass flow rate at nozzle exit
    FlowPath.inlet.Fl_O:tot:h : float
        Inlet enthalpy of compressor
    FlowPath.comp.Fl_O:tot:h : float
        Exit enthalpy of compressor

    Notes
    -----
    [1] see https://github.com/jcchin/pycycle2/wiki
    """

    def __init__(self):
        super(FlowPath, self).__init__()

        des_vars = (('ram_recovery', 0.99),
                    ('effDes', 0.9),
                    ('duct_MN', 0.65),
                    ('duct_dPqP', 0.),
                    ('nozzle_Cfg', 1.0),
                    ('nozzle_dPqP', 0.),
                    ('shaft_Nmech', 10000.),
                    ('inlet_MN', 0.6),
                    ('comp_MN', 0.65))

        self.add('input_vars',IndepVarComp(des_vars))

        self.add('fl_start', FlowStart(thermo_data=janaf, elements=AIR_MIX))
        # internal flow
        self.add('inlet', Inlet(thermo_data=janaf, elements=AIR_MIX))
        self.add('comp', Compressor(thermo_data=janaf, elements=AIR_MIX))
        self.add('duct', Duct(thermo_data=janaf, elements=AIR_MIX))
        self.add('nozzle', Nozzle(thermo_data=janaf, elements=AIR_MIX))
        self.add('shaft', Shaft(1))

        # connect components
        connect_flow(self, 'fl_start.Fl_O', 'inlet.Fl_I')
        connect_flow(self, 'inlet.Fl_O', 'comp.Fl_I')
        connect_flow(self, 'comp.Fl_O', 'duct.Fl_I')
        connect_flow(self, 'duct.Fl_O', 'nozzle.Fl_I')

        self.connect('input_vars.ram_recovery', 'inlet.ram_recovery')
        self.connect('input_vars.effDes', 'comp.map.effDes')
        self.connect('input_vars.duct_MN', 'duct.MN_target')
        self.connect('input_vars.duct_dPqP', 'duct.dPqP')
        self.connect('input_vars.nozzle_Cfg', 'nozzle.Cfg')
        self.connect('input_vars.nozzle_dPqP', 'nozzle.dPqP')
        self.connect('input_vars.shaft_Nmech', 'shaft.Nmech')
        self.connect('input_vars.inlet_MN', 'inlet.MN_target')
        self.connect('input_vars.comp_MN', 'comp.MN_target')

        self.connect('comp.trq', 'shaft.trq_0')
        self.connect('shaft.Nmech', 'comp.Nmech')

if __name__ == "__main__":

    prob = Problem()
    root = prob.root = Group()

    root.add('FlowPath', FlowPath())

    recorder = SqliteRecorder('FlowPathdb')
    recorder.options['record_params'] = True
    recorder.options['record_metadata'] = True
    prob.driver.add_recorder(recorder)

    params = (('P', .1879, {'units': 'psi'}),
              ('T', 605.06, {'units': 'degR'}),
              ('W', 7.2673, {'units': 'kg/s'}),
              ('vehicleMach', 0.8),
              ('PRdes', 12.5),
              ('PsE', 0.05588, {'units': 'psi'}))

    prob.root.add('des_vars', IndepVarComp(params))

    prob.root.connect('des_vars.P', 'FlowPath.fl_start.P')
    prob.root.connect('des_vars.T', 'FlowPath.fl_start.T')
    prob.root.connect('des_vars.W', 'FlowPath.fl_start.W')
    prob.root.connect('des_vars.vehicleMach', 'FlowPath.fl_start.MN_target')
    prob.root.connect('des_vars.PRdes', 'FlowPath.comp.map.PRdes')
    prob.root.connect('des_vars.PsE', 'FlowPath.nozzle.Ps_exhaust')

    # Make sure balance runs before FlowPath
    #prob.root.set_order(['des_vars', 'balance', 'FlowPath'])
    prob.setup(check=True)
    prob.root.list_connections()

    #prob.print_all_convergence()
    import time
    t = time.time()
    prob.run()

    batteries = (-prob['FlowPath.comp.power'] * HPtoKW * (tubeLen / (
        prob['FlowPath.fl_start.Fl_O:stat:V'] * 0.3048) / 3600.0)) / teslaPack

    astar = np.sqrt(prob['FlowPath.fl_start.Fl_O:stat:gamma'] * R_UNIVERSAL_SI *
                    (cu(prob['FlowPath.fl_start.Fl_O:stat:T'], 'degR', 'degK')))
    ustar = astar * prob['FlowPath.fl_start.Fl_O:stat:MN']
    dstar = prob['FlowPath.fl_start.Fl_O:stat:gamma'] * cu(
        prob['FlowPath.fl_start.Fl_O:stat:P'], 'psi', 'Pa') / astar**2
    mustar = 0.00001716 * (cu(
        prob['FlowPath.fl_start.Fl_O:stat:T'], 'degR', 'degK') / 273.15)**1.5 * (
            273.15 + 110.4) / (cu(prob['FlowPath.fl_start.Fl_O:stat:T'], 'degR',
                                  'degK') + 110.4)  # --- Sutherlands Law
    #Re = dstar*ustar/mustar*Lstar_Lref
    #print (mustar)

    print("")
    print("--- Output ----------------------")

    print("--- Freestream Static Conditions ---")
    print("Pod Mach No.:  %.6f " % (prob['FlowPath.fl_start.Fl_O:stat:MN']))
    print("Ambient Ps:    %.6f psi" % (prob['FlowPath.fl_start.Fl_O:stat:P']))
    print("Ambient Ts:    %.6f R" % (prob['FlowPath.fl_start.Fl_O:stat:T']))
    print("Ambient Pt:    %.6f psi" % (prob['FlowPath.fl_start.Fl_O:tot:P']))
    print("Ambient Tt:    %.6f R" % (prob['FlowPath.fl_start.Fl_O:tot:T']))
    print("Ambient Ds:   %.6f kg/m^3" %
          (cu(prob['FlowPath.fl_start.Fl_O:stat:rho'], 'lbm/ft**3', 'kg/m**3')))
    print("Ambient Viscosity %.8f kg/(m-s)" % (mustar))  #*1.48816394
    print("Pod Velocity:   %.6f m/s" %
          (cu(prob['FlowPath.fl_start.Fl_O:stat:V'], 'ft/s', 'm/s')))
    print("Reynolds No.=  %.6f  -/grid unit" % (
        (cu(prob['FlowPath.fl_start.Fl_O:stat:rho'], 'lbm/ft**3', 'kg/m**3') * cu(
            prob['FlowPath.fl_start.Fl_O:stat:V'], 'ft/s', 'm/s')) / (mustar)))
    print("")

    print("--- Fan Face Conditions ---")
    print("Compressor Mach No.:   %.6f " % (prob['FlowPath.inlet.Fl_O:stat:MN']))
    print("Compressor Area:       %.6f m^2" %
          (cu(prob['FlowPath.inlet.Fl_O:stat:area'], 'inch**2', 'm**2')))
    print("Compressor Radius:     %.6f m" % (np.sqrt(
        (cu(prob['FlowPath.inlet.Fl_O:stat:area'], 'inch**2', 'm**2')) / np.pi)))
    print("Compressor Ps:         %.6f psi" %
          (prob['FlowPath.inlet.Fl_O:stat:P']))
    print("Compressor Ts:         %.6f degR" %
          (prob['FlowPath.inlet.Fl_O:stat:T']))
    print("Compressor Pt:         %.6f psi" % (prob['FlowPath.inlet.Fl_O:tot:P']))
    print("Compressor Tt:         %.6f degR" %
          (prob['FlowPath.inlet.Fl_O:tot:T']))
    print("Compressor MFR:        %.6f kg/s" %
          (cu(prob['FlowPath.inlet.Fl_O:stat:W'], 'lbm/s', 'kg/s')))
    print("Compressor SPR:        %.6f " % (
        prob['FlowPath.inlet.Fl_O:stat:P'] / prob['FlowPath.fl_start.Fl_O:stat:P']))
    print("Compressor Power Reqd: %.6f hp" % (prob['FlowPath.comp.power']))
    print ("Compressor inlet ht:         %.6f Btu/lbm" % (prob['FlowPath.inlet.Fl_O:tot:h']))
    print ("Compressor exit ht:         %.6f Btu/lbm" % (prob['FlowPath.comp.Fl_O:tot:h']))
    print("")

    print ("--- Compressor Exit Conditions ---")
    print ("Compressor Mach No.:   %.6f " % (prob['FlowPath.comp.Fl_O:stat:MN']))
    print ("Compressor Area:       %.6f in^2" % (prob['FlowPath.comp.Fl_O:stat:area']))
    print ("Compressor Radius:     %.6f m" % (np.sqrt((cu(prob['FlowPath.comp.Fl_O:stat:area'], 'inch**2', 'm**2'))/np.pi)))
    print ("Compressor Ps:         %.6f psi" % (prob['FlowPath.comp.Fl_O:stat:P']))
    print ("Compressor Ts:         %.6f degR" % (prob['FlowPath.comp.Fl_O:stat:T']))
    print ("Compressor Pt:         %.6f psi" % (prob['FlowPath.comp.Fl_O:tot:P']))
    print ("Compressor Tt:         %.6f degR" % (prob['FlowPath.comp.Fl_O:tot:T']))
    print ("")

    print("--- Nozzle Plenum Conditions ---")
    print("Nozzle Plenum Area:   %.6f m^2" %
          (cu(prob['FlowPath.duct.Fl_O:stat:area'], 'inch**2', 'm**2')))
    print("Nozzle Plenum Radius: %.6f m  " % (np.sqrt(
        (cu(prob['FlowPath.duct.Fl_O:stat:area'], 'inch**2', 'm**2')) / np.pi)))
    print("Nozzle Plenum Ps:     %.6f psi " % (prob['FlowPath.duct.Fl_O:stat:P']))
    print("Nozzle Plenum Ts:     %.6f degR " %
          (prob['FlowPath.duct.Fl_O:stat:T']))
    print("Nozzle Plenum Pt:     %.6f psi " % (prob['FlowPath.duct.Fl_O:tot:P']))
    print("Nozzle Plenum Tt:     %.6f degR " % (prob['FlowPath.duct.Fl_O:tot:T']))
    print("Nozzle Plenum TPR     %.6f" %
          (prob['FlowPath.duct.Fl_O:tot:P'] / prob['FlowPath.fl_start.Fl_O:stat:P']))
    print("Nozzle Plenum TTR     %.6f" %
          (prob['FlowPath.duct.Fl_O:tot:T'] / prob['FlowPath.fl_start.Fl_O:stat:T']))
    print("")

    print("--- Nozzle Exit Conditions ---")
    print("Mach No.:            %.6f " % (prob['FlowPath.nozzle.Fl_O:stat:MN']))
    print("Nozzle Exit Area:    %.6f m^2" %
          (cu(prob['FlowPath.nozzle.Fl_O:stat:area'], 'inch**2', 'm**2')))
    print("Nozzle Exit Radius:  %.6f m  " % (np.sqrt(
        (cu(prob['FlowPath.nozzle.Fl_O:stat:area'], 'inch**2', 'm**2')) / np.pi)))
    print("Nozzle Exit Ps:      %.6f psi" % (prob['FlowPath.nozzle.Fl_O:stat:P']))
    print("Nozzle Exit Ts:      %.6f degR" %
          (prob['FlowPath.nozzle.Fl_O:stat:T']))
    print("Nozzle Exit Pt:      %.6f psi" % (prob['FlowPath.nozzle.Fl_O:tot:P']))
    print("Nozzle Exit Tt:      %.6f degR" % (prob['FlowPath.nozzle.Fl_O:tot:T']))
    print("Nozzle Exit MFR:     %.6f kg/s" %
          (cu(prob['FlowPath.nozzle.Fl_O:stat:W'], 'lbm/s', 'kg/s')))
    print("Nozzle Thrust:       %.6f lb" % prob['FlowPath.nozzle.Fg'])
    print("Inlet Ram Drag:      %.6f lb" % prob['FlowPath.inlet.F_ram'])
    print("Pod Gross Thrust:    %.6f lb" %
          (prob['FlowPath.nozzle.Fg'] - prob['FlowPath.inlet.F_ram']))

    print("")
    print("--- Force/Power Balances ---")
    print("comp pwr out: ", prob['FlowPath.comp.power'])
    print("comp trq out: ", prob['FlowPath.comp.trq'])
    print("net trq: ", prob['FlowPath.shaft.trq_net'])
    print
    # print 'resid', prob['FlowPath.pwr_balance.pwr_net']
    print("comp.Fl_O:tot:P", prob['FlowPath.comp.Fl_O:tot:P'])
    print("comp.Fl_O:tot:T", prob['FlowPath.comp.Fl_O:tot:T'])
    print("comp.Fl_O:tot:h", prob['FlowPath.comp.Fl_O:tot:h'])

    import sqlitedict
    from pprint import pprint

    db = sqlitedict.SqliteDict('FlowPathdb', 'openmdao')
    data = db['rank0:Driver/1']
    u = data['Unknowns']
    #pprint(u) # print all outputs
    prob.cleanup()
    remove('./FlowPathdb')
    quit()
