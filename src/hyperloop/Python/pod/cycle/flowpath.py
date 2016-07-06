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
    ram_recovery : float
        Perfcentage of ram pressure recovered (1-ram_recovery) is lost
    inlet_MN : float
        Mach Number at the front face of the inlet
    comp.PRdes : float
        Pressure Ratio used to "design" and size the compressor
    comp.effDes : float
        Target Efficiency of the "design" compressor
    comp.MN_target : float
        Mach Number at the front face of the compressor
    duct.dPqP : float
        Pressure loss across a duct
    duct.MN_target : float
        Mach Number at the front face of the duct
    nozzle.Cfg : float
        Gross Thrust Performance Coefficient
    nozzle.dPqP : float
        Pressure loss in the nozzle
    shaft.Nmech : float
        Mechanical RPM of the shaft (connected to compressor and motor)

    Returns
    -------
    Freestream Conditions : float
        Required as boundary conditions for CFD
    Fan Face Conditions : float
        Required as boundary conditions for CFD
    Nozzle Plenum Conditions : float
        Required as boundary conditions for CFD
    Nozzle Exit Conditions : float
        Required as boundary conditions for CFD.
        Also includes Thrust, Inlet Ram Drag, and Pod Gross Thrust
    comp pwr out : float
        Power required by the compressor
    comp trq out : float
        Torque required by compressor motor

    Notes
    -----
    [1] see https://github.com/jcchin/pycycle2/wiki
    """

    def __init__(self):
        super(FlowPath, self).__init__()

        # initiate components
        #self.add('fc', FlightConditions())
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

    params = (('vehicleMach', 0.8),
              ('inlet_MN', 0.65),
              ('P', 0.1885057735, {'units': 'psi'}),
              ('T', 591.0961831, {'units': 'degR'}),
              ('W', 4.53592, {'units': 'kg/s'}),
              ('PsE', 0.59344451, {'units': 'psi'}),
              ('cmpMach', 0.65), )

    prob.root.add('des_vars', IndepVarComp(params))
    prob.root.connect('des_vars.P', 'FlowPath.fl_start.P')
    prob.root.connect('des_vars.T', 'FlowPath.fl_start.T')
    prob.root.connect('des_vars.W', 'FlowPath.fl_start.W')

    prob.root.connect('des_vars.vehicleMach', 'FlowPath.fl_start.MN_target')
    prob.root.connect('des_vars.inlet_MN', 'FlowPath.inlet.MN_target')

    prob.root.connect('des_vars.PsE', 'FlowPath.nozzle.Ps_exhaust')

    # Make sure balance runs before FlowPath
    #prob.root.set_order(['des_vars', 'balance', 'FlowPath'])
    prob.setup(check=False)
    #prob.root.list_connections()

    # Flight Conditions

    # Inlet Conditions
    prob['FlowPath.inlet.ram_recovery'] = 0.99
    if prob['des_vars.inlet_MN'] > prob['des_vars.vehicleMach']:
        prob['des_vars.inlet_MN'] = prob['des_vars.vehicleMach']

    # Compressor Conditions
    prob['FlowPath.comp.map.PRdes'] = 6.0
    prob['FlowPath.comp.map.effDes'] = 0.9
    prob['FlowPath.comp.MN_target'] = 0.65

    # Duct
    prob['FlowPath.duct.MN_target'] = 0.65
    prob['FlowPath.duct.dPqP'] = 0.

    # Nozzle Conditions
    prob['FlowPath.nozzle.Cfg'] = 1.0
    prob['FlowPath.nozzle.dPqP'] = 0.

    # Shaft
    prob['FlowPath.shaft.Nmech'] = 10000.

    #prob.print_all_convergence()
    import time
    t = time.time()
    prob.run()
    #print (time.time() - t)
    #inputs = ['balance.Pt', 'balance.Tt', 'balance.W', 'balance.BPR']
    #prob.check_total_derivatives()

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
    print("")

    # print ("--- Compressor Exit Conditions ---")
    # print ("Compressor Mach No.:   %.6f " % (prob['FlowPath.comp.Fl_O:stat:MN']))
    # print ("Compressor Area:       %.6f in^2" % (prob['FlowPath.comp.Fl_O:stat:area']))
    # print ("Compressor Radius:     %.6f m" % (np.sqrt((cu(prob['FlowPath.comp.Fl_O:stat:area'], 'inch**2', 'm**2'))/np.pi)))
    # print ("Compressor Ps:         %.6f psi" % (prob['FlowPath.comp.Fl_O:stat:P']))
    # print ("Compressor Ts:         %.6f degR" % (prob['FlowPath.comp.Fl_O:stat:T']))
    # print ("Compressor Pt:         %.6f psi" % (prob['FlowPath.comp.Fl_O:tot:P']))
    # print ("Compressor Tt:         %.6f degR" % (prob['FlowPath.comp.Fl_O:tot:T']))
    # print ("")

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

    print ("Compressor inlet ht:         %.6f degR" % (prob['FlowPath.inlet.Fl_O:tot:h']))
    print ("Compressor exit ht:         %.6f degR" % (prob['FlowPath.comp.Fl_O:tot:h']))

    import sqlitedict
    from pprint import pprint

    db = sqlitedict.SqliteDict('FlowPathdb', 'openmdao')
    data = db['rank0:Driver/1']
    u = data['Unknowns']
    #pprint(u) # print all outputs
    prob.cleanup()
    remove('./FlowPathdb')
    quit()
