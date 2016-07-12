"""
Group for the Cycle containing the following components:
FlowPath and Compressor Mass
"""
from __future__ import print_function
from openmdao.api import IndepVarComp, Component, Problem, Group
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

from hyperloop.Python.pod.cycle.flow_path import FlowPath
from hyperloop.Python.pod.cycle.compressor_mass import CompressorMass
from hyperloop.Python.pod.cycle.comp_len import CompressorLen

class Cycle(Group):
  def __init__(self):
    super(Cycle, self).__init__()

    self.add('FlowPath', FlowPath(), promotes=['comp.trq', 'comp.power', 'comp.Nmech', 'inlet.Fl_O:stat:area', 'nozzle.Fg', 
                                              'inlet.F_ram', 'nozzle.Fl_O:tot:T', 'nozzle.Fl_O:stat:W', 'fl_start.Fl_O:stat:P',
                                              'fl_start.Fl_O:stat:T', 'fl_start.Fl_O:tot:P', 'fl_start.Fl_O:tot:T',
                                              'fl_start.Fl_O:stat:rho', 'fl_start.Fl_O:stat:V', 'inlet.Fl_O:stat:MN',
                                              'inlet.Fl_O:stat:P', 'inlet.Fl_O:stat:T', 'inlet.Fl_O:tot:P', 'inlet.Fl_O:tot:T',
                                              'inlet.Fl_O:stat:W', 'comp.Fl_O:stat:MN', 'comp.Fl_O:stat:area',
                                              'comp.Fl_O:stat:P', 'comp.Fl_O:stat:T'])
    self.add('CompressorMass', CompressorMass(), promotes=['comp_mass'])
    self.add('CompressorLen', CompressorLen(), promotes=['A_inlet', 'm_pod', 'T_tunnel', 'p_tunnel', 'comp_len'])
    
    self.connect('FlowPath.inlet.Fl_O:tot:h', ['CompressorMass.h_in', 'CompressorLen.h_in'])
    self.connect('FlowPath.comp.Fl_O:tot:h', ['CompressorMass.h_out', 'CompressorLen.h_out'])
    self.connect('inlet.Fl_O:stat:area', ['CompressorMass.comp_inletArea', 'CompressorLen.comp_inletArea'])
    self.connect('inlet.Fl_O:tot:T', 'CompressorLen.comp_inletTemp')
    self.connect('inlet.Fl_O:stat:W', 'CompressorMass.mass_flow')

if __name__ == "__main__":
    prob = Problem()
    root = prob.root = Group()

    root.add('Cycle', Cycle())

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
    prob.root.connect('des_vars.PsE', 'Cycle.FlowPath.nozzle.Ps_exhaust')
    prob.root.connect('des_vars.P', 'Cycle.FlowPath.fl_start.P')
    prob.root.connect('des_vars.T', 'Cycle.FlowPath.fl_start.T')
    prob.root.connect('des_vars.W', 'Cycle.FlowPath.fl_start.W')

    prob.root.connect('des_vars.vehicleMach', 'Cycle.FlowPath.fl_start.MN_target')
    prob.root.connect('des_vars.inlet_MN', 'Cycle.FlowPath.inlet.MN_target')

    prob.setup()
    prob.root.list_connections()

    # Inlet Conditions

    prob['Cycle.FlowPath.inlet.ram_recovery'] = 0.99
    if prob['des_vars.inlet_MN'] > prob['des_vars.vehicleMach']:
        prob['des_vars.inlet_MN'] = prob['des_vars.vehicleMach']

    # Compressor Conditions
    prob['Cycle.FlowPath.comp.map.PRdes'] = 6.0
    prob['Cycle.FlowPath.comp.map.effDes'] = 0.9
    prob['Cycle.FlowPath.comp.MN_target'] = 0.65

    # Duct
    prob['Cycle.FlowPath.duct.MN_target'] = 0.65
    prob['Cycle.FlowPath.duct.dPqP'] = 0.

    # Nozzle Conditions
    prob['Cycle.FlowPath.nozzle.Cfg'] = 1.0
    prob['Cycle.FlowPath.nozzle.dPqP'] = 0.

    # Shaft
    prob['Cycle.FlowPath.shaft.Nmech'] = 10000.

    #prob.print_all_convergence()
    import time
    t = time.time()

    prob.run()

    print('Comp_Mass %f' % prob['Cycle.comp_mass'])
    print('Comp_len %f' % prob['Cycle.comp_len'])
    print('H_int %f' % prob['Cycle.FlowPath.inlet.Fl_O:tot:h'])
    print('H_out %f' % prob['Cycle.FlowPath.comp.Fl_O:tot:h'])
    print("Compressor Area:       %.6f m^2" %
          (cu(prob['Cycle.inlet.Fl_O:stat:area'], 'inch**2', 'm**2')))
    print("Compressor Tt:         %.6f degR" %
          (prob['Cycle.inlet.Fl_O:tot:T']))
