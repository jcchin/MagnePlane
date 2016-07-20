"""
Group for the Compressor Cycle Components. This group contains the following components:
Flowpath, Compressor Mass, and Compressor Length.
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
    """
	Params
    ------
    A_inlet : float
        Inlet area of pod. (m**2)
    M_pod : float
        Pod Mach Number. (unitless)
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
    comp_len : float
        Length of Compressor (m)
    comp_mass : float
        Compressor Mass (kg)
    comp.trq : float
        Torque required by compressor motor(lb*ft)
	comp.power : float
        Torque required by compressor motor(hp)
    comp.nozzle.Fg : float
        Gross Thrust (lb)
    comp.F_ram : float
        Ram Drag (lb)
    nozzle.Fl_O:tot:T : float
        Nozzle Exit Tt (degR)
    nozzle.Fl_O:stat:W : float
        Nozzle Exit MFR (kg/s)
	
    References
    ----------
    .. [1] Miceahal Tong Correlation used.
	.. [2] NASA-Glenn NPSS compressor cycle model.
    """
    def __init__(self):
        super(Cycle, self).__init__()

        self.add('FlowPath', FlowPath(), promotes=['comp.trq', 'comp.power', 'nozzle.Fg', 'inlet.F_ram', 'nozzle.Fl_O:tot:T',
                                                    'nozzle.Fl_O:stat:W', 'fl_start.MN_target'])
        self.add('CompressorMass', CompressorMass(), promotes=['comp_mass'])
        self.add('CompressorLen', CompressorLen(), promotes=['A_inlet', 'M_pod', 'T_tunnel', 'p_tunnel', 'comp_len'])
        
        self.connect('FlowPath.inlet.Fl_O:tot:h', ['CompressorMass.h_in', 'CompressorLen.h_in'])
        self.connect('FlowPath.comp.Fl_O:tot:h', ['CompressorMass.h_out', 'CompressorLen.h_out'])
        self.connect('FlowPath.inlet.Fl_O:stat:area', ['CompressorMass.comp_inletArea', 'CompressorLen.comp_inletArea'])
        self.connect('FlowPath.inlet.Fl_O:tot:T', 'CompressorLen.comp_inletTemp')
        self.connect('FlowPath.inlet.Fl_O:stat:W', 'CompressorMass.mass_flow')

if __name__ == "__main__":
    prob = Problem()
    root = prob.root = Group()

    root.add('Cycle', Cycle())

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

    prob.run()

    print('Comp_Mass %f' % prob['Cycle.comp_mass'])
    print('Comp_len %f' % prob['Cycle.comp_len'])
    print('H_int %f' % prob['Cycle.FlowPath.inlet.Fl_O:tot:h'])
    print('H_out %f' % prob['Cycle.FlowPath.comp.Fl_O:tot:h'])
    print("Compressor Area:       %.6f m^2" %
          (cu(prob['Cycle.FlowPath.inlet.Fl_O:stat:area'], 'inch**2', 'm**2')))
    print("Compressor Tt:         %.6f Btu/lbm" %
          (prob['Cycle.FlowPath.inlet.Fl_O:tot:T']))
