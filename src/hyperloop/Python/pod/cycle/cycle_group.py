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
from hyperloop.Python.pod.cycle.flow_path_inputs import FlowPathInputs

class Cycle(Group):
    """
	Params
    ------
    pod_mach_number : float
        Vehicle mach number (unitless)
    tube_pressure : float
        Tube total pressure (Pa)
    tube_temp : float
        Tube total temperature (K)
    comp_inlet_area : float
        Inlet area of compressor. (m**2)
    comp.map.PRdes : float
        Pressure ratio of compressor (unitless)
    nozzle.Ps_exhaust : float
        Exit pressure of nozzle (Pa)
    
    Returns
    -------
    comp_len : float
        Length of Compressor (m)
    comp_mass : float
        Mass of compressor (kg)
    comp.trq : float
        Total torque required by motor (ft*lbf)
    comp.power : float
        Total power required by motor (hp)
    comp.Fl_O:stat:area : float
        Area of the duct (in**2)
    nozzle.Fg : float
        Nozzle thrust (lbf)
    inlet.F_ram : float
        Ram drag (lbf)
    nozzle.Fl_O:tot:T : float
        Total temperature at nozzle exit (degR)
    nozzle.Fl_O:stat:W : float
        Total mass flow rate at nozzle exit (lbm/s)
	
    References
    ----------
    .. [1] Miceahal Tong Correlation used.
	.. [2] NASA-Glenn NPSS compressor cycle model.
    """

    def __init__(self):
        super(Cycle, self).__init__()

        self.add('CompressorLen', CompressorLen(), promotes=['comp_len'])
        self.add('CompressorMass', CompressorMass(), promotes=['comp_mass'])
        self.add('FlowPathInputs', FlowPathInputs(), promotes=['pod_mach', 'tube_pressure', 'tube_temp', 'comp_inlet_area'])
        self.add('FlowPath', FlowPath(), promotes=['comp.trq', 'comp.power', 'nozzle.Fg', 'inlet.F_ram',
                                                    'nozzle.Fl_O:stat:W', 'comp.Fl_O:stat:area', 'comp.map.PRdes', 
                                                    'nozzle.Ps_exhaust', 'nozzle.Fl_O:tot:T'])
        
        # Connects cycle group level variables to downstream components
        self.connect('pod_mach', 'FlowPath.fl_start.MN_target')
        self.connect('comp_inlet_area', ['CompressorLen.comp_inletArea', 'CompressorMass.comp_inletArea'])

        # Connects FlowPathInputs outputs to downstream components
        self.connect('FlowPathInputs.Pt', 'FlowPath.fl_start.P')
        self.connect('FlowPathInputs.Tt', 'FlowPath.fl_start.T')
        self.connect('FlowPathInputs.m_dot', 'FlowPath.fl_start.W')

        # Connects FlowPath outputs to downstream components
        self.connect('nozzle.Fl_O:stat:W', 'CompressorMass.mass_flow')
        self.connect('FlowPath.inlet.Fl_O:tot:h', ['CompressorMass.h_in', 'CompressorLen.h_in'])
        self.connect('FlowPath.comp.Fl_O:tot:h', ['CompressorMass.h_out', 'CompressorLen.h_out'])

if __name__ == "__main__":
    
    prob = Problem()
    root = prob.root = Group()

    root.add('Cycle', Cycle())

    params = (('comp_PR', 6.0, {'units': 'unitless'}),
              ('PsE', 0.05588, {'units': 'psi'}),
              ('pod_mach_number', .8, {'units': 'unitless'}),
              ('tube_pressure', 850., {'units': 'Pa'}),
              ('tube_temp', 320., {'units': 'K'}),
              ('comp_inlet_area', 2.3884, {'units': 'm**2'}))

    prob.root.add('des_vars', IndepVarComp(params))

    prob.root.connect('des_vars.comp_PR', 'Cycle.comp.map.PRdes')
    prob.root.connect('des_vars.PsE', 'Cycle.nozzle.Ps_exhaust')
    prob.root.connect('des_vars.pod_mach_number', 'Cycle.pod_mach')
    prob.root.connect('des_vars.tube_pressure', 'Cycle.tube_pressure')
    prob.root.connect('des_vars.tube_temp', 'Cycle.tube_temp')
    prob.root.connect('des_vars.comp_inlet_area', 'Cycle.comp_inlet_area')

    prob.setup()
    # prob.root.list_connections()
    #print(prob.root.Cycle.list_order())
    #from openmdao.api import view_tree
    #view_tree(prob)
    #exit()

    prob.run()

    print('Pt               %f' % prob['Cycle.FlowPathInputs.Pt'])
    print('Tt               %f' % prob['Cycle.FlowPathInputs.Tt'])
    print('m_dot            %f' % prob['Cycle.FlowPathInputs.m_dot'])

    print('H in             %f' % prob['Cycle.FlowPath.inlet.Fl_O:tot:h'])
    print('H out            %f' % prob['Cycle.FlowPath.comp.Fl_O:tot:h'])

    print('Comp_len         %f m' % prob['Cycle.comp_len'])
    print('Comp_Mass        %f kg' % prob['Cycle.comp_mass'])

    print('Torque           %f N*m' % (cu(prob['Cycle.comp.trq'], 'ft*lbf', 'N*m')))
    print('Power            %f W' % (cu(prob['Cycle.comp.power'], 'hp', 'W')))

    print('A_duct           %f m**2' % (cu(prob['Cycle.comp.Fl_O:stat:area'], 'inch**2', 'm**2')))
    print('nozzle.Fg        %f N' % (cu(prob['Cycle.nozzle.Fg'], 'lbf', 'N')))
    print('inlet.F_ram      %f N' % (cu(prob['Cycle.inlet.F_ram'], 'lbf', 'N')))

    print('Nozzle exit temp %f K' % (cu(prob['Cycle.nozzle.Fl_O:tot:T'], 'degR', 'K')))
    print('Nozzle exit MFR  %f kg/s' % (cu(prob['Cycle.nozzle.Fl_O:stat:W'], 'lbm/s', 'kg/s')))
