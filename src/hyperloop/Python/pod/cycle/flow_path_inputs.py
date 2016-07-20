"""
A group that converts user inputs into format flow_path needs
"""
from __future__ import print_function
import numpy as np

from openmdao.api import Group, Component, IndepVarComp, Problem

class FlowPathInputs(Component):
    """
	Params
	------
	tube_pressure : float
		Ambient temperature inside tube (K)
	pod_mach : float
		Pod mach number (unitless)
	comp_inlet_area : float
		Compressor inlet area (m**2)
	tube_temp : float
		Ambient temperature inside tube (K)
	gamma : float
		Specific heat ratio
	comp_mach : float
		Compressor mach number (unitless)
	R : float
		Ideal gas constant (J/(kg*K))
	eta : float
		Efficiency of inlet (unitless)

	Returns
	-------
	Pt : float
		Total pressure inside tube (Pa)
	Tt : float
		Total temperature inside tube (K)
	m_dot : float
		Mass flow rate (kg/s)
	"""

    def __init__(self):
        super(FlowPathInputs, self).__init__()

        self.add_param('tube_pressure',
                        val=850.,
                        desc='ambient pressure inside tube',
                        units='Pa')
        self.add_param('pod_mach',
                        val=.8,
                        desc='pod mach speed',
                        units='unitless')
        self.add_param('comp_inlet_area',
                         val=3.8,
                         desc='compressor inlet area',
                         units='m**2')
        self.add_param('tube_temp',
                         val=320.,
                         desc='ambient temp inside tube',
                         units='K')
        self.add_param('gamma',
                         val=1.4,
                         desc='specific heat ratio',
                         units='unitless')
        self.add_param('comp_mach',
                         val=.6,
                         desc='compressor mach number',
                         units='unitless')
        self.add_param('R',
                         val=287.,
                         desc='ideal gas constant',
                         units='J/(kg*K)')
        self.add_param('eta',
                         val=.99,
                         desc='efficiency of inlet',
                         units='unitless')

        self.add_output('Pt',
                         val=0.,
                         desc='total pressure inside tube',
                         units='Pa')
        self.add_output('Tt',
                         val=0.,
                         desc='total temp inside tube',
                         units='K')
        self.add_output('m_dot',
                         val=0.,
                         desc='mass flow rate',
                         units='kg/s')

    def solve_nonlinear(self, params, unknowns, resids):
        tube_pressure = params['tube_pressure']
        pod_mach = params['pod_mach']
        comp_inlet_area = params['comp_inlet_area']
        tube_temp = params['tube_temp']
        gamma = params['gamma']
        comp_mach = params['comp_mach']
        R = params['R']
        eta = params['eta']

        Tt = tube_temp*(1+((gamma-1)/2)*(pod_mach**2))
        Pt = tube_pressure*((1+((gamma-1)/2)*(pod_mach**2))**(gamma/(gamma-1.0)))
        rho = tube_pressure/(R*tube_temp)
        rho_t = rho*((1+((gamma-1)/2))**(1/(gamma-1)))

        p02 = tube_pressure*((1 + eta*((Tt/tube_temp)-1))**(gamma/(gamma-1)))
        rho_2 = rho_t*((p02/Pt)**(1/gamma))

        rho_comp = rho_t/((1+((gamma-1)/2)*(comp_mach**2))**(1/(gamma-1)))
        T_comp = Tt/(1+((gamma-1)/2)*(comp_mach**2))
        m_dot = rho_comp*comp_inlet_area*comp_mach*np.sqrt(gamma*R*T_comp)

        unknowns['Pt'] = Pt
        unknowns['Tt'] = Tt
        unknowns['m_dot'] = m_dot

if __name__ == "__main__":
    top = Problem()
    root = top.root = Group()

    root.add('flow_path_inputs', FlowPathInputs())

    top.setup()
    top.run()

    print('Pt %f' % top['flow_path_inputs.Pt'])
    print('Tt %f' % top['flow_path_inputs.Tt'])
    print('m_dot %f' % top['flow_path_inputs.m_dot'])
