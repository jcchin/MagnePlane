import pytest
from openmdao.api import Group, Problem, IndepVarComp

from hyperloop.Python.tube import tube_group
import numpy as np

def create_problem(GroupName):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', GroupName)
    return prob

class TestTube(object):
    def test_case1(self):

        TubeGroup = tube_group.TubeGroup()
        prob = create_problem(TubeGroup)

        # prob.root.list_connections()

        des_vars = (('tunnel_pressure', 6.37552, {'units' : 'torr'}),
                    ('tube_area', 41., {'units': 'm**2'}),
                    ('tube_length', 480000., {'units': 'm'}))

        prob.root.add('des_vars',IndepVarComp(des_vars))

        prob.root.connect('des_vars.tunnel_pressure', 'comp.p_tunnel')
        prob.root.connect('des_vars.tube_area', 'comp.tube_area')
        prob.root.connect('des_vars.tube_length', 'comp.tube_length')

        prob.setup()

        #Tube Inputs
        prob['comp.pressure_initial'] = 760.2
        prob['comp.pwr'] = 18.5
        prob['comp.speed'] = 163333.3
        prob['comp.time_down'] = 300.0
        prob['comp.gamma'] = .8
        prob['comp.pump_weight'] = 715.0
        prob['comp.nozzle_air_W'] = 1.08
        prob['comp.nozzle_air_Tt'] = 1710.0
        prob['comp.num_pods'] = 34.
        prob['comp.h'] = 10.
        prob['comp.vf'] = 335.
        prob['comp.v0'] = 324.
        prob['comp.Cd'] = 0.2
        prob['comp.S'] = 1.4
        prob['comp.D_mag'] = 150.
        prob['comp.nozzle_thrust'] = 21473.92
        prob['comp.ram_drag'] = 7237.6
        prob['comp.num_thrust'] = 5.0
        prob['comp.time_thrust'] = 1.5
        prob['comp.electricity_price'] = 0.13
        prob['comp.tube_thickness'] = 0.05
        prob['comp.m_pod'] = 3100.
        prob['comp.r_pylon'] = 0.1
        prob['comp.num_pods'] = 34.

        prob.run()

        # Print Statement for debugging
        """
        print('Vacuum.weight_tot:%f' % prob['comp.Vacuum.weight_tot'])
        print('Struct.vac_weight: %f' % prob['comp.Struct.vac_weight'])
        print('Vacuum.tot_pwr %f' % prob['comp.Vacuum.tot_pwr'])
        print('TubePower.vac_power: %f' % prob['comp.TubePower.vac_power'])
        print('PropMech.pwr_req: %f' % prob['comp.PropMech.pwr_req'])
        print('Total Tube Power: %f' % prob['comp.TubePower.tot_power'])
        """

        # Test Values
        assert np.isclose(prob['comp.TubePower.tot_power'],72181.13, rtol=.01)
        assert np.isclose(prob['comp.temp_boundary'],322.63, rtol=.01)
