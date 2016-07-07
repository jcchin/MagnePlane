"""
Test for tube_group.py.
"""
import pytest
from openmdao.api import Group, Problem

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
        prob.setup()
        # prob.root.list_connections()

        #Tube Inputs
        #Vacuum
        prob['comp.Vacuum.pinit'] = 760.2
        prob['comp.Vacuum.pfinal'] = 7.0
        prob['comp.Vacuum.speed'] = 163333.3
        prob['comp.Vacuum.rad'] = 5.0
        prob['comp.Vacuum.len'] = 5000.0
        prob['comp.Vacuum.pwr'] = 18.5
        prob['comp.Vacuum.eprice'] = 0.13
        prob['comp.Vacuum.tdown'] = 300.0
        prob['comp.Vacuum.gamma'] = 0.8
        prob['comp.Vacuum.pumpweight'] = 715.0

        #Temp Balance
        prob['comp.TempBalance.ss_temp_residual'] = 0.0
        prob['comp.TempBalance.temp_boundary'] = 322.0

        #Tube Wall Temp
        prob['comp.TubeWallTemp.radius_outer_tube'] = 1.115
        prob['comp.TubeWallTemp.length_tube'] = 482803.0
        prob['comp.TubeWallTemp.num_pods'] = 34.0
        prob['comp.TubeWallTemp.temp_boundary'] = 322.0
        prob['comp.TubeWallTemp.temp_outside_ambient'] = 305.6
        prob['comp.TubeWallTemp.nozzle_air_W'] = 34.0
        prob['comp.TubeWallTemp.nozzle_air_Cp'] = 34.
        prob['comp.TubeWallTemp.nozzle_air_Tt'] = 34.0
        prob['comp.TubeWallTemp.bearing_air_W'] = 34.0
        prob['comp.TubeWallTemp.bearing_air_Cp'] = 34.0
        prob['comp.TubeWallTemp.bearing_air_Tt'] = 34.0

        prob['comp.TubeWallTemp.solar_insolation'] = 1000.0
        prob['comp.TubeWallTemp.nn_incidence_factor'] = 0.7
        prob['comp.TubeWallTemp.surface_reflectance'] = 0.5
        prob['comp.TubeWallTemp.emissivity_tube'] = 0.5
        prob['comp.TubeWallTemp.sb_constant'] = 0.00000005670373
        prob['comp.TubeWallTemp.Nu_multiplier'] =  1.0

        #Tube and Pylon
        prob['comp.Struct.rho_tube'] = 7820.0
        prob['comp.Struct.E_tube'] = 200.0 * (10**9)
        prob['comp.Struct.v_tube'] = 0.3
        prob['comp.Struct.Su_tube'] = 152.0e6
        prob['comp.Struct.sf'] = 1.5
        prob['comp.Struct.g'] = 9.81
        prob['comp.Struct.unit_cost_tube'] = 0.3307
        prob['comp.Struct.p_tunnel'] = 100.0
        prob['comp.Struct.p_ambient'] = 101300.0
        prob['comp.Struct.alpha_tube'] = 0.0
        prob['comp.Struct.dT_tube'] = 0.0
        prob['comp.Struct.m_pod'] = 3100.0
        prob['comp.Struct.r'] = 1.1
        prob['comp.Struct.t'] = 0.05
        prob['comp.Struct.rho_pylon'] = 2400.0
        prob['comp.Struct.E_pylon'] = 41.0 * (10**9)
        prob['comp.Struct.v_pylon'] = 0.2
        prob['comp.Struct.Su_pylon'] = 40.0 * (10**6)
        prob['comp.Struct.unit_cost_pylon'] = 0.05
        prob['comp.Struct.h'] = 10.0
        prob['comp.Struct.r_pylon'] = 1.1
        prob['comp.Struct.vac_weight'] = 0.0

        #Propulsion Mechanics
        prob['comp.PropMech.Cd'] = .2
        prob['comp.PropMech.S'] = 1.4
        prob['comp.PropMech.p_tube'] = 100.0
        prob['comp.PropMech.T_ambient'] = 293.0
        prob['comp.PropMech.R'] = 286.9
        prob['comp.PropMech.D_magnetic'] = 150.0
        prob['comp.PropMech.Thrust_pod'] = 3500.0
        prob['comp.PropMech.vf'] = 335.0
        prob['comp.PropMech.v0'] = 324.0
        prob['comp.PropMech.rho_pm'] = 7400.0
        prob['comp.PropMech.A'] = .0225
        prob['comp.PropMech.t'] = .05
        prob['comp.PropMech.g'] = 9.81
        prob['comp.PropMech.m_pod'] = 3100.0
        prob['comp.PropMech.eta'] = .8


        #prob['comp.Power.'] =

        prob.run()

        # Print Statement for debugging

        print('Vacuum: %f' %prob['comp.Vacuum.'])
        print('Tube temperature: %f' % prob['comp.Thermal.'])
        print('Structural: %f' %prob['comp.Struct.'])
        print('Propulsion Mechanics: %f' %prob['comp.PropMech.'])
        print('Total Tube Power required: %f' % prob['comp.TubePower.'])

        # Test Values
        assert np.isclose(prob['comp.Vacuum.'], , rtol=.01)
        assert np.isclose(prob['comp.Thermal.'], , rtol=.01)
        assert np.isclose(prob['comp.Struct.'], , rtol=.01)
        assert np.isclose(prob['comp.PropMech.'], , rtol=.01)
        assert np.isclose(prob['comp.TubePower.'], , rtol=.01)
