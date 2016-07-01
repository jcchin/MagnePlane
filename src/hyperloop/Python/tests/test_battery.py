from __future__ import print_function
import pytest
from hyperloop.Python import battery
import numpy as np
from openmdao.api import Group, Problem


def create_problem(battery):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', battery)
    return prob


class TestBattery(object):
    def test_case1_vs_hand_calc(self):

        prob = create_problem(battery.Battery())
        prob.setup()

        prob['comp.des_time'] = 1.0
        prob['comp.time_of_flight'] = 2.0
        prob['comp.des_power'] = 7.0
        prob['comp.des_current'] =1.0
        prob['comp.q_l'] =0.1
        prob['comp.e_full'] =1.4
        prob['comp.e_nom'] = 1.2
        prob['comp.e_exp'] =1.27
        prob['comp.q_n'] =6.8
        prob['comp.t_exp'] =1.0
        prob['comp.t_nom'] =4.3
        prob['comp.r'] =0.0046
        prob['comp.cell_mass'] =170
        prob['comp.cell_height'] =61.0
        prob['comp.cell_diameter'] =33.0

        prob.run()

        assert np.isclose(prob['comp.n_cells'], 2.0, rtol=0.001)
        assert np.isclose(prob['comp.output_voltage'], 2.4, rtol=0.001)
        assert np.isclose(prob['comp.battery_mass'], 0.34, rtol=0.001)
        assert np.isclose(prob['comp.battery_volume'], 115.0583, rtol=0.001)

