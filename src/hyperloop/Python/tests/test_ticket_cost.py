import numpy as np
from openmdao.api import Group, Problem

from hyperloop.Python import ticket_cost

def create_problem(component):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', component)
    return prob

class TestTubeAndPylon(object):
    def test_case1_vs_npss(self):

        component = ticket_cost.TicketCost()
        prob = create_problem(component)
        prob.setup()

        prob['comp.length_cost'] = 2.437e6
        prob['comp.pod_cost'] = 1.0e6
        prob['comp.capital_cost'] = 1.0e10
        prob['comp.energy_cost'] = .13
        prob['comp.ib'] = .04
        prob['comp.bm'] = 20.0
        prob['comp.operating_time'] = 16.0*3600
        prob['comp.JtokWh'] = 2.7778e-7
        prob['comp.m_pod'] = 3100.0
        prob['comp.n_passengers'] = 28.0
        prob['comp.pod_period'] = 120.0
        prob['comp.avg_speed'] = 286.86
        prob['comp.track_length'] = 600.0e3
        prob['comp.prop_power'] = 350.0e3
        prob['comp.vac_power'] = 71.049e6
        prob['comp.steady_vac_power'] = 950.0e3
        prob['comp.vf'] = 286.86
        prob['comp.g'] = 9.81
        prob['comp.Cd'] = .2
        prob['comp.S'] = 40.42
        prob['comp.p_tunnel'] = 850.0
        prob['comp.T_tunnel'] = 320.0
        prob['comp.R'] = 287.0
        prob['comp.eta'] = .8
        prob['comp.D_mag'] = (9.81*3100.0)/200.0
        prob['comp.thrust_time'] = 1.5
        prob['comp.prop_period'] = 25.0e3

        prob.run()

        assert np.isclose(prob['comp.ticket_cost'], 80.8496946995, rtol=0.1)

