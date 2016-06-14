from openmdao.core.component import Component
import math
"""Cost estimate of tunnel given length and diameter"""

class TunnelCost(Component):
    def __init__(self):
        super(TunnelCost, self).__init__()
        # default inner diameter for passenger tube from Hyperloop Alpha
        self.add_param('diameter', 2.23, desc='diameter of tunnel', units='m')
        # default tunnel length from SF to LA
        self.add_param('length', 563.00, desc='length of tunnel', units='km')

        self.add_output('cost', 0.0, desc='total cost of tunnel', units='USD')

    # https://www.researchgate.net/publication/233926915_Planning_level_tunnel_cost_estimation_based_on_statistical_analysis_of_historical_data
    # formula taken from conventional subway excavation data
    def solve_nonlinear(self, params, unknowns, resids):
        unknowns['cost'] = math.pow(10, (1.10 + (0.933 * math.log10(params['length'])) + (0.614 * math.log10(params['diameter']))))

if __name__ == '__main__':
    from openmdao.core.problem import Problem
    from openmdao.core.group import Group

    p = Problem(root=Group())
    p.root.add('comp', TunnelCost())
    p.setup()
    p.root.list_connections()
    p.run()

    print 'diameter (m): %f' % p['comp.diameter']
    print 'length (km): %f' % p['comp.length']
    print 'cost (millions USD): %f' % p['comp.cost']
