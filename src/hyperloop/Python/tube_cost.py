from math import pi, sin

from openmdao.core.component import Component

class TubeCost(Component):
    def __init__(self):
        super(TubeCost, self).__init__()
        self.add_param('cmt', 1.2, desc='cost of materials', units='1/kg')
        self.add_param('cship', 100000000.0, desc='cost to ship and crew', units='')
        self.add_param('cpod', 100000.0, desc='cost of a pod', units='')
        self.add_param('bm', 20., desc='bond maturity')
        self.add_param('ib', 0.06, desc='interest bond rate', units='deg')
        self.add_param('stress', 20000.0, desc='stress of steel', units='MPa')
        self.add_param('sf', 5.0, desc='safety factor', units='')
        self.add_param('depth', 10., desc='depth of tube', units='m')
        self.add_param('den', 8050.0, desc='density of steel', units='kg/m**3')
        self.add_param('g', 9.81, desc='gravity', units='m/s**2')
        self.add_param('radius', 4.0, desc='tube radius', units='m')
        self.add_param('pod_freq', 60, desc='time between flights', units='s')
        self.add_param('range', 4100000., desc='length of tube', units='m')
        self.add_param('npax', 25., desc='passengers per pod', units='')

        self.add_output('tube_weight', 0.0, desc='tube weight per meter', units='kg/m')
        self.add_output('po_tube', 0.0, desc='pressure on the tube', units='kg/m**2')
        self.add_output('tube_thick', 0.0, desc='tube thickness', units='m')
        self.add_output('ct', 0.0, desc='tube cost per meter', units='1/m')
        self.add_output('cttot', 0.0, desc='total tube cost', units='')
        self.add_output('npod', 0.0, desc='number of pods in the tube', units='')
        self.add_output('ctick', 0.0, desc='ticket cost', units='1/m')

    def solve_nonlinear(self, p, u, r):
        u['po_tube'] = 1. + p['depth']*p['g']*1000.
        u['tube_thick'] = u['po_tube']*p['radius']/p['stress']
        u['tube_weight'] = 2.*pi*p['radius']*u['tube_thick']
        u['ct'] = p['cmt']*u['tube_weight']
        u['cttot'] = u['ct']*p['range']
        u['npod'] = p['range']/p['pod_freq']  #wat?
        u['ctick'] = (u['ct']*p['range'] + p['cpod']*u['npod'] + p['cship'])*(1+p['ib']) \
                    / (p['npax']/p['pod_freq'])*p['bm']/365./24./3600.

if __name__ == '__main__':
    from openmdao.core.problem import Problem
    from openmdao.core.group import Group

    p = Problem(root=Group())
    p.root.add('cost', TubeCost())
    p.setup()
    p.run()
