# import libraries
from math import pi, sin
import matplotlib.pyplot as plt

from openmdao.core.component import Component


class TubeCost(Component):
    def __init__(self):
        super(TubeCost, self).__init__()
        self.add_param('cmt', 1.2, desc='cost of materials', units='1/kg')
        self.add_param('cship', 1.e11, desc='cost to ship and crew', units='')
        self.add_param('cpod', 1.e6, desc='cost of a pod', units='')
        self.add_param('bm', 20., desc='bond maturity')
        self.add_param('ib', 0.06, desc='interest bond rate', units='deg')
        self.add_param('stress', 2.0e8, desc='stress of steel', units='Pa')
        self.add_param('sf', 5.0, desc='safety factor', units='')
        self.add_param('depth', 10., desc='depth of tube', units='m')
        self.add_param('density',
                       8050.0,
                       desc='density of steel',
                       units='kg/m**3')
        self.add_param('g', 9.81, desc='gravity', units='m/s**2')
        self.add_param('radius', 4.0, desc='tube radius', units='m')
        self.add_param('pod_freq',
                       30.,
                       desc='seconds between departures',
                       units='s')
        self.add_param('range', 4.1e6, desc='length of tube', units='m')
        self.add_param('npax', 25., desc='passengers per pod', units='')
        self.add_param('speed', 270., desc='passengers per pod', units='m/s')

        self.add_output('tube_weight',
                        0.0,
                        desc='tube weight per meter',
                        units='kg/m')
        self.add_output('po_tube',
                        0.0,
                        desc='pressure on the tube',
                        units='kg/m**2')
        self.add_output('tube_thick', 0.0, desc='tube thickness', units='m')
        self.add_output('ct', 0.0, desc='tube cost per meter', units='1/m')
        self.add_output('cttot', 0.0, desc='total tube cost', units='')
        self.add_output('npod',
                        0.0,
                        desc='number of pods in the tube',
                        units='')
        self.add_output('ctick', 0.0, desc='ticket cost', units='1/m')

    def solve_nonlinear(self, p, u, r):
        u['po_tube'] = 101000 + p['depth'] * p['g'] * 1000.
        u['tube_thick'] = p['sf'] * u['po_tube'] * p['radius'] / (2 *
                                                                  p['stress'])
        u['tube_weight'] = p['density'] * pi * u['tube_thick'] * (
            2 * p['radius'] - u['tube_thick'])
        u['ct'] = p['cmt'] * u['tube_weight']
        u['cttot'] = u['ct'] * p['range']
        u['npod'] = (p['range'] / p['speed']) / p['pod_freq']
        u['ctick'] = ((u['ct']*p['range'] + p['cpod']*u['npod'] + p['cship'])*(1+p['ib'])) \
                    / (p['npax']/p['pod_freq'])/p['bm']/365./24./3600.


if __name__ == '__main__':
    from openmdao.core.problem import Problem
    from openmdao.core.group import Group

    p = Problem(root=Group())
    p.root.add('cost', TubeCost())
    p.setup()
    p.run()

    # save variable sweeps in arrays
    cost_array = []
    cx = []
    cost_array2 = []
    cx2 = []

    print(p['cost.po_tube'])
    print(p['cost.tube_thick'])
    print(p['cost.ct'])
    print(p['cost.ct'])
    print(p['cost.npod'])

    for i in xrange(1, 100, 1):
        p['cost.pod_freq'] = i
        p.run()
        cost_array.append(p['cost.ctick'])
        cx.append(i)

    for i in xrange(10, 35, 1):
        p['cost.pod_freq'] = 30.
        p['cost.stress'] = i * 1.e7
        p.run()
        cost_array2.append(p['cost.ctick'])
        cx2.append(i * 1.e7)

# plot variable sweeps
    fig = plt.figure()
    a1 = fig.add_subplot(211)
    a1.plot(cx, cost_array)
    plt.xlabel('seconds between departures')
    plt.ylabel('ticket cost')
    a2 = fig.add_subplot(212)
    a2.plot(cx2, cost_array2)
    plt.xlabel('steel strength (Pa)')
    plt.ylabel('ticket price')
    plt.show()

# Tom Gregory
# Cost of steel per ton is about USD 777 and fabrication +
# erection cost is about USD 266. But this is for industrial applications upto
# 14 meters height. This may be high for high rise buildings but fabrication
# and erection costs generally do not cross the supply cost.

# Supply            AUD 2,500/tonne    (~USD 1,821/ton)

# Shop Detailing    AUD 500/tonne        (~USD 364/ton)

# Fabrication        AUD 3,000/tonne     (~USD 2,185/ton)

# Transport        AUD 150/tonne        (~USD 109/ton)

# Erection Labour    AUD 2,400/tonne     (~USD 1,748/ton)

# Erection Plant        AUD 1,200/tonne    (~USD 874/ton)

# TOTAL            AUD 9,750/tonne    (~USD 5,339/ton)

# Steel:

# Another reference

# Tube cost = supply + transport + stir welding + erection

# cmtl     = 700     + 150      + 140   +200 = 1200 $/(1000 kg)

# ship cost: w friction welding, handling,
