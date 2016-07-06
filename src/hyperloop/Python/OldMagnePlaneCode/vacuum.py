from openmdao.core.component import Component
import numpy as np

# http://www.cheresources.com/invision/blog/4/entry-373-power-consumption-of-vacuum-pumps/
# http://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20130009951.pdf


class Vacuum(Component):
    def __init__(self):
        super(Vacuum, self).__init__()

        self.add_param('leakage', 1200.0, units='m**3/hr', desc='leakage rate')
        self.add_param(
            'V',
            40.0,
            desc='volume flow rate of process gas being evacuated per pump',
            units='m**3/hr')
        self.add_param('T',
                       291.11,
                       desc='temperature of process gas being evacuated',
                       units='K')
        self.add_param('MW',
                       0.02897,
                       desc='molecular weight of process gas being evacuated',
                       units='kg/mol')
        self.add_param('P',
                       16299.0,
                       desc='operating pressure of system evacuated',
                       units='Pa')

        self.add_output('BkW', 1.0, desc='power consumption (brake kilowatts)')
        self.add_output('pwr',
                        1.0,
                        desc='total power consumption (brake kilowatts)')
        self.add_output('SF', 1.0, desc='pump size factor')
        self.add_output('n', 1.0, desc='number of pumps')

    def solve_nonlinear(self, p, u, r):
        rho = p['P'] / (287.058 * p['T'])
        G = rho * p['V']
        m = G * np.sqrt(p['T'] * 28.96 /
                        (293.15 * p['MW']))  # Air equivalent flow rate, kg/h
        u['n'] = np.ceil(p['leakage'] / p['V'])
        u['SF'] = m / (p['P'] * 0.00750062)
        # Pump size factor kg/ h/torr
        u['BkW'] = 13.5 * (
            u['SF']
        )**1.088  # Rough estimate for a rotary piston vacuum pump. SF should be between 0.01 - 4.
        u['pwr'] = u['BkW'] * u['n']


if __name__ == '__main__':

    from openmdao.core.problem import Problem
    from openmdao.core.group import Group

    p = Problem(root=Group())
    p.root.add('comp', Vacuum())
    p.setup()
    p.root.list_connections()
    p.run()

    print('Power Consumption (Bhp): %f' % (p['comp.BkW'] * 1.34102))
    print('Total Power Consumption (Bhp): %f  over %i pumps' %
          (p['comp.pwr'] * 1.34102, p['comp.n']))

    if (p['comp.SF'] < 0.01 or p['comp.SF'] > 4.0):
        print("invalid SF")
