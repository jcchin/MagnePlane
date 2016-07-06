from openmdao.api import Component,Group,Problem


class TubePower(Component):
    def __init__(self):
        super(TubePower, self).__init__()
        self.add_param('vac_power',val=1000.0,desc='Power for vacuum',units='W')
        self.add_param('prop_power',val=300000.0,desc='Power for propulsion systems (LIM/LSM)',units='W')
        self.add_param('tube_temp',val=0.0,desc='Tube temperature',units='K')

        #self.add_output('cooling_power',val=0.0,desc='Power for tube temperature control',units='W')
        self.add_output('tot_power', val=0.0, desc='Total power output', units='W')

    def solve_nonlinear(self, params, unknowns, resids):
        unknowns['tot_power'] = params['vac_power']+params['prop_power']#+params['cooling_power']

if __name__ == '__main__':
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', TubePower())
    prob.setup()
    prob.run()

    print('Total Tube Power Required: %f' % prob['comp.tot_power'])

