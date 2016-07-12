from openmdao.api import IndepVarComp,Component,Group
import numpy as np

class TubePower(Component):
    """
    Computes the total power requirement for all tube components:
    Vacuum, TubeAndPylon, PropulsionMechanics

    Params
    ------
    vac_power : float
        Power requirement to run vacuum pumps (W)
    prop_power : float
        Power requirement to accelerate pod (W)
    tube_temp : float
        Ambient temperature inside tube (K)

    Outputs
    -------
    cooling_power : float
        Power requirement to cool tube (W)
    tot_power : float
        Total power requirement for all tube components (W)

    Notes
    -----
    Power requirement to cool the tube is not currently calculated in this component. Params to calculate
    that power in the future are commented out for the meantime.

    TODO: add in calculations for refrigeration power requirement?

    """

    def __init__(self):
        super(TubePower, self).__init__()
        self.add_param('vac_power',val=0.0,desc='Power for vacuum',units='W')
        self.add_param('prop_power',val=0.0,desc='Power for propulsion systems (LIM/LSM)',units='W')
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

