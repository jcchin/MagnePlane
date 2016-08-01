from openmdao.api import Group, Component, Problem, IndepVarComp, ExecComp, ScipyOptimizer
from hyperloop.Python.pod.magnetic_levitation.breakpoint_levitation import BreakPointDrag
from hyperloop.Python.pod.magnetic_levitation.breakpoint_levitation import MagMass
from hyperloop.Python.pod.magnetic_levitation.magnetic_drag import MagDrag

class LevGroup(Group):
    """The Levitation group represents a `Group` of the size of magnets required for levitation at breakpoint velocity,
    and the magnetic drag resulting from this levitation at a given speed. These values are computed
    in an OpenMDAO model.

    Models the levitation parameters following previous work from [1]_

    Components
    ----------
    Drag : BreakPointDrag
        Represents the drag and magnetic parameters need for levitation at breakpoint speed.
    Mass : MagMass
        Represents the mass of magnets needed for levitation at breakpoint speed.
    MDrag : MagDrag
        Represents the magnetic drag acquired from levitation at a given speed.

    Params
    ------
    m_pod : float
        mass of the pod (kg)
    l_pod : float
        length of the pod (m)
    d_pod : float
        diameter of the pod (m)
    vel_b : float
        desired breakpoint levitation speed (m/s)
    h_lev : float
        Levitation height. Default value is .01
    vel : float
        desired magnetic drag speed (m/s)

    Outputs
    -------
    mag_drag : float
        magnetic drag from levitation system (N)
    total_pod_mass : float
        total mass of the pod including magnets (kg)

    References
    ----------
    .. [1] Friend, Paul. Magnetic Levitation Train Technology 1. Thesis.
       Bradley University, 2004. N.p.: n.p., n.d. Print.

    """
    
    def __init__(self):
        super(LevGroup, self).__init__()

        # Creates components of the group.
        self.add('Drag', BreakPointDrag(), promotes=['m_pod', 'd_pod', 'l_pod', 'vel_b', 'h_lev'])
        self.add('Mass', MagMass(), promotes=['total_pod_mass'])
        self.add('MDrag', MagDrag(), promotes=['vel', 'mag_drag'])

        # Connects promoted group params to rest of group
        self.connect('m_pod', 'Mass.m_pod')
        self.connect('d_pod', 'Mass.d_pod')
        self.connect('l_pod', 'Mass.l_pod')

        # Connect Drag outputs to MDrag inputs
        self.connect('Drag.track_res', 'MDrag.track_res')
        self.connect('Drag.track_ind', 'MDrag.track_ind')
        self.connect('Drag.pod_weight', 'MDrag.pod_weight')
        self.connect('Drag.lam', 'MDrag.lam')

if __name__ == "__main__":

    prob = Problem()
    root = prob.root = Group()

    root.add('lev', LevGroup())

    # Define Parameters
    params = (('m_pod', 3000.0, {'units': 'kg'}),
              ('l_pod', 22.0, {'units': 'm'}),
              ('d_pod', 1.0, {'units': 'm'}),
              ('vel_b', 23.0, {'units': 'm/s'}),
              ('h_lev', 0.01, {'unit': 'm'}),
              ('vel', 350.0, {'units': 'm/s'}))

    prob.root.add('input_vars', IndepVarComp(params))

    # Constraint Equation
    #root.add('con1', ExecComp('c1 = (fyu - m_pod * g)/1e5'))

    # Connect
    #prob.root.connect('lev.Drag.fyu', 'con1.fyu')
    #prob.root.connect('lev.Drag.g', 'con1.g')

    prob.root.connect('input_vars.m_pod', 'lev.m_pod')
    #prob.root.connect('lev.m_pod', 'con1.m_pod')
    prob.root.connect('input_vars.l_pod', 'lev.l_pod')
    prob.root.connect('input_vars.d_pod', 'lev.d_pod')
    prob.root.connect('input_vars.vel_b', 'lev.vel_b')
    prob.root.connect('input_vars.h_lev', 'lev.h_lev')
    prob.root.connect('input_vars.vel', 'lev.vel')

    # Finite Difference
    #root.deriv_options['type'] = 'fd'
    #root.fd_options['form'] = 'forward'
    #root.fd_options['step_size'] = 1.0e-6

    # Optimizer Driver
    #top.driver = ScipyOptimizer()
    #top.driver.options['optimizer'] = 'COBYLA'

    # Design Variables
    #top.driver.add_desvar('input_vars.mag_thk', lower=.01, upper=.15, scaler=100)
    #top.driver.add_desvar('input_vars.gamma', lower=0.1, upper=1.0)

    # Add Constraint
    #top.driver.add_constraint('con1.c1', lower=0.0)

    # Problem Objective
    #alpha = .5
    #root.add('obj_cmp', ExecComp('obj = (alpha*fxu)/1000 + ((1-alpha)*m_mag)'))
    #prob.root.connect('lev.Drag.fxu', 'obj_cmp.fxu')
    #prob.root.connect('lev.m_mag', 'obj_cmp.m_mag')

    #top.driver.add_objective('obj_cmp.obj')

    prob.setup()
    #prob.root.list_connections()
    #from openmdao.api import view_tree
    #view_tree(prob)

    prob.run()

    print('Mag_drag %f N' % prob['lev.mag_drag'])
    print('Total pod mass %f kg' % prob['lev.total_pod_mass'])
