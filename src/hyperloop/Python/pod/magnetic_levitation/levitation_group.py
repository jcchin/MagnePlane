from openmdao.api import Group, Problem, IndepVarComp, ExecComp, ScipyOptimizer
from hyperloop.Python.pod.magnetic_levitation.breakpoint_levitation import BreakPointDrag, MagMass
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
    w_track : float
        width of the track (m)
    vel_b : float
        desired breakpoint levitation speed (m/s)

    Outputs
    -------
    mag_drag : float
        magnetic drag from levitation system (N)
    m_mag : float
        mass of the magnets needed for levitation (kg)
    cost : float
        total material cost for the magnets (USD)

    References
    ----------
    .. [1] Friend, Paul. Magnetic Levitation Train Technology 1. Thesis.
       Bradley University, 2004. N.p.: n.p., n.d. Print.

    """
    
    def __init__(self):
        super(LevGroup, self).__init__()

        # Creates components of the group.
        self.add('Drag', BreakPointDrag(), promotes=['m_pod', 'w_track', 'l_pod', 'vel_b'])
        self.add('Mass', MagMass(), promotes=['l_pod', 'cost', 'm_mag'])
        self.add('MDrag', MagDrag(), promotes=['mag_drag'])

        # Connect Drag outputs to MDrag inputs
        self.connect('Drag.track_res', 'MDrag.track_res')
        self.connect('Drag.track_ind', 'MDrag.track_ind')
        self.connect('Drag.pod_weight', 'MDrag.pod_weight')
        self.connect('Drag.lam', 'MDrag.lam')
        self.connect('Drag.pod_weight', 'MDrag.pod_weight')

        # Connect w_track to w_mag to assume magnet array width = width of track
        self.connect('w_track', ['Drag.w_mag', 'Mass.w_mag'])

if __name__ == "__main__":

    top = Problem()
    root = top.root = Group()

    # Define Parameters
    params = (('m_pod', 3000.0, {'units': 'kg'}),
    		  ('l_pod', 25.0, {'units': 'm'}),
              ('w_track', 2.0, {'units': 'm'}),
              ('vel_b', 23.0, {'units': 'm/s'}),
              ('w_mag', 2.0, {'units': 'm'}),
              ('mag_thk', .05, {'units': 'm'}),
              ('gamma', .05),
              ('g', 9.81, {'units': 'm/s**2'}))

    root.add('input_vars', IndepVarComp(params))
    root.add('lev', LevGroup())

    # Constraint Equation
    root.add('con1', ExecComp('c1 = (fyu - m_pod * g)/1e5'))

    # Connect
    root.connect('lev.Drag.fyu', 'con1.fyu')
    root.connect('input_vars.g', 'lev.Drag.g')
    root.connect('lev.Drag.g', 'con1.g')

    root.connect('input_vars.mag_thk', 'lev.Drag.mag_thk')
    root.connect('input_vars.mag_thk', 'lev.Mass.mag_thk')

    root.connect('input_vars.gamma', 'lev.Drag.gamma')
    root.connect('input_vars.gamma', 'lev.Mass.gamma')

    root.connect('input_vars.m_pod', 'lev.m_pod')
    root.connect('lev.m_pod', 'con1.m_pod')
    root.connect('input_vars.l_pod', 'lev.l_pod')
    root.connect('input_vars.w_mag', 'lev.Drag.w_mag')
    root.connect('input_vars.vel_b', 'lev.vel_b')

    # Finite Difference
    root.deriv_options['type'] = 'fd'
    root.fd_options['form'] = 'forward'
    root.fd_options['step_size'] = 1.0e-6

    # Optimizer Driver
    top.driver = ScipyOptimizer()
    top.driver.options['optimizer'] = 'COBYLA'

    # Design Variables
    top.driver.add_desvar('input_vars.mag_thk', lower=.01, upper=.15, scaler=100)
    top.driver.add_desvar('input_vars.gamma', lower=0.1, upper=1.0)

    # Add Constraint
    top.driver.add_constraint('con1.c1', lower=0.0)

    # Problem Objective
    alpha = .5
    root.add('obj_cmp', ExecComp('obj = (alpha*fxu)/1000 + ((1-alpha)*m_mag)'))
    root.connect('lev.Drag.fxu', 'obj_cmp.fxu')
    root.connect('lev.m_mag', 'obj_cmp.m_mag')

    top.driver.add_objective('obj_cmp.obj')

    top.setup()
    top.run()

    print('Mag_drag %fN' % top['lev.mag_drag'])
