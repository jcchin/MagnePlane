"""
In this file we define the "Right Hand Side" of the dynamics for the
Magneplane concept to be used by the pointer framework, and create an
OpenMDAO problem to solve the optimal control problem using Pointer.
"""


import numpy as np

from openmdao.core.component import Component

try:
    from openmdao.drivers.pyoptsparse_driver import pyOptSparseDriver
except ImportError:
    pyOptSparseDriver = None
from openmdao.drivers.scipy_optimizer import ScipyOptimizer

from pointer.components import Problem, Trajectory, RHS, \
                               EOMComp, CollocationPhase


class MagneplaneEOM(EOMComp):
    """ The equations of motion for the Magneplane concept.

    In this implementation we assume a Magneplane vehicle is constrained
    to move tangentially to some 'track' defined in cartesian 3-space.

    We assume that this cartesian 3-space is inertial (the rotation of the
    Earth is ignored).  The inertial frame coordinates are specified in a
    'North-East-Down' (NED) coordinate frame $\hat{e}$ such that:

    $\hat{e}_0$ (North) lies tangential to the Earth surface and points North
    $\hat{e}_1$ (East) lies tangential to the Earth surface and points East
    $\hat{e}_2$ (Down) Is perpendicular to the Earth surface is nadir-pointing

    Instead of using latitude/longitude we will assume some 'km per deg' of
    latitude and longitude and specify the track path using an x-y set in the
    NED frame.  The altitude of the track will be specified ground level at a
    given position.  Using digital elevation maps from the USGS, we will then
    be able to determine the entire path of the tube/track in 3-space.

    The motion the pod along the track is constrained to be tangential with
    the track.  The simplified 3DOF equations of motion for the pod are:

    ..math::

        \frac{dv}{dt} = -g \cdot \sin \theta + \frac{T - D}{m}
        \frac{dx}{dt} = v \cdot \cos \theta \cos \psi
        \frac{dy}{dt} = v \cdot \cos \theta \cos \psi
        \frac{dz}{dt} = -v \cdot \sin \psi

    Here $\psi$, $\theta$ and $\phi$ are the 3-2-1 Euler angle set used
    to define the pod axial frame in the NED frame.

    $\psi$ (azimuth angle) is a rotation about $\hat{e}_2$ that rotates the pod
    x-axis to the proper heading.

    $\theta$ (elevation angle) is a rotation about $\hat{e}'_1$ that rotates
     the pod x-axis to the proper elevation.

    $\phi$ (roll angle) is a rotation about  $\hat{e}''_1$ that rotates the pod
    within the tube.

    In general the roll angle $\phi$ will typically be zero or small.  It
    may be necessary to bank the pod in turns to achieve a comfortable g-loading
    on passengers.

    Note that in the presence of no thrust or drag/friction, these equations
    essentially describe the Brachistochrone problem.  We can therefore use
    the known analytic solution to these brachistochrone problem as a
    test of these equations in the absence of thrust and drag on the pod.
    """

    def __init__(self, grid_data):
        super(MagneplaneEOM, self).__init__(grid_data, time_units='s')

        self.deriv_options['type'] = 'user'

        self.add_param('x',desc='north component of position',units='m',eom_state=True)
        self.add_param('y',desc='east component of position',units='m',eom_state=True)
        self.add_param('z',desc='down component of position',units='m',eom_state=True)
        self.add_param('v',desc='pod velocity',units='m/s',eom_state=True)

        self.add_param('g',desc='gravitational acceleration',units='m/s/s')
        self.add_param('psi',desc='azimuth angle',units='rad')
        self.add_param('theta',desc='elevation angle',units='rad')
        #self.add_param('phi',desc='roll angle',units='rad')
        self.add_param('T', desc='thrust force', units='N')
        self.add_param('D', desc='drag/friction force', units='N')
        self.add_param('mass', desc='pod mass', units='kg')

        # Jacobian
        self._J = {}

        # Partials of dx/dt
        self._J['dXdt:x', 'v'] = np.eye(self.num_nodes)
        self._J['dXdt:x', 'theta'] = np.eye(self.num_nodes)
        self._J['dXdt:x', 'psi'] = np.eye(self.num_nodes)

        # Partials of dy/dt
        self._J['dXdt:y', 'v'] = np.eye(self.num_nodes)
        self._J['dXdt:y', 'theta'] = np.eye(self.num_nodes)
        self._J['dXdt:y', 'psi'] = np.eye(self.num_nodes)

        # Partials of dz/dt
        self._J['dXdt:z', 'v'] = np.eye(self.num_nodes)
        self._J['dXdt:z', 'theta'] = np.eye(self.num_nodes)

        # Partials of dv/dt
        self._J['dXdt:v', 'g'] = np.eye(self.num_nodes)
        self._J['dXdt:v', 'T'] = np.eye(self.num_nodes)
        self._J['dXdt:v', 'D'] = np.eye(self.num_nodes)
        self._J['dXdt:v', 'mass'] = np.eye(self.num_nodes)
        self._J['dXdt:v', 'theta'] = np.eye(self.num_nodes)

    def solve_nonlinear(self, params, unknowns, resids):
        v = params['v']
        g = params['g']
        theta = params['theta']
        psi = params['psi']
        T = params['T']
        D = params['D']
        mass = params['mass']

        unknowns['dXdt:x'][:] =  v*np.cos(theta)*np.cos(psi)
        unknowns['dXdt:y'][:] =  v*np.cos(theta)*np.sin(psi)
        unknowns['dXdt:z'][:] = -v*np.sin(theta)
        unknowns['dXdt:v'][:] = -g*np.sin(theta) + (T-D)/mass

    def linearize(self, params, unknowns, resids):
        v = params['v']
        g = params['g']
        theta = params['theta']
        psi = params['psi']
        T = params['T']
        D = params['D']
        mass = params['mass']

        np.fill_diagonal(self._J['dXdt:x','v'], np.cos(theta)*np.cos(psi))
        np.fill_diagonal(self._J['dXdt:x','theta'], -v*np.sin(theta)*np.cos(psi))
        np.fill_diagonal(self._J['dXdt:x','psi'], -v*np.cos(theta)*np.sin(psi))

        np.fill_diagonal(self._J['dXdt:y','v'], np.cos(theta)*np.sin(psi))
        np.fill_diagonal(self._J['dXdt:y','theta'], -v*np.sin(theta)*np.sin(psi))
        np.fill_diagonal(self._J['dXdt:y','psi'], v*np.cos(theta)*np.cos(psi))

        np.fill_diagonal(self._J['dXdt:z','v'], -np.sin(theta))
        np.fill_diagonal(self._J['dXdt:z','theta'], -v*np.cos(theta))

        np.fill_diagonal(self._J['dXdt:v', 'g'], -np.sin(theta))
        np.fill_diagonal(self._J['dXdt:v', 'T'], 1.0/mass)
        np.fill_diagonal(self._J['dXdt:v', 'D'], -1.0/mass)
        np.fill_diagonal(self._J['dXdt:v', 'mass'], (D-T)/mass**2)
        np.fill_diagonal(self._J['dXdt:v', 'theta'], -g*np.cos(theta))

        return self._J



class AngularVelocityComp(EOMComp):
    """ Component to compute the angular velocity of the pod in
    the inertial NED frame.  This is from the definition of the
    angular velocity vector based on euler angles and rates in a
    3-2-1 Euler angle sequence.
    """

    def __init__(self, grid_data):
        super(AngularVelocityComp, self).__init__(grid_data=grid_data, time_units='s')

        self.deriv_options['type'] = 'fd'

        self.add_param('psi',desc='azimuth angle',units='rad')
        self.add_param('theta',desc='elevation angle',units='rad')
        self.add_param('phi',desc='roll angle',units='rad')

        self.add_param('dUdt:psi',desc='azimuth angle rate',units='rad/s')
        self.add_param('dUdt:theta',desc='elevation angle rate',units='rad/s')
        self.add_param('dUdt:phi',desc='roll angle rate',units='rad/s')

        nn = grid_data['num_nodes']

        self.add_output('omega_x',shape=(nn,), desc='omega about north',units='rad/s')
        self.add_output('omega_y',shape=(nn,), desc='omega about east',units='rad/s')
        self.add_output('omega_z',shape=(nn,), desc='omega about down',units='rad/s')

    def solve_nonlinear(self, params, unknowns, resids):
        psi = params['psi']
        theta = params['theta']
        phi = params['phi']
        psi_dot = params['dUdt:psi']
        theta_dot = params['dUdt:theta']
        phi_dot = params['dUdt:phi']

        unknowns['omega_x'][:] = phi_dot - psi_dot*np.sin(theta)
        unknowns['omega_y'][:] = theta_dot*np.cos(phi) + psi_dot*np.sin(phi)*np.cos(theta)
        unknowns['omega_z'][:] = -theta_dot*np.sin(phi) + psi_dot*np.cos(phi)*np.cos(theta)








class MagneplaneRHS(RHS):

    def __init__(self,grid_data,dynamic_controls=None,static_controls=None):
        super(MagneplaneRHS,self).__init__(grid_data,dynamic_controls,static_controls)

        self.add(name='eom',system=MagneplaneEOM(grid_data),promotes=['*'])
        self.add(name='omega',system=AngularVelocityComp(grid_data),promotes=['*'])

        self.complete_init()



def magneplane_brachistochrone(solver='SLSQP', num_seg=3, seg_ncn=3):

    prob = Problem()
    traj = prob.add_traj(Trajectory("traj0"))

    if solver == 'SNOPT':
        if pyOptSparseDriver is None:
            raise ValueError('Requested SNOPT but pyoptsparse is not available')
        driver=pyOptSparseDriver()
        driver.options['optimizer'] = solver
        driver.opt_settings['Major iterations limit'] = 1000
        driver.opt_settings['iSumm'] = 6
        driver.opt_settings['Major step limit'] = 0.5
        driver.opt_settings["Major feasibility tolerance"] = 1.0E-6
        driver.opt_settings["Major optimality tolerance"] = 1.0E-6
        driver.opt_settings["Minor feasibility tolerance"] = 1.0E-4
        driver.opt_settings['Verify level'] = 3
    else:
        driver=ScipyOptimizer()
        driver.options['tol'] = 1.0E-6
        driver.options['disp'] = True
        driver.options['maxiter'] = 500

    prob.driver = driver

    dynamic_controls = [ {'name':'g','units':'m/s**2'},
                        {'name':'T','units':'N'},
                        {'name':'D','units':'N'},
                        {'name':'mass','units':'kg'},
                        {'name':'psi','units':'rad'},
                        {'name':'theta', 'units':'rad'},
                        {'name':'phi','units':'rad'} ]

    phase0 = CollocationPhase(name='phase0',rhs_class=MagneplaneRHS,num_seg=num_seg,seg_ncn=seg_ncn,rel_lengths="equal",
                              dynamic_controls=dynamic_controls,static_controls=None)

    traj.add_phase(phase0)

    phase0.set_state_options('x', lower=0,upper=10,ic_val=0,ic_fix=True,fc_val=10,fc_fix=True,defect_scaler=0.1)
    phase0.set_state_options('y', lower=0,upper=0,ic_val=0,ic_fix=True,fc_val=0,fc_fix=True,defect_scaler=0.1)
    phase0.set_state_options('z', lower=-10,upper=0,ic_val=-10,ic_fix=True,fc_val=-5,fc_fix=True,defect_scaler=0.1)
    phase0.set_state_options('v', lower=0, upper=np.inf,ic_val=0.0,ic_fix=True,fc_val=10.0,fc_fix=False,defect_scaler=0.1)

    phase0.set_dynamic_control_options(name='psi', val=phase0.node_space(0.0, 0.0), opt=False)
    phase0.set_dynamic_control_options(name='theta', val=phase0.node_space(-.46,-.46),opt=True,lower=-1.57,upper=1.57,scaler=1.0)
    phase0.set_dynamic_control_options(name='phi', val=phase0.node_space(0.0, 0.0), opt=False)

    phase0.set_dynamic_control_options(name='g', val=phase0.node_space(9.80665, 9.80665), opt=False)
    phase0.set_dynamic_control_options(name='T', val=phase0.node_space(0.0, 0.0), opt=False)
    phase0.set_dynamic_control_options(name='D', val=phase0.node_space(0.0, 0.0), opt=False)
    phase0.set_dynamic_control_options(name='mass', val=phase0.node_space(1000.0, 1000.0), opt=False)

    phase0.set_time_options(t0_val=0,t0_lower=0,t0_upper=0,tp_val=2.0,tp_lower=0.5,tp_upper=10.0)

    traj.add_objective(name="t",phase="phase0",place="end",scaler=1.0)

    return prob



if __name__ == "__main__":
    prob = magneplane_brachistochrone('SNOPT',num_seg=10,seg_ncn=2)

    prob.setup()

    # np.set_printoptions(linewidth=1024)
    # with open('check_partials.txt','wb') as f:
    #     prob.check_partial_derivatives(out_stream=f)
    #
    # exit(0)

    prob.run()

    simout = prob.trajectories['traj0'].simulate(dt=0.01)

    import matplotlib.pyplot as plt
    plt.plot(prob['traj0.phase0.rhs_c.x'],prob['traj0.phase0.rhs_c.z'],'ro')
    plt.plot(simout['phase0']['x'],simout['phase0']['z'])
    plt.gca().invert_yaxis()
    #plt.plot(simout['phase0']['t'],-simout['phase0']['z'])

    plt.figure()

    plt.plot(prob['traj0.phase0.rhs_c.t'],prob['traj0.phase0.rhs_c.omega_x'],'ro')
    plt.plot(prob['traj0.phase0.rhs_c.t'],prob['traj0.phase0.rhs_c.omega_y'],'bo')
    plt.plot(prob['traj0.phase0.rhs_c.t'],prob['traj0.phase0.rhs_c.omega_z'],'go')

    #plt.plot(simout['phase0']['t'],simout['phase0']['omega_y'])
    #plt.plot(simout['phase0']['t'],simout['phase0']['omega_z'])

    plt.show()

    #

