''' FUN3D Wrapper '''

# --- Python/system level imports
import numpy as np

# --- OpenMDAO main and library imports
from openmdao.core.component import Component
from openmdao.core.problem import Problem
from openmdao.core.group import Group

class Aero(Component):

    '''Placeholder for real aerodynamic calculations of the capsule'''

    def __init__(self):
        super(Aero, self).__init__()

        self.add_param('d_inf', 1.0, desc='Freestream static density', units='kg/m**3')
        self.add_param('p_inf', 1.0, desc='Freestream static pressure', units='Pa')
        self.add_param('t_inf', 1.0, desc='Freestream static temperature', units='K')
        self.add_param('M_inf', 1.0, desc='Freestream Mach no.')
        self.add_param('mdot', 1.0, desc='Compressor mass flow rate', units='kg/s')

        self.add_param('coef_drag', 1.0, desc='capsule drag coefficient')
        self.add_param('area_frontal', 1.5, desc='frontal area of capsule', units='m**2')
        self.add_param('velocity_capsule', 600.0, desc='capsule velocity', units='m/s')
        self.add_param('gross_thrust', 0.0, desc='nozzle gross thrust', units='N')

        self.add_output('net_force', 0.0, desc='net force with drag considerations', units='N')
        self.add_output('drag', 0.0, desc='drag force', units='N')

    def solve_nonlinear(self, params, unknowns, resids):
        # drag = Cd * rho * Velocity ** 2 * Area / 2.0

        print ""
        print "---------------------------------"
        print "------- FUN3D Parameters --------"
        print "---------------------------------"

        astar = np.sqrt(gamma_inf*self.R*self.t_inf)
        ustar = astar*self.M_inf
        dstar = gamma_inf*self.p_inf/astar**2
        mustar = 0.00001716*(self.t_inf/273.15)**1.5*(273.15+110.4)/(self.t_inf+110.4) # --- Sutherlands Law
        Re = dstar*ustar/mustar*Lstar_Lref

        print "Non-Dimensional Variables:"
        print ("SPR(1)     = %f    " % (self.ps2/self.p_inf))
        print ""
        print ("TPR(2)     = %f    " % (pp_0/self.p_inf))
        print ("TTR(2)     = %f    " % (self.T4_0/self.t_inf))
        print ""
        print ("c(1)     = %f    " % (a2/astar))
        print ("u(1)     = %f    " % (v2/ustar))
        print ("rho(1)   = %f    " % (d2/dstar))
        print ""
        print ("c(2)     = %f    " % (ap/astar))
        print ("u(2)     = %f    " % (Mp*ap/ustar))
        print ("rho(2)   = %f    " % (dp/dstar))
        print ""
        print ("L*/Lref = %f " % Lstar_Lref)
        print ("Re/grid = %f " % Re)
        print ""
        print "Dimensional Variables:"
        print ("a*    = %f    m/s" % astar)
        print ("u*    = %f    m/s" % ustar)
        print ("rho*  = %f    kg/m^3" % dstar)
        print ("mu*     = %f  kg/(m-s)" % mustar)

        # --- Open template file
        filein = open( '../Fun3D/Flow/fun3d.template' )

        src = Template( filein.read() )

        d = { 'Re':Re, 'M_inf':self.M_inf, 'SPR':(ps2/self.p_inf), 'TPR':(pp_0/self.p_inf), 'TTR':(self.T4_0/self.t_inf), 'c1':(a2/astar), 'u1':(v2/ustar), 'rho1':(d2/dstar), 'c2':(ap/astar), 'u2':(Mp*ap/ustar), 'rho2':(dp/dstar)}

        # --- Perform Substitutions
        result = src.substitute(d)

        # --- Write output
        fh = open('../Fun3D/fun3d.nml', 'w')
        fh.write(result)
        fh.close()

        # --- External code call here

        # --- Parse output files here

        unknowns['drag'] = params['coef_drag'] * params['rho'] * params['velocity_capsule'] ** 2 * params['area_frontal'] / 2.0
        unknowns['net_force'] = params['gross_thrust'] - unknowns['drag']

if __name__ == '__main__':

    p = Problem(root=Group())
    p.root.add('aero', Aero())
    p.setup()
    p.root.list_connections()
    p.run()

    print 'drag (N): %f' % p['aero.drag']
    print 'net_force N(kW*hr): %f' % p['aero.net_force']


