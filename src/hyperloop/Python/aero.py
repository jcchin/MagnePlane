''' FUN3D Wrapper '''

# --- Python/system level imports
import numpy as np
from string import Template

# --- OpenMDAO main and library imports
from openmdao.core.component import Component
from openmdao.core.problem import Problem
from openmdao.core.group import Group

class Aero(Component):

    '''Placeholder for real aerodynamic calculations of the capsule'''

    def __init__(self):
        super(Aero, self).__init__()

        self.add_param('p_inf', 3497.8, desc='Freestream static pressure', units='Pa')
        self.add_param('t_inf', 291.111, desc='Freestream static temperature', units='K')
        self.add_param('a_inf', 342.241, desc='Freestream reference speed of sound', units='m/s')             
        self.add_param('M_inf', 0.8, desc='Freestream Mach no.')        
        self.add_param('Re', 223789.835, desc='Reynolds no. per unit grid length')
        self.add_param('SPR', 1.136, desc='Fan face static pressure ratio')
        self.add_param('MFR', 5.803, desc='Compressor mass flow rate (actual)', units='kg/s')
        self.add_param('TPR', 18.865, desc='Nozzle plenum total pressure ratio')
        self.add_param('TTR', 2.407, desc='Nozzle plenum total temperature ratio')        
        self.add_param('Lstar_Lref', 1.0, desc='Ratio of dimensional reference length to grid length')

        self.add_param('coef_drag', 1.0, desc='capsule drag coefficient')
        self.add_param('area_frontal', 1.5, desc='frontal area of capsule', units='m**2')
        self.add_param('velocity_capsule', 600.0, desc='capsule velocity', units='m/s')
        self.add_param('gross_thrust', 0.0, desc='nozzle gross thrust', units='N')

        self.add_output('net_force', 0.0, desc='net force with drag considerations', units='N')
        self.add_output('drag', 0.0, desc='drag force', units='N')

    def solve_nonlinear(self, params, unknowns, resids):
        # drag = Cd * rho * Velocity ** 2 * Area / 2.0

        print ("")
        print ("---------------------------------")
        print ("------- FUN3D Parameters --------")
        print ("---------------------------------")

        print ("Non-Dimensional Variables:")
        print ("SPR(1)     = %f    " % (params['SPR']))
        print ("")
        print ("TPR(2)     = %f    " % (params['TPR']))
        print ("TTR(2)     = %f    " % (params['TTR']))
        print ("")
        print ("L*/Lref = %f " % (params['Lstar_Lref']))
        print ("Re/grid = %f " % (params['Re']))
        print ("")
        print ("Dimensional Variables:")
        print ("a*    = %f    m/s" % (params['a_inf']))
        print ("u*    = %f    m/s" % (params['M_inf']*params['a_inf']))
        print ("p*  = %f      Pa" % (params['p_inf']))
        print ("t*  = %f      Pa" % (params['t_inf']))

        # --- Open template file
        filein = open( '../Aero/Fun3D/fun3d.template' )

        src = Template( filein.read() )

        d = { 'Re':params['Re'], 'M_inf':params['M_inf'], 'SPR':params['SPR'], 'TPR':params['TPR'], 'TTR':params['TTR'] }

        # --- Perform Substitutions
        result = src.substitute(d)

        # --- Write output
        fh = open('../Aero/Fun3D/fun3d.nml', 'w')
        fh.write(result)
        fh.close()

        # --- External code call here
        # Transfer solver input files to NAS
            # All files in "../Fun3D/Flow/*" can be transferred to "pfe:/nobackup/username/some_directory" - Can some_directory be created on the fly?
        
        # Submit PBS script to queue
            # "qsub qscript"  
            
        # Transfer solver output files from NAS to local host
            # Return the following solver files:
            # "Hyperloop_tec_boundary.plt"
            # "Hyperloop.forces"

        # --- Parse output files here

        #unknowns['drag'] = params['coef_drag'] * params['rho'] * params['velocity_capsule'] ** 2 * params['area_frontal'] / 2.0
        #unknowns['net_force'] = params['gross_thrust'] - unknowns['drag']

if __name__ == '__main__':

    p = Problem(root=Group())
    p.root.add('aero', Aero())
    p.setup()
    p.root.list_connections()
    p.run()

    #print 'drag (N): %f' % p['aero.drag']
    #print 'net_force N(kW*hr): %f' % p['aero.net_force']


