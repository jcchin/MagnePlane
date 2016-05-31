''' Freestream component '''

# --- OpenMDAO imports
from openmdao.core.component import Component

# --- Local Python imports

class Freestream(Component):
    ''' OpenMDAO component for calculating freestream properties '''

    # -----------------------------------
    # --- Initialize Input Parameters ---
    # -----------------------------------
    self.add_param('M_inf', 0.65,  desc='freestream vehicle Mach number', units='m**2')
    self.add_param('T_inf', 215.0, desc='freestream vehicle static pressure', units='K')
    self.add_param('P_inf', 0.5,   desc='freestream vehicle static temperature', units='Pa')

    # ------------------------------------
    # --- Initialize Output Parameters ---
    # ------------------------------------
    def solve_nonlinear(self, params, unknowns, resids):
        unknowns['d_inf'] = params['energy'] / params['e']

    d_inf = Float(1.0, iotype = 'out', desc = 'freestream static density', units = 'kg/m**3')
    p_inf = Float(1.0, iotype = 'out', desc = 'freestream static pressure', units = 'Pa')
    t_inf = Float(1.0, iotype = 'out', desc = 'freestream static temperature', units = 'K')

    def __init__(self):
        super(Freestream, self).__init__()
        
        self.force_execute = True
    
    def execute(self):
    
        [self.d_inf, self.p_inf, self.t_inf] = Atmosphere(self.alt*0.0003048)
        print "-------------------------------------"
        print " --- Freestream Static Conditions ---"       
        print "-------------------------------------"
        print ("P_inf = %.6f Pa = %.2f psf" % (self.p_inf, self.p_inf*0.0208854342)) 
        print ("T_inf = %.6f K = %.2f R" % (self.t_inf, self.t_inf*1.8))
        print ("D_inf = %.6f kg/m^3 = %.2f lb/ft^3" % (self.d_inf, self.d_inf*0.0624279606)) 
        print "-------------------------------------"
        print " --- Freestream Total Conditions ---"       
        print "-------------------------------------"
        print ("P_inf = %.6f Pa = %.2f psf" % (self.p_inf, self.p_inf*0.0208854342)) 
        print ("T_inf = %.6f K = %.2f R" % (self.t_inf, self.t_inf*1.8))
        print ("D_inf = %.6f kg/m^3 = %.2f lb/ft^3" % (self.d_inf, self.d_inf*0.0624279606)) 

if __name__ == '__main__':
    from openmdao.core.problem import Problem
    from openmdao.core.group import Group

    p = Problem(root=Group())
    p.root.add('comp', Battery())
    p.setup()
    p.root.list_connections()
    p.run()

    print 'mass (Kg): %f' % p['comp.mass']
    print 'energy (kW*hr): %f' % p['comp.energy']
    print 'volume (m**3): %f' % p['comp.volume']
    print 'length (m): %f' % p['comp.len']


if __name__ == "__main__":
    
    # -------------------------
    # --- Default Test Case ---
    # ------------------------- 
    Freestream_Comp = Freestream()
    Freestream_Comp.run()
