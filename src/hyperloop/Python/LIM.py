# Model for a Single Sided Linear Induction Motor(SLIM), using Circuit Model 
import math, numpy, scipy
from openmdao.core.component import Component
from openmdao.api import IndepVarComp, Component, Problem, Group, ScipyOptimizer, ExecComp, SqliteRecorder


class Thrust(Component):
    def __init__(self):
        super(Thrust, self).__init__()

        self.add_param('R2', val=.082, desc = 'resistance of rotor' ,units = 'ohms')
        self.add_param('R1', val = 7.6*10**-7, dest = 'per phase stator resistance', units = 'ohms')
        #self.add_param('p', val=1.0, desc = 'no. of poles', units = 'none') 
        self.add_param('P1', val = 180000, desc = 'input power', units = 'watts') 
        self.add_param('X_m', val=4.781, desc = 'per phase magnetic reactance', units = '' )
        self.add_param('m', val = 3.0, desc = 'input phase', units = 'none') 
        self.add_param('V1', val = 450.0, desc = 'input voltage', units = 'volts') 
        self.add_param('L_s', val = 0.6, desc = 'length of stator' , units = 'meters') 
        self.add_param('V_s', val = 16.31, desc = 'synchronous velocity', units = 'm/s')
        self.add_param('V_r', val = 15.5, desc = 'rotor velocity', units = 'm/s')        
        
        self.add_param('f', val = 60.0, desc ='input frequency', units = 'Hz')
        self.add_param('L', val = 0.0017, desc ='inductance of inductor', units = 'henries')
        self.add_param('c_time', val = 2.0, desc = 'contact time', units = 's')
        self.add_param('mass', val = 15000 , desc = 'mass of pod', units = 'kg')
        
        
        self.add_output('phi', 0.0, desc = 'phase angle', units = 'TBD')
        self.add_output('slip_ratio', 0.0, desc = 'slip ratio', units = 'TBD')
        self.add_output('reactance_of_inductor', 0.0, desc = 'reactance of inductor', units = 'TBD')
        self.add_output('omega', 0.0, desc = 'omega', units = 'TBD')
        self.add_output('thrust', 0.0, desc = 'thrust', units = 'N')
        self.add_output('a', 0.0, desc = 'acceleration', units = 'm/s^2')
        
    def solve_nonlinear(self, params, unknowns, resids):
        R1 = params['R1']
        R2 = params['R2'] 
        #p = params['p']
        P1 = params['P1']
        X_m = params['X_m']
        m = params['m']
        V1 = params['V1']
        L_s = params['L_s']
        V_s = params['V_s']
        V_r = params['V_r']
        
        f = params['f']
        L = params['L']
        
        mass = params['mass']
        c_time = params['c_time']
        
        unknowns['phi'] = self.phase_angle_calc(f, L, R1)
        phi = unknowns['phi']
        
        
        unknowns['slip_ratio'] = self.slip_ratio(V_s, V_r)
        S = unknowns['slip_ratio']
        
        
        unknowns['reactance_of_inductor'] = self.reactance_of_inductor(f,L)
        X_l = unknowns['reactance_of_inductor']
        
        
        unknowns['omega']= self.omega(f)
        w = unknowns['omega']
        
        unknowns['thrust'] = (P1**2*R2*X_m**2*S*(1-S)) / (m*V1**2*numpy.cos(phi)**2*(R2**2+S**2*X_m**2))
        unknowns['a'] = self.acceleration(P1, c_time, mass)
    
    
    def phase_angle_calc(self, f, L, R1):
        phi = numpy.arctan((2*math.pi*f*L)/R1)
        return phi
    
    
    def slip_ratio(self, V_s, V_r):
        slip = (V_s - V_r) / V_s
        return slip
    
    def reactance_of_inductor(self, f, L):
        X_l = 2*math.pi*f*L
        return X_l
        
    def omega(self, f):
    
        w = 2*math.pi*f
        return w

    def acceleration(self,P1, c_time, mass ):
        a = (P1 / (2*mass*c_time))**0.5 
        return a
		
if __name__ == '__main__':
   

    p = Problem(root=Group())
    p.root.add('comp', Thrust())
    p.setup()
    p.root.list_connections()
    p.run()
    
    print 'phase angle : %f' %p['comp.omega']
    print 'slip ratio : %f' %p['comp.slip_ratio']
    print 'reactance of inductor : %f' %p['comp.reactance_of_inductor']
    print 'omega : %f' %p['comp.omega']
    print 'thrust : %f' %p['comp.thrust']
    print 'acceleration : %f' %p['comp.a']
    