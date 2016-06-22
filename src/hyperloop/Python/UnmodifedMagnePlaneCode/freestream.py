''' Freestream component '''

# --- OpenMDAO imports
from openmdao.core.component import Component
from openmdao.core.problem import Problem
from openmdao.core.group import Group

# --- Local Python imports

class Freestream(Component):
    ''' OpenMDAO component for calculating freestream total properties '''

    def __init__(self):
        super(Freestream, self).__init__()

        # -----------------------------------
        # --- Initialize Input Parameters ---
        # -----------------------------------
        self.add_param('M',  0.8,   desc='freestream Mach number', units='m**2')
        self.add_param('Ts', 291.,  desc='freestream static pressure', units='K')
        self.add_param('Ps', 3500., desc='freestream static temperature', units='Pa')
        self.add_param('R', 287.05, desc='specific gas constant')
        self.add_param('G', 1.4,    desc='ratio of specific heats')

        # -----------------------------------
        # --- Initialize Output Parameters ---
        # -----------------------------------
        self.add_output('Ds', 0.0,  desc='freestream static density')
        self.add_output('Pt', 0.0,  desc='freestream total pressure')
        self.add_output('Tt', 0.0,  desc='freestream total temperature')
        self.add_output('Dt', 0.0,  desc='freestream total density')

    def solve_nonlinear(self, p, u, r):

        # ---------------------------------
        # --- Compute Output Parameters ---
        # ---------------------------------
        u['Ds'] = p['Ps'] / (p['R']*p['Ts'])
        u['Pt'] = p['Ps'] / ((1+(p['G']-1)/2*p['M']**2)**(-p['G']/(p['G']-1)))
        u['Tt'] = p['Ts'] / ((1+(p['G']-1)/2*p['M']**2))**-1
        u['Dt'] = u['Pt'] / (p['R']*u['Tt'])

        print("-------------------------------------")
        print(" --- Freestream Static Conditions ---")
        print("-------------------------------------")
        print("MN = %.6f "                         % (p['M']))
        print("Ps = %.3f Pa     :  = %.6f psi"     % (p['Ps'], p['Ps']*0.000145038))
        print("Ts = %.4f K      :  = %.4f R"       % (p['Ts'], p['Ts']*1.8))
        print("Ds = %.6f kg/m^3 :  = %.6f lb/ft^3" % (u['Ds'], u['Ds']*0.0624279606))
        print (" ")
        print("-------------------------------------")
        print(" --- Freestream Total Conditions ----")
        print("-------------------------------------")
        print("Pt = %.3f Pa     :  = %.3f psi"     % (u['Pt'], u['Pt']*0.000145038))
        print("Tt = %.4f K      :  = %.4f R"       % (u['Tt'], u['Tt']*1.8))
        print("Dt = %.6f kg/m^3 :  = %.6f lb/ft^3" % (u['Dt'], u['Dt']*0.0624279606))

if __name__ == "__main__":

    # -------------------------
    # --- Default Test Case ---
    # -------------------------
    p = Problem(root=Group())
    p.root.add('comp', Freestream())
    p.setup()
    p.root.list_connections()
    p.run()

