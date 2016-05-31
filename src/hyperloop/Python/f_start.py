''' Freestream component '''

# --- OpenMDAO imports
from openmdao.core.problem import Problem
from openmdao.core.group import Group
from openmdao.core.component import Component
from pycycle.components import FlowStart
from pycycle import species_data
from pycycle.constants import AIR_MIX
from openmdao.components.indep_var_comp import IndepVarComp



if __name__ == "__main__":
    # -------------------------
    # --- Default Test Case ---
    # -------------------------
    p1 = Problem()
    p1.root = Group()
    #p1.root.add("freestream",Freestream())
    p1.root.add("fs",FlowStart())

    params = (
        ('P', 0.5, {"units" : "psi"}),
        ('T', 291.0, {"units" : "K"}),
        ('MN', 0.8),
    )
    p1.root.add('des_vars', IndepVarComp(params))
    p1.root.connect('des_vars.P', 'fs.P')
    #p1.root.connect('des_vars.MN', 'fs.MN')
    p1.root.connect('des_vars.T', 'fs.T')

    p1.setup(check=False)
    p1.run()
    p1.root.list_connections()
    print("total P: ", p1["fs.Fl_O:tot:P"])
    print("static P: ", p1["fs.Fl_O:stat:P"])