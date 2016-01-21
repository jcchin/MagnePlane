import numpy as np

from openmdao.core.group import Group, Component, IndepVarComp
from openmdao.solvers.newton import Newton
from openmdao.solvers.scipy_gmres import ScipyGMRES

from pycycle.components import Compressor, Shaft, FlowStart, Inlet, Nozzle, Duct, FlightConditions
from pycycle.species_data import janaf
from pycycle.connect_flow import connect_flow
from pycycle.constants import AIR_FUEL_MIX, AIR_MIX

from openmdao.solvers.ln_gauss_seidel import LinearGaussSeidel
from openmdao.solvers.ln_direct import DirectSolver


class CompressionCycle(Group):
    """A group that models an inlet->compressor->nozzle->shaft"""

    def __init__(self):
        super(CompressionCycle, self).__init__()

        # initiate components
        self.add('fc', FlightConditions())
        #self.add('fl_start', FlowStart(thermo_data=janaf, elements=AIR_MIX))
        self.add('inlet', Inlet(thermo_data=janaf, elements=AIR_MIX))
        self.add('comp', Compressor(thermo_data=janaf, elements=AIR_MIX))
        self.add('duct', Duct(thermo_data=janaf, elements=AIR_MIX))
        self.add('nozzle', Nozzle(thermo_data=janaf, elements=AIR_MIX))
        self.add('shaft', Shaft(1))

        # connect components

        connect_flow(self,'fc.Fl_O', 'inlet.Fl_I')
        connect_flow(self,'inlet.Fl_O', 'comp.Fl_I')
        connect_flow(self,'comp.Fl_O', 'duct.Fl_I')
        connect_flow(self,'duct.Fl_O', 'nozzle.Fl_I')

        self.connect("comp.trq","shaft.trq_0")
        self.connect('shaft.Nmech', 'comp.Nmech')

        #from openmdao.solvers.nl_gauss_seidel import NLGaussSeidel

        #self.ln_solver = DirectSolver()
        #self.ln_solver = LinearGaussSeidel()

        # self.ln_solver = ScipyGMRES()
        # self.ln_solver.options['atol'] = 1e-8
        # self.ln_solver.options['maxiter'] = 100
        # self.ln_solver.options['restart'] = 100
        #self.nl_solver.options['alpha'] = 1.1
        #self.nl_solver.options['solve_subsystems'] = False
        self.nl_solver = Newton()
        self.nl_solver.options['rtol'] = 1.4e-8
        #self.nl_solver.options['rtol'] = 1.4e-8  # This is scipy's newton Tolerance for apples-to-apples
        self.nl_solver.options['maxiter'] = 75
        self.nl_solver.options['iprint'] = 1


if __name__ == "__main__":
    from openmdao.api import Problem
    prob = Problem()
    prob.root = CompressionCycle()

    params = (
        ('P', 17.0, {'units':'psi'}),#('P', 101325.0, {'units':'N/m**2'}), # Pascals
        ('T', 530.0, {'units':'degR'}), # 70 F
        ('W', 1.0, {'units':'lbm/s'}),
        ('PR_design', 2.87),
        ('Ps_exhaust', 10.0, {'units':'lbf/inch**2'}),
        ('alt', 1000.0, {'units':'ft'}),
        ('MN', 0.8)
    )
    prob.root.add('des_vars', IndepVarComp(params))

    #prob.root.connect("des_vars.P", "fl_start.P")
    #prob.root.connect("des_vars.T", "fl_start.T")
    prob.root.connect("des_vars.W", "fc.fs.W")
    #prob.root.connect("des_vars.PR_design", "turb.PR_design")
    prob.root.connect('des_vars.MN', 'fc.MN_target')
    prob.root.connect("des_vars.Ps_exhaust", "nozzle.Ps_exhaust")

    prob.setup(check=True)

    #Flow Start
    prob['des_vars.T'] = 600.
    prob['des_vars.P'] = 30.
    prob['des_vars.W'] = 75.

    # Inlet
    prob['inlet.ram_recovery'] = 1.0

    # #Compressor
    prob['comp.map.PRdes'] = 5.0
    prob['comp.map.effDes'] = 0.92

    #Nozzle
    prob['nozzle.Cfg'] = 0.99
    #prob['nozz.Ps_exhaust'] = 14.7
    prob['nozzle.dPqP'] = 1.0

    # #Shaft
    prob['shaft.Nmech'] = 15000.

    # #Flow End 374.29
    import time
    t = time.time()
    prob.run()
    print time.time() - t
    #prob.check_partial_derivatives()

    # scipy fd check:
    #print approx_fprime(np.array([turb_pr_des]), f, 0.001)
    #
    # our fd check:
    #print prob.calc_gradient(['des_vars.PR_design'], ['shaft.trq_net'], mode='fwd',
    #                              return_format='dict')
    #quit()

    print "comp pwr out: ", prob['comp.power']
    print "comp trq out: ", prob['comp.trq']
    print "net trq: ", prob['shaft.trq_net']
    print
    # print 'resid', prob['pwr_balance.pwr_net']
    print "comp.Fl_O:tot:P", prob['comp.Fl_O:tot:P']
    print "comp.Fl_O:tot:T", prob['comp.Fl_O:tot:T']
    print "comp.Fl_O:tot:h", prob['comp.Fl_O:tot:h']

    exit()