from XDSM import XDSM

opt = 'Optimization'
dat = 'DataInter'

Large="\Large "

def build_chemeq(xdsm): 
    x.addComp('solver', 'MDA', Large + 'Solver')

    x.addComp('ChemEq', 'Analysis', Large + r'\TwolineComponent{6em}{Chemical}{Equilibrium}')
    x.addComp('Cp', 'Analysis', Large + '$C_P$')
    x.addComp('Cv', 'Analysis', Large + '$C_V$')
    x.addComp('props', 'Function', Large + r'\TwolineComponent{8.2em}{Thermodynamic}{Properties}')


    x.addDep('ChemEq', 'solver', dat, "$n, \pi$")
    x.addDep('solver', 'ChemEq', dat, r"$\mathcal{R}_\text{Gibbs}, \mathcal{R}_\text{mass}$")
    x.addDep('Cp', 'ChemEq', dat, "$n, b, b_o$")
    x.addDep('Cv', 'ChemEq', dat, "$n, b, b_o$")
    x.addDep('props', 'ChemEq', dat, "$n$")
    x.addDep('props', 'Cp', dat, "$C_P$")
    x.addDep('props', 'Cv', dat, "$C_V$")

# x.addDep('aero', 'prop', dat, "Mass Flow")
# x.addDep('solver', 'aero', dat, "Drag")
# x.addDep('solver', 'prop', dat, "Thrust")
# x.addDep('prop', 'solver', dat, "Throttle")

# x.addDep('aero', 'opt', dat, "OML Shape")
# x.addDep('prop', 'opt', dat, "Engine Size")

# x.addDep('opt', 'prop', dat, "TSFC")


x = XDSM()
build_chemeq(x)
x.write('chem_eq_xdsm',True)

# x = XDSM()
# x.addComp('opt', 'Optimization', Large + 'Optimizer')
# build_chemeq(x)
# x.addDep('ChemEq', 'opt', dat, "$P,\phi$")
# x.addDep('opt', 'props', dat, r"$T, \frac{dT}{dP}, \frac{dT}{d\phi}$")
# x.write('opt_chem_eq_xdsm',True)


