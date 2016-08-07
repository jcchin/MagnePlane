from XDSM import XDSM

opt = 'Optimization'
dat = 'DataInter'

Large="\Large "

def build_tube(xdsm):

    x.addComp('Temp', 'AnalysisGroup', Large + r'\TwolineComponent{6em}{Tube}{Temperature}')
    x.addComp('PropMech', 'Function', Large + r'\TwolineComponent{6em}{Propulsion}{Mechanics}')
    x.addComp('Vacuum', 'FunctionGroup', Large + r'Vacuum')
    x.addComp('Power', 'Function', Large + r'Tube Power')
    x.addComp('Structure', 'FunctionGroup', Large + r'Structure')

    x.addDep('PropMech', 'Temp', dat, r"$T_\text{tube}$")
    x.addDep('Vacuum', 'Temp', dat, r"$T_\text{tube}$")
    x.addDep('Power', 'Temp', dat, r"$T_\text{tube}$")

    x.addDep('Power', 'PropMech', dat, r"$Pwr_\text{required}$")

    x.addDep('Power', 'Vacuum', dat, r"$E_\text{total}, Pwr_\text{total}$")
    x.addDep('Structure', 'Vacuum', dat, r"$w_\text{total}$")

x = XDSM()
build_tube(x)
x.write('tube',True)
