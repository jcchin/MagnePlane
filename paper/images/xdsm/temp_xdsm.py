from XDSM import XDSM

opt = 'Optimization'
dat = 'DataInter'

Large="\Large "

def build_temp(xdsm):

    x.addComp('Solver', 'MDA', Large + 'Solver')
    x.addComp('TempBal', 'Function', Large + '\TwolineComponent{6em}{Temp}{Balance}')
    x.addComp('TubeWallTemp', 'Function', Large + r'\TwolineComponent{6em}{Tube Wall}{Temp}')

    x.addDep('TubeWallTemp', 'Solver', dat, r"$T_\text{boundary}$")
    x.addDep('Solver', 'TempBal', dat, r"$\mathcal{R}(T_\text{boundary})$")
    x.addDep('TempBal', 'TubeWallTemp', dat, r"$T_\text{ss residual}$")

x = XDSM()
build_temp(x)
x.write('temp',True)
