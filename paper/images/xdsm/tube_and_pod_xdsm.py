from XDSM import XDSM

opt = 'Optimization'
dat = 'DataInter'

Large="\Large "

def build_tubepod(xdsm): 
    x.addComp('solver', 'MDA', Large + 'Solver')

    x.addComp('Tube', 'Function', Large + r'Tube')
    x.addComp('Pod', 'Function', Large + r'Pod')

    x.addDep('Tube', 'Pod', dat, "$n, b, b_o$")
    x.addDep('Pod', 'Tube', dat, "$n, b, b_o$")

x = XDSM()
build_tubepod(x)
x.write('tube_and_pod',True)
