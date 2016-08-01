from XDSM import XDSM

opt = 'Optimization'
dat = 'DataInter'

Large="\Large "

def build_levitation(xdsm):

    x.addComp('Drag', 'Function', Large + r'\TwolineComponent{6em}{BreakPoint}{Drag}')
    x.addComp('MDrag', 'Function', Large + r'\TwolineComponent{6em}{Magnetic}{Drag}')
    x.addComp('Mass', 'Function', Large + r'\TwolineComponent{6em}{Magnetic}{Mass}')

    x.addDep('MDrag', 'Drag', dat, r"$R_\text{track}, L_\text{track}, \lambda, w_\text{pod}$")

x = XDSM()
build_levitation(x)
x.write('levitation',True)
