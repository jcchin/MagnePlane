from XDSM import XDSM

opt = 'Optimization'
dat = 'DataInter'

Large="\Large "

def build_cycle(xdsm):

    x.addComp('FlowpathInp', 'Function', Large + r'\TwolineComponent{6em}{Flowpath}{Inputs}')
    x.addComp('Flowpath', 'Analysis', Large + r'Flowpath')
    x.addComp('CompLen', 'Function', Large + r'\TwolineComponent{6em}{Compressor}{Length}')
    x.addComp('CompMass', 'Function', Large + r'\TwolineComponent{6em}{Compressor}{Mass}')

    x.addDep('Flowpath', 'FlowpathInp', dat, r"$P_\text{total}, T_\text{total}, \dot{m}$")
    x.addDep('CompMass', 'Flowpath', dat, r"$\dot{m}, h_\text{in}, h_\text{out}$")
    x.addDep('CompLen', 'Flowpath', dat, r"$h_\text{in}, h_\text{out}$")

x = XDSM()
build_cycle(x)
x.write('cycle',True)
