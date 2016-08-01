from XDSM import XDSM

opt = 'Optimization'
dat = 'DataInter'

Large="\Large "

def build_steady_state_vacuum(xdsm):

    x.addComp('Fl_Start', 'Function', Large + r'\TwolineComponent{6em}{Flow}{Start}')
    x.addComp('Compressor', 'Analysis', Large + r'Compressor')

    x.addDep('Compressor', 'Fl_Start', dat, r"$Fl_\text{total}, Fl_\text{static}$")

x = XDSM()
build_steady_state_vacuum(x)
x.write('steady_state_vacuum',True)
