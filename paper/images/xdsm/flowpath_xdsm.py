from XDSM import XDSM

opt = 'Optimization'
dat = 'DataInter'

Large="\Large "

def build_flowpath(xdsm):

    x.addComp('Fl_Start', 'FunctionGroup', Large + r'Flow Start')
    x.addComp('Inlet', 'FunctionGroup', Large + r'Inlet')
    x.addComp('Compressor', 'AnalysisGroup', Large + r'Compressor')
    x.addComp('Duct', 'FunctionGroup', Large + r'Duct')
    x.addComp('Nozzle', 'FunctionGroup', Large + r'Nozzle')
    x.addComp('Shaft', 'FunctionGroup', Large + r'Shaft')

    x.addDep('Inlet', 'Fl_Start', dat, r"$Fl_\text{total}, Fl_\text{static}$")
    x.addDep('Compressor', 'Inlet', dat, r"$Fl_\text{total}, Fl_\text{static}$")
    x.addDep('Duct','Compressor',  dat, r"$Fl_\text{total}, Fl_\text{static}$")
    x.addDep('Nozzle','Duct', dat, r"$Fl_\text{total}, Fl_\text{static}$")
    x.addDep('Shaft','Compressor', dat, r"$trq$")
    x.addDep('Compressor','Shaft', dat, r"$N_\text{mech}$")

x = XDSM()
build_flowpath(x)
x.write('flowpath',True)
