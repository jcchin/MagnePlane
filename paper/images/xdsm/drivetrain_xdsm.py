from XDSM import XDSM

opt = 'Optimization'
dat = 'DataInter'

Large="\Large "

def build_drivetrain(xdsm):

    x.addComp('Motor', 'Analysis', Large + r'Motor')
    x.addComp('Inverter', 'Function', Large + r'Inverter')
    x.addComp('Battery', 'Function', Large + r'Battery')

    x.addDep('Inverter', 'Motor', dat, r"$I_\text{phase}, V_\text{phase}, f$")
    x.addDep('Battery', 'Inverter', dat, r"$I_\text{input}, Pwr_\text{input}$")

x = XDSM()
build_drivetrain(x)
x.write('drivetrain',True)
