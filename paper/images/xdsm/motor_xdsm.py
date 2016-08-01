from XDSM import XDSM

opt = 'Optimization'
dat = 'DataInter'

Large="\Large "

def build_motor(xdsm):

    x.addComp('MotorSize', 'Function', Large + r'Motor Size')
    x.addComp('Solver', 'MDA', Large + 'Solver')
    x.addComp('MotorBalance', 'Function', Large + r'Motor Balance')
    x.addComp('Motor', 'Function', Large + r'Motor')

    x.addDep('Motor', 'MotorSize', dat, r"\TwolineComponent{16em}{$\tau_\text{max}, Pwr_\text{mech}, \omega_\text{operating}$}{$Pwr_\text{iron loss}, R_\text{winding}, Pwr_\text{windage loss}$}")
    x.addDep('MotorBalance', 'Motor', dat, r"$I, V, Pwr_\text{motor input}$")
    x.addDep('Motor', 'Solver', dat, r"$I_\text{no load}$")
    x.addDep('Solver', 'MotorBalance', dat, r"$\mathcal{R}(I_\text{no load})$")

x = XDSM()
build_motor(x)
x.write('motor',True)
