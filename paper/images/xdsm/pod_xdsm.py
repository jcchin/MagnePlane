from XDSM import XDSM

opt = 'Optimization'
dat = 'DataInter'

Large="\Large "

def build_pod(xdsm):

    x.addComp('Drag', 'Function', Large + r'Drag')
    x.addComp('Cycle', 'Analysis', Large + r'Cycle')
    x.addComp('Drivetrain', 'Analysis', Large + r'Drivetrain')
    x.addComp('Geometry', 'Function', Large + r'Geometry')
    x.addComp('Pod Mach', 'Function', Large + r'Pod Mach')
    x.addComp('Mass', 'Function', Large + r'Mass')
    x.addComp('Levitation', 'Function', Large + r'Levitation')

    x.addDep('Geometry', 'Cycle', dat, r"$L_\text{comp}, A_\text{pod duct}$")
    x.addDep('Mass', 'Cycle', dat, r"$m_\text{comp}$")
    x.addDep('Drivetrain', 'Cycle', dat, r"$\tau_\text{comp}, Pwr_\text{comp}$")

    x.addDep('Mass', 'Drivetrain', dat, r"$m_\text{battery}, m_\text{motor}$")
    x.addDep('Geometry', 'Drivetrain', dat, r"$L_\text{battery}, L_\text{motor}$")

    x.addDep('Pod Mach', 'Geometry', dat, r"$A_\text{pod}, L_\text{pod}$")
    x.addDep('Mass', 'Geometry', dat, r"$L_\text{pod}, D_\text{pod}, BF$")
    x.addDep('Levitation', 'Geometry', dat, r"$L_\text{pod}, D_\text{pod}$")

    x.addDep('Levitation', 'Mass', dat, r"$m_\text{pod}$")

x = XDSM()
build_pod(x)
x.write('pod',True)
