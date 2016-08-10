from XDSM import XDSM

opt = 'Optimization'
dat = 'DataInter'

Large="\Large "

def build_tubepod(xdsm): 

    x.addComp('Solver', 'MDA', Large + 'Solver')

    x.addComp('Pod', 'FunctionGroup', Large + r'Pod')
    x.addComp('Tube', 'FunctionGroup', Large + r'Tube')
    x.addComp('Mission', 'Function', Large + r'Mission')
    x.addComp('Cost', 'Function', Large + r'Ticket Cost')

    x.addDep('Mission', 'Pod', dat, r"$m_\text{pod}, D_\text{mag}, S_\text{ref}, F_\text{gross}, D_\text{ram}, C_\text{D}$")

    x.addDep('Cost', 'Tube', dat, r"\TwolineComponent{11em}{$P_\text{tube}, Cost_\text{land}, Cost_\text{water}$}{$Pwr_\text{prop}, Pwr_\text{vac}$}")
    x.addDep('Cost', 'Pod', dat, r"$Pwr_\text{pod}, D_\text{mag}, S_\text{ref}, C_\text{D}$")
    x.addDep('Cost', 'Mission', dat, r"$t_\text{thrust}, \Delta x_\text{coast}, n_\text{thrust}$")

    x.addDep('Tube', 'Pod', dat, r"\TwolineComponent{14em}{$A_\text{tube}, m_\text{pod}, F_\text{gross}, D_\text{ram}$}{$S_\text{ref}, D_\text{mag}, T_\text{nozzle}, \dot{m}, C_\text{D}$}")

    x.addDep('Pod', 'Tube', dat, r"$T_\text{boundary}$")

    x.addDep('Tube', 'Solver', dat, r"$T_\text{tube}$")
    x.addDep('Solver', 'Pod', dat, r"$\mathcal{R}(T_\text{tube})$")
    x.addDep('Mission', 'Solver', dat, r"$T_\text{tube}$")

x = XDSM()
build_tubepod(x)
x.write('tube_and_pod',True)
