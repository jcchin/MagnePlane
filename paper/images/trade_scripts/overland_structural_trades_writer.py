import numpy as np
import matplotlib.pylab as plt
from openmdao.api import Group, Problem, IndepVarComp, ExecComp, ScipyOptimizer

from hyperloop.Python import structural_optimization

# def create_problem(component):
#     root = Group()
#     prob = Problem(root)
#     prob.root.add('comp', component)
#     return prob

# class PressureTradeStudy(object):
#     def test_case1_vs_npss(self):

#         component = tube_and_pod.TubeAndPod()
#         prob = create_problem(component)

if __name__ == '__main__':
	top = Problem()
	root = top.root = Group()

	params = (#('r', 1.1, {'units': 'm'}),
				('tube_area', 53.134589, {'units': 'm**2'}),
				('t', 5.0, {'units': 'm'}),
				('r_pylon', 1.1, {'units': 'm'}),
				('Su_tube', 152.0e6, {'units': 'Pa'}),
				('sf', 1.5),
				('p_ambient', 850.0, {'units': 'Pa'}),
				('p_tunnel', 101300.0, {'units': 'Pa'}),
				('v_tube', .3),
				('rho_tube', 7820.0, {'units': 'kg/m**3'}),
				('rho_pylon', 2400.0, {'units': 'Pa'}),
				('Su_pylon', 40.0e6, {'units': 'Pa'}),
				('E_pylon', 41.0e9, {'units': 'Pa'}),
				('h', 10.0, {'units': 'm'}),
				('m_pod', 3100.0, {'units': 'kg'})
				)
	root.add('input_vars', IndepVarComp(params))
	root.add('p', structural_optimization.StructuralOptimization())

	root.add('con1', ExecComp('c1 = ((Su_tube/sf) - von_mises)'))  #Impose yield stress constraint for tube
	root.add('con2', ExecComp('c2 = t - t_crit'))  #Impose buckling constraint for tube dx = ((pi**3)*E_pylon*(r_pylon**4))/(8*(h**2)*rho_tube*pi*(((r+t)**2)-(r**2))*g)

	#root.connect('input_vars.r', 'p.r')
	root.connect('input_vars.tube_area', 'p.tube_area')
	root.connect('input_vars.t', 'p.t')
	root.connect('input_vars.r_pylon', 'p.r_pylon')

	root.connect('input_vars.Su_tube', 'con1.Su_tube')
	root.connect('input_vars.sf', 'con1.sf')
	root.connect('p.von_mises', 'con1.von_mises')

	root.connect('input_vars.t', 'con2.t')
	root.connect('p.t_crit', 'con2.t_crit')

	root.p.deriv_options['type'] = "cs"
	# root.p.deriv_options['form'] = 'forward'
	root.p.deriv_options['step_size'] = 1.0e-10

	top.driver = ScipyOptimizer()
	top.driver.options['optimizer'] = 'SLSQP'

	top.driver.add_desvar('input_vars.t', lower=.001, scaler=100.0)
	top.driver.add_desvar('input_vars.r_pylon', lower=.1)
	top.driver.add_objective('p.total_material_cost', scaler = 1.0e-4)
	top.driver.add_constraint('con1.c1', lower=0.0, scaler=1000.0)
	top.driver.add_constraint('con2.c2', lower=0.0)

	top.setup()

	top['p.p_tunnel'] = 850.0
	# top['p.m_pod']= 10000.0
	top['p.h'] = 10.0

	m_pod = np.linspace(10000.0, 20000, num = 3)
	A_tube = np.linspace(20.0, 50.0, num = 30)

	dx = np.zeros((len(m_pod), len(A_tube)))
	t_tube = np.zeros((len(m_pod), len(A_tube)))
	r_pylon = np.zeros((len(m_pod), len(A_tube)))
	cost = np.zeros((1, len(A_tube)))

	for i in range(len(A_tube)):
		for j in range(len(m_pod)):
			top['input_vars.tube_area'] = A_tube[i]
			top['p.m_pod'] = m_pod[j]

			top.run()

			dx[j,i] = top['p.dx']
			t_tube[j,i] = top['p.t']
			r_pylon[j,i] = top['p.r_pylon']
		cost[0,i] = top['p.total_material_cost']

	np.savetxt('../../../paper/images/data_files/overland_structural_trades/m_pod.txt', m_pod, fmt = '%f', delimiter = '\t', newline = '\r\n')
	np.savetxt('../../../paper/images/data_files/overland_structural_trades/A_tube.txt', A_tube, fmt = '%f', delimiter = '\t', newline = '\r\n')
	np.savetxt('../../../paper/images/data_files/overland_structural_trades/dx.txt', dx, fmt = '%f', delimiter = '\t', newline = '\r\n')
	np.savetxt('../../../paper/images/data_files/overland_structural_trades/cost.txt', cost, fmt = '%f', delimiter = '\t', newline = '\r\n')
