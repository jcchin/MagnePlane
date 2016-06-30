import pytest
from hyperloop.Python import magnetic_drag
import numpy as np
from openmdao.api import Group, Problem


def create_problem(magdrag):
    root = Group()
    prob = Problem(root)
    prob.root.add('comp', magdrag)
    return prob


class TestVac(object):
    def test_case1_vs_breakpoint(self):

        magdrag = magnetic_drag.MagDrag()

        prob = create_problem(magdrag)

        prob.setup()

        prob['comp.v'] = 23
        prob['comp.R'] = 0.019269
        prob['comp.L'] = 3.59023e-6
        prob['comp.Fyu'] = 29430.0
        prob['comp.lam'] = 0.125658

        prob.run()

        print('magdrag is %f' % prob['comp.magdraglev'])
        assert np.isclose(prob['comp.magdraglev'], 137342.0, rtol=.001)
