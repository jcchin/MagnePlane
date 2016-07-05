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

        prob['comp.vel'] = 23
        prob['comp.track_res'] = 0.019269
        prob['comp.track_ind'] = 3.59023e-6
        prob['comp.fyu'] = 29430.0
        prob['comp.lam'] = 0.125658

        prob.run()

        print('magdrag is %f' % prob['comp.mag_drag'])
        assert np.isclose(prob['comp.mag_drag'], 137342.0, rtol=.001)
