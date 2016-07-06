from __future__ import print_function, division

import unittest

from numpy.testing import assert_almost_equal

from Python.UnmodifedMagnePlaneCode import mission_pointer as mp

SOLVER = 'SNOPT'
NUM_SEG = 10
SEG_NCN = 2


class TestMissionPointer(unittest.TestCase):
    def test_brachistochrone_result(self):
        prob = mp.magneplane_brachistochrone(solver=SOLVER,
                                             num_seg=NUM_SEG,
                                             seg_ncn=SEG_NCN)
        prob.setup()
        prob.run()
        assert_almost_equal(prob['traj0.phase0.tp'], 1.8016, decimal=3)


if __name__ == '__main__':
    unittest.main()
