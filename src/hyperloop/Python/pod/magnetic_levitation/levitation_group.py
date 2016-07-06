"""
Group containing the breakpointlev.py classes Drag and Mass
"""

from openmdao.api import Group, Problem, IndepVarComp

from Python.pod.magnetic_levitation.breakpoint_levitation import BreakPointDrag, MagMass


class LevGroup(Group):
    def __init__(self):
        super(LevGroup, self).__init__()

        # Creates components of the group.
        self.add('Drag', BreakPointDrag())
        self.add('Mass', MagMass())

if __name__ == "__main__":

    top = Problem()

    root = top.root = Group()

    # Define Parameters
    params = (('m_pod', .375, {'units': 'kg'}), ('l_pod', 25.0, {'units': 'm'}),
              ('w_track', 2.0, {'units': 'm'}), ('vel_b', 23.0, {'units': 'm/2'}),
              ('w_mag', 2.0, {'units': 'm'}))

    root.add('input_vars', IndepVarComp(params))
    root.add('lev', LevGroup())

    root.connect('input_vars.m_pod', 'lev.Drag.m_pod')

    root.connect('input_vars.l_pod', 'lev.Drag.l_pod')
    root.connect('input_vars.l_pod', 'lev.Mass.l_pod')

    root.connect('input_vars.w_mag', 'lev.Drag.w_mag')
    root.connect('input_vars.w_mag', 'lev.Mass.w_mag')

    top.setup()
    top.run()

    # print('l_pod Drag is %f' % top['lev.Drag.l_pod'])
    # print('l_pod Mass is %f' % top['lev.Mass.l_pod'])
    # print('\n')
    # print('w_mag from Drag is %f' % top['lev.Drag.w_mag'])
    # print('w_mag from Mass is %f' % top['lev.Mass.w_mag'])
