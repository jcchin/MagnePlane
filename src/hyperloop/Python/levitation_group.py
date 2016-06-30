"""
Group containing the breakpointlev.py classes Drag and Mass
"""

from openmdao.api import Group, Problem, IndepVarComp
from hyperloop.Python.breakpoint_levitation import Drag, Mass


class LevGroup(Group):
    def __init__(self):
        super(LevGroup, self).__init__()

        # Creates components of the group.
        self.add('Drag', Drag())
        self.add('Mass', Mass())


if __name__ == "__main__":

    top = Problem()

    root = top.root = Group()

    # Define Parameters
    params = (('mpod', .375, {'units': 'kg'}), ('lpod', 25.0, {'units': 'm'}),
              ('Pc', 2.0, {'units': 'm'}), ('vb', 23.0, {'units': 'm/2'}),
              ('w', 2.0, {'units': 'm'}))

    root.add('input_vars', IndepVarComp(params))
    root.add('lev', LevGroup())

    root.connect('input_vars.mpod', 'lev.Drag.mpod')

    root.connect('input_vars.lpod', 'lev.Drag.lpod')
    root.connect('input_vars.lpod', 'lev.Mass.lpod')

    root.connect('input_vars.w', 'lev.Drag.w')
    root.connect('input_vars.w', 'lev.Mass.w')

    top.setup()
    top.run()

    print('lpod Drag is %f' % top['lev.Drag.lpod'])
    print('lpod Mass is %f' % top['lev.Mass.lpod'])
    print('\n')
    print('w from Drag is %f' % top['lev.Drag.w'])
    print('w from Mass is %f' % top['lev.Mass.w'])
