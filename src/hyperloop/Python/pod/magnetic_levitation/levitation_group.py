"""
Group containing the breakpointlev.py classes Drag and Mass
"""

from openmdao.api import Group, Problem, IndepVarComp
from breakpoint_levitation import BreakPointDrag, MagMass
from magnetic_drag import MagDrag

class LevGroup(Group):
    def __init__(self):
        super(LevGroup, self).__init__()

        # Creates components of the group.
        self.add('Drag', BreakPointDrag(), promotes=['m_pod', 'w_track', 'l_pod', 'w_mag'])
        self.add('Mass', MagMass(), promotes=['l_pod', 'w_mag', 'm_mag', 'cost'])
        self.add('MDrag', MagDrag(), promotes=['mag_drag'])

        self.connect('Drag.track_res', 'MDrag.track_res')
        self.connect('Drag.track_ind', 'MDrag.track_ind')
        self.connect('Drag.lam', 'MDrag.lam')
        self.connect('Drag.fyu', 'MDrag.fyu')        

if __name__ == "__main__":

    top = Problem()

    root = top.root = Group()

    # Define Parameters
    params = (('m_pod', .375, {'units': 'kg'}), 
    		  ('l_pod', 25.0, {'units': 'm'}),
              ('w_track', 2.0, {'units': 'm'}), 
              ('vel_b', 23.0, {'units': 'm/s'}),
              ('w_mag', 2.0, {'units': 'm'}))

    root.add('input_vars', IndepVarComp(params))
    root.add('lev', LevGroup())

    root.connect('input_vars.m_pod', 'lev.m_pod')
    root.connect('input_vars.l_pod', 'lev.l_pod')
    root.connect('input_vars.w_mag', 'lev.w_mag')
    root.connect('input_vars.vel_b', 'lev.Drag.vel_b')

    top.setup()
    top.run()

    print('Mag_drag %fN' % top['lev.mag_drag'])
