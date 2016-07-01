"""
Magnetic Drag Calculation.
Default track and magnet parameters taken from breakpointlev.py.
Calculates Magnetic Drag at set velocity desired with given parameters.
"""

from math import pi
from openmdao.api import Group, Component, Problem


class MagDrag(Component):
    """
    Params
    ------
    vel : float
        Desired velocity of the pod. Default value is 335.
    track_res : float
        Resistance of the track. Default value is 3.14e-4.
    track_ind : float
        Inductance of the track. Default value is 3.59e-6.
    fyu : float
        Levitation force. Default value is 29430.0.
    lam : float
        Wavelength of the Halbach Array. Default value is 0.125658.

    Returns
    -------
    omega : float
        Frequency of Induced current at chosen velocity. Default value is 0.0.
    mag_drag_lev : float
        Magnetic Drag Force from Levitation. Default value is 0.0.
    mag_drag_prop : float
        Magnetic Drag Force from Propulsion (TBD). Default value is 0.0.
    mag_drag : float
        Total Magnetic Drag Force. Default value is 0.0.

    Notes
    -----
    [1] Friend, Paul. Magnetic Levitation Train Technology 1. Thesis.
        Bradley University, 2004. N.p.: n.p., n.d. Print.
    """

    def __init__(self):
        super(MagDrag, self).__init__()

        # Inputs
        self.add_param('vel', val=350.0, units='m/s', desc='Desired Velocity')
        self.add_param('track_res', val=3.14e-4, units='ohm', desc='Track Resistance')
        self.add_param('track_ind',
                       val=3.59023e-6,
                       units='ohm*s',
                       desc='Track Inductance')
        self.add_param('fyu', val=29430.0, units='N', desc='Levitation Force')
        self.add_param('lam',
                       val=0.125658,
                       units='m',
                       desc='Halbach wavelength')

        # Outputs
        self.add_output('omega', val=0.0, units='rad/s', desc='Frequency')
        self.add_output('mag_drag_lev',
                        val=0.0,
                        units='N',
                        desc='Magnetic Drag from Levitation')
        self.add_output('mag_drag_prop',
                        val=0.0,
                        units='N',
                        desc='Magnetic Drag from Propulsion')
        self.add_output('mag_drag',
                        val=0.0,
                        units='N',
                        desc='Total Magnetic Drag')

    def solve_nonlinear(self, params, unknowns, resids):

        vel = params['vel']  # Desired Velocity for Drag Value
        track_res = params['track_res']  # Track Resistance
        track_ind = params['track_ind']  # Track Inductance
        fyu = params['fyu']  # Levitation Force Required
        lam = params['lam']  # Halbach Array Wavelength

        omega = 2 * pi * vel / lam  # Frequency of Induced Current
        mag_drag_lev = track_res * fyu / (omega * track_ind)  # Magnetic Drag from Levitation
        mag_drag_prop = 0  # Magnetic Drag from Propulsion (TBD)
        mag_drag = mag_drag_lev + mag_drag_prop  # Total Magnetic Drag

        unknowns['omega'] = omega
        unknowns['mag_drag_lev'] = mag_drag_lev
        unknowns['mag_drag_prop'] = mag_drag_prop
        unknowns['mag_drag'] = mag_drag


if __name__ == "__main__":

    top = Problem()
    root = top.root = Group()

    root.add('p', MagDrag())

    top.setup(check=True)

    top.run()

    # print('Magnetic Drag from Levitation is %2.2fN' % top['p.mag_drag_lev'])
    # print('Magnetic Drag from Propulsion is %2.2fN' % top['p.mag_drag_prop'])
    # print('\n')
    # print('Total Magnetic Drag is %2.2fN' % top['p.mag_drag'])
