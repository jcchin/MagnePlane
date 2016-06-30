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
    v : float
        Desired velocity of the pod. Default value is 335.
    R : float
        Resistance of the track. Default value is 3.14e-4.
    L : float
        Inductance of the track. Default value is 3.59e-6.
    Fyu : float
        Levitation force. Default value is 29430.0.
    lam : float
        Wavelength of the Halbach Array. Default value is 0.125658.

    Returns
    -------
    omega : float
        Frequency of Induced current at chosen velocity. Default value is 0.0.
    magdraglev : float
        Magnetic Drag Force from Levitation. Default value is 0.0.
    magdragprop : float
        Magnetic Drag Force from Propulsion (TBD). Default value is 0.0.
    magdrag : float
        Total Magnetic Drag Force. Default value is 0.0.

    Notes
    -----
    [1] Friend, Paul. Magnetic Levitation Train Technology 1. Thesis. Bradley University, 2004. N.p.: n.p., n.d. Print.
    """

    def __init__(self):
        super(MagDrag, self).__init__()

        # Inputs
        self.add_param('v', val=350.0, units='m/s', desc='Desired Velocity')
        self.add_param('R', val=3.14e-4, units='ohm', desc='Track Resistance')
        self.add_param('L', val=3.59023e-6, units='ohm*s', desc='Track Inductance')
        self.add_param('Fyu', val=29430.0, units='N', desc='Levitation Force')
        self.add_param('lam', val=0.125658, units='m', desc='Halbach wavelength')

        # Outputs
        self.add_output('omega', val=0.0, units='rad/s', desc='Frequency')
        self.add_output('magdraglev', val=0.0, units='N', desc='Magnetic Drag from Levitation')
        self.add_output('magdragprop', val=0.0, units='N', desc='Magnetic Drag from Propulsion')
        self.add_output('magdrag', val=0.0, units='N', desc='Total Magnetic Drag')

    def solve_nonlinear(self, params, unknowns, resids):

        v = params['v']  # Desired Velocity for Drag Value
        R = params['R']  # Track Resistance
        L = params['L']  # Track Inductance
        Fyu = params['Fyu']  # Levitation Force Required
        lam = params['lam']  # Halbach Array Wavelength

        omega = 2*pi*v/lam  # Frequency of Induced Current
        magdraglev = R*Fyu/(omega*L)  # Magnetic Drag from Levitation
        magdragprop = 0  # Magnetic Drag from Propulsion (TBD)
        magdrag = magdraglev + magdragprop  # Total Magnetic Drag

        unknowns['omega'] = omega
        unknowns['magdraglev'] = magdraglev
        unknowns['magdragprop'] = magdragprop
        unknowns['magdrag'] = magdrag

if __name__ == "__main__":

    top = Problem()
    root = top.root = Group()

    root.add('p', MagDrag())

    top.setup(check=True)

    top.run()

    print('Magnetic Drag from Levitation is %2.2fN' % top['p.magdraglev'])
    print('Magnetic Drag from Propulsion is %2.2fN' % top['p.magdragprop'])
    print('\n')
    print('Total Magnetic Drag is %2.2fN' % top['p.magdrag'])
